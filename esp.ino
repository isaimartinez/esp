#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
// Crear tres instancias de I2C
TwoWire I2CSensors = TwoWire(0);
TwoWire I2CDisplay = TwoWire(1);
TwoWire I2CMax30100 = TwoWire(2);
#include <TinyGPS++.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MAX30100_PulseOximeter.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

// WiFi credentials
const char* ssid = "ESP32_AP";
const char* password = "12345678";

// Create objects
WebServer server(80);
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;
PulseOximeter pox;

// I2C pins configuration
#define I2C1_SDA 21
#define I2C1_SCL 22
#define I2C2_SDA 16  // OLED display
#define I2C2_SCL 17  // OLED display
#define I2C3_SDA 15   // MAX30100
#define I2C3_SCL 4   // MAX30100
#define I2C_FREQ 50000  // Reduced from 100000 to 50000 for more reliable communication

// OLED Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1  // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  // Common OLED display address
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2CDisplay, OLED_RESET);

// GPS module connected to Serial2
// Note: We need to change these pins as they're used for I2C2
#define GPS_RX 13
#define GPS_TX 14
#define GPS_BAUD 9600

// Create an instance of HardwareSerial for GPS
HardwareSerial gpsSerial(2);

// Data management structure to centralize sensor readings
struct SensorData {
  // GPS data
  bool gpsConnected;
  float latitude;
  float longitude;
  float altitude;
  float speed;
  int satellites;
  
  // MPU6050 data
  bool mpuConnected;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float temperature;
  
  // MAX30100 data
  bool maxConnected;
  float heartRate;
  float spO2;
  bool heartbeat;
  
  // OLED status
  bool oledConnected;
  
  // Timestamp of last update
  unsigned long lastUpdate;
} sensorData;

// Variables to track sensor states
unsigned long lastHeartbeatDetection = 0;
unsigned long lastLogTime = 0;
const unsigned long LOG_INTERVAL = 1000; // Log every second
const unsigned long UPDATE_INTERVAL = 500; // Update sensors every 500ms

// Callback for pulse detection
void onBeatDetected() {
  lastHeartbeatDetection = millis();
  sensorData.heartbeat = true;
  Serial.println("♥ Heartbeat detected!");
}

// Debugging flags
#define I2C_DEBUG_MODE 1      // Set to 1 to enable detailed I2C debugging output
#define USE_MPU_RESET_PIN 0   // Set to 1 if you have connected the MPU6050 AD0 pin to a GPIO for reset
#define MPU_RESET_PIN 19      // GPIO pin connected to the AD0 of MPU6050 (if USE_MPU_RESET_PIN is 1)

// Function to scan all I2C addresses on a specific Wire instance
void scanI2CDevices(TwoWire &wire, const char* busName) {
  byte error, address;
  int nDevices = 0;
  
  Serial.print("Scanning for I2C devices on ");
  Serial.print(busName);
  Serial.println("...");
  
  for(address = 1; address < 127; address++) {
    wire.beginTransmission(address);
    error = wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.print(" on ");
      Serial.print(busName);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }    
  }
  
  if (nDevices == 0) {
    Serial.print("No I2C devices found on ");
    Serial.print(busName);
    Serial.println("\n");
  } else {
    Serial.print("Found ");
    Serial.print(nDevices);
    Serial.print(" device(s) on ");
    Serial.print(busName);
    Serial.println("\n");
  }
}

// Function to check if an I2C device is connected at a specific address on specific bus
bool checkI2CDevice(TwoWire &wire, byte address, const char* busName) {
  if (I2C_DEBUG_MODE) {
    Serial.printf("Testing connection to device at address 0x%02X on %s...\n", address, busName);
  }
  
  wire.beginTransmission(address);
  byte error = wire.endTransmission();
  
  if (I2C_DEBUG_MODE) {
    Serial.print("I2C Device 0x");
    Serial.print(address, HEX);
    Serial.print(" on ");
    Serial.print(busName);
    Serial.print(": ");
  }
  
  if (error == 0) {
    if (I2C_DEBUG_MODE) {
      Serial.println("Connected");
    }
    return true;
  } else if (error == 1) {
    if (I2C_DEBUG_MODE) {
      Serial.println("Data too long to fit in transmit buffer");
    }
  } else if (error == 2) {
    if (I2C_DEBUG_MODE) {
      Serial.println("Received NACK on transmit of address");
    }
  } else if (error == 3) {
    if (I2C_DEBUG_MODE) {
      Serial.println("Received NACK on transmit of data");
    }
  } else if (error == 4) {
    if (I2C_DEBUG_MODE) {
      Serial.println("Other error");
    }
  } else {
    if (I2C_DEBUG_MODE) {
      Serial.println("Unknown error");
    }
  }
  return false;
}

// Function to reset I2C bus by toggling SCL line
bool resetI2CBus(int sdaPin, int sclPin) {
  Serial.printf("Attempting I2C bus reset on pins SDA=%d, SCL=%d\n", sdaPin, sclPin);
  
  // Release the SDA and SCL lines and enable pull-ups
  pinMode(sdaPin, INPUT_PULLUP);
  pinMode(sclPin, INPUT_PULLUP);
  delay(10);
  
  // Check if SDA is high - if not there is a problem
  if (digitalRead(sdaPin) == LOW) {
    Serial.println("SDA line is stuck LOW, attempting recovery...");
    
    // Clock through a few cycles to see if we can get SDA high
    pinMode(sclPin, OUTPUT);
    
    for (int i = 0; i < 10; i++) {
      digitalWrite(sclPin, HIGH);
      delayMicroseconds(5);
      digitalWrite(sclPin, LOW);
      delayMicroseconds(5);
    }
    
    // Last clock high to prepare stop condition
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(5);
    
    // Set SDA as output to prepare for stop
    pinMode(sdaPin, OUTPUT);
    digitalWrite(sdaPin, LOW);
    delayMicroseconds(5);
    
    // Generate stop condition by releasing SDA while SCL is HIGH
    digitalWrite(sdaPin, HIGH);
    delayMicroseconds(5);
    
    // Return pins to input mode with pull-ups
    pinMode(sdaPin, INPUT_PULLUP);
    pinMode(sclPin, INPUT_PULLUP);
    
    // Verify SDA is now HIGH
    if (digitalRead(sdaPin) == HIGH) {
      Serial.println("SDA line successfully recovered!");
      return true;
    } else {
      Serial.println("SDA line recovery failed!");
      return false;
    }
  } else {
    Serial.println("SDA line is HIGH, bus is free");
    return true;
  }
}

// Function to check if MPU6050 data is valid (not stuck or unchanging)
bool isMPUDataValid(float accelX, float accelY, float accelZ) {
  // Check if values are not exactly zero (which would be suspicious)
  if (accelX == 0.0 && accelY == 0.0 && accelZ == 0.0) {
    return false;
  }
  
  // Check if acceleration z is reasonable (should be around 9.8 m/s² if sensor is flat)
  // Allow a range of ±5 m/s² to account for different orientations
  if (abs(accelZ) < 5.0 || abs(accelZ) > 15.0) {
    // If Z acceleration is way off from expected gravity, check if the total acceleration
    // magnitude is still close to 9.8 m/s² (in case the sensor is oriented differently)
    float totalAccel = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
    if (totalAccel < 8.0 || totalAccel > 12.0) {
      return false;
    }
  }
  
  return true;
}

// Function to initialize I2C buses with proper configuration
void initializeI2CBuses() {
  // Record the starting time for I2C initialization
  unsigned long i2cStartTime = millis();
  Serial.println("\n--- I2C Bus Initialization ---");
  
  // First, try to reset all I2C buses
  resetI2CBus(I2C1_SDA, I2C1_SCL);
  resetI2CBus(I2C2_SDA, I2C2_SCL);
  resetI2CBus(I2C3_SDA, I2C3_SCL);
  
  // Initialize first I2C bus for sensors
  if (I2C_DEBUG_MODE) {
    Serial.printf("Setting up sensors I2C bus on SDA=%d, SCL=%d at %d Hz\n", I2C1_SDA, I2C1_SCL, I2C_FREQ);
  }
  I2CSensors.begin(I2C1_SDA, I2C1_SCL, I2C_FREQ);
  I2CSensors.setTimeout(1000);
  
  // Enable internal pull-ups for SDA and SCL lines
  pinMode(I2C1_SDA, INPUT_PULLUP);
  pinMode(I2C1_SCL, INPUT_PULLUP);
  
  if (I2C_DEBUG_MODE) {
    Serial.printf("SDA1 pin status: %d, SCL1 pin status: %d\n", digitalRead(I2C1_SDA), digitalRead(I2C1_SCL));
  }
  Serial.printf("I2C Sensors bus initialized on pins SDA=%d, SCL=%d at %d Hz with pull-up resistors\n", I2C1_SDA, I2C1_SCL, I2C_FREQ);
  
  // Initialize second I2C bus for OLED display
  if (I2C_DEBUG_MODE) {
    Serial.printf("Setting up display I2C bus on SDA=%d, SCL=%d at %d Hz\n", I2C2_SDA, I2C2_SCL, I2C_FREQ);
  }
  I2CDisplay.begin(I2C2_SDA, I2C2_SCL, I2C_FREQ);
  I2CDisplay.setTimeout(1000);
  
  // Enable internal pull-ups for SDA and SCL lines
  pinMode(I2C2_SDA, INPUT_PULLUP);
  pinMode(I2C2_SCL, INPUT_PULLUP);
  
  if (I2C_DEBUG_MODE) {
    Serial.printf("SDA2 pin status: %d, SCL2 pin status: %d\n", digitalRead(I2C2_SDA), digitalRead(I2C2_SCL));
  }
  Serial.printf("I2C Display bus initialized on pins SDA=%d, SCL=%d at %d Hz with pull-up resistors\n", I2C2_SDA, I2C2_SCL, I2C_FREQ);
  
  // Initialize third I2C bus for MAX30100
  if (I2C_DEBUG_MODE) {
    Serial.printf("Setting up MAX30100 I2C bus on SDA=%d, SCL=%d at %d Hz\n", I2C3_SDA, I2C3_SCL, I2C_FREQ);
  }
  I2CMax30100.begin(I2C3_SDA, I2C3_SCL, I2C_FREQ);
  I2CMax30100.setTimeout(1000);
  
  // Enable internal pull-ups for SDA and SCL lines
  pinMode(I2C3_SDA, INPUT_PULLUP);
  pinMode(I2C3_SCL, INPUT_PULLUP);
  
  if (I2C_DEBUG_MODE) {
    Serial.printf("SDA3 pin status: %d, SCL3 pin status: %d\n", digitalRead(I2C3_SDA), digitalRead(I2C3_SCL));
  }
  Serial.printf("I2C MAX30100 bus initialized on pins SDA=%d, SCL=%d at %d Hz with pull-up resistors\n", I2C3_SDA, I2C3_SCL, I2C_FREQ);
  
  // Add a delay to stabilize I2C
  delay(200);
  
  // Calculate time taken for I2C initialization
  if (I2C_DEBUG_MODE) {
    Serial.printf("I2C initialization took %d ms\n", millis() - i2cStartTime);
  }
  
  // Scan for I2C devices on all buses
  scanI2CDevices(I2CSensors, "Sensors Bus");
  scanI2CDevices(I2CDisplay, "Display Bus");
  scanI2CDevices(I2CMax30100, "MAX30100 Bus");
}

// Function to initialize MPU6050
bool initializeMPU6050() {
  Serial.println("\n--- MPU6050 Initialization ---");
  
  // Try a hardware reset of the MPU6050 if configured
  #if USE_MPU_RESET_PIN
    Serial.printf("Performing hardware reset of MPU6050 using pin %d\n", MPU_RESET_PIN);
    pinMode(MPU_RESET_PIN, OUTPUT);
    digitalWrite(MPU_RESET_PIN, LOW);
    delay(100);
    digitalWrite(MPU_RESET_PIN, HIGH);
    delay(100);
    Serial.println("Performed hardware reset of MPU6050");
  #else
    Serial.println("MPU6050 hardware reset disabled in configuration");
  #endif
  
  // Reset the I2C bus before trying to initialize the MPU
  resetI2CBus(I2C1_SDA, I2C1_SCL);
  delay(50);
  
  Serial.println("Checking for MPU6050 on Sensors Bus...");
  
  // Try both possible MPU6050 addresses
  bool mpuFound = false;
  byte mpuAddresses[] = {0x68, 0x69};
  
  for (int i = 0; i < 2 && !mpuFound; i++) {
    if (checkI2CDevice(I2CSensors, mpuAddresses[i], "Sensors Bus")) {
      Serial.printf("MPU6050 found at address 0x%02X, initializing...\n", mpuAddresses[i]);
      
      I2CSensors.setTimeout(1000);
      
      // Multiple initialization attempts
      int attempts = 0;
      bool success = false;
      
      while (attempts < 3 && !success) {
        if (mpu.begin(mpuAddresses[i], &I2CSensors, 1)) {
          Serial.println("MPU6050 initialized successfully");
          
          // Set up the accelerometer range
          mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
          Serial.println("Accelerometer range set to ±8G");
          
          // Set up the gyroscope range
          mpu.setGyroRange(MPU6050_RANGE_500_DEG);
          Serial.println("Gyroscope range set to ±500°/s");
          
          // Set filter bandwidth
          mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
          Serial.println("Filter bandwidth set to 21Hz");
          
          sensorData.mpuConnected = true;
          mpuFound = true;
          success = true;
          
          // Verify we can read values from the MPU
          sensors_event_t a, g, temp;
          if (mpu.getEvent(&a, &g, &temp)) {
            Serial.println("Successfully read values from MPU6050:");
            Serial.printf("Accel: X=%.2f, Y=%.2f, Z=%.2f m/s²\n", a.acceleration.x, a.acceleration.y, a.acceleration.z);
            Serial.printf("Gyro: X=%.2f, Y=%.2f, Z=%.2f rad/s\n", g.gyro.x, g.gyro.y, g.gyro.z);
            Serial.printf("Temp: %.2f°C\n", temp.temperature);
            
            // Check data validity
            if (isMPUDataValid(a.acceleration.x, a.acceleration.y, a.acceleration.z)) {
              Serial.println("MPU6050 data appears valid");
            } else {
              Serial.println("WARNING: MPU6050 data may not be valid!");
            }
          } else {
            Serial.println("WARNING: MPU6050 initialization succeeded but cannot read values!");
            success = false;
          }
        } else {
          attempts++;
          Serial.printf("MPU6050 initialization attempt %d failed\n", attempts);
          delay(100);
        }
      }
      
      if (!success) {
        Serial.println("MPU6050 initialization failed after multiple attempts");
      } else {
        return true;
      }
    }
  }
  
  if (!mpuFound) {
    Serial.println("MPU6050 not found on either address 0x68 or 0x69");
  }
  
  return false;
}

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  Serial.println("\n\n");
  Serial.println("==============================================");
  Serial.println("ESP32 Sensor Web Server - Starting up...");
  Serial.println("==============================================");

  // Initialize data structure
  initSensorData();

  // Initialize I2C buses
  initializeI2CBuses();
  
  // Initialize MPU6050
  bool mpuInitialized = initializeMPU6050();
  
  // Initialize OLED display first
  Serial.println("\n--- OLED Display Initialization ---");
  // Try common OLED addresses
  byte oledAddresses[] = {0x3C, 0x3D, 0x78, 0x7A};
  
  sensorData.oledConnected = false;
  for (int i = 0; i < 4 && !sensorData.oledConnected; i++) {
    Serial.printf("Trying OLED at address 0x%02X on Display Bus... ", oledAddresses[i]);
    if (display.begin(SSD1306_SWITCHCAPVCC, oledAddresses[i])) {
      Serial.println("SUCCESS!");
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("ESP32 Sensor Hub");
      display.println("Initializing...");
      display.display();
      sensorData.oledConnected = true;
      Serial.printf("OLED connected at 0x%02X\n", oledAddresses[i]);
    } else {
      Serial.println("Failed");
    }
  }
  
  if (!sensorData.oledConnected) {
    Serial.println("Could not find an OLED display");
  }
  
  // Initialize MAX30100 on I2CMax30100 bus
  Serial.println("\n--- MAX30100 Initialization ---");
  Serial.print("Initializing pulse oximeter on MAX30100 Bus...");
  
  // Necesitamos asignar el bus I2CMax30100 a la variable Wire que usa internamente MAX30100
  // Esto es un hack porque la biblioteca MAX30100 no permite especificar qué bus I2C usar
  Wire = I2CMax30100;
  
  // Initialize the PulseOximeter instance and handle failures
  if (!pox.begin()) {
    Serial.println("FAILED");
    Serial.println("MAX30100 initialization failed - check wiring or I2C address");
    sensorData.maxConnected = false;
  } else {
    Serial.println("SUCCESS");
    Serial.println("MAX30100 initialized successfully");
    // Register callback for the beat detection
    pox.setOnBeatDetectedCallback(onBeatDetected);
    Serial.println("Heartbeat detection callback set");
    sensorData.maxConnected = true;
  }
  
  // Initialize GPS
  Serial.println("\n--- GPS Initialization ---");
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.print("GPS initialized on pins RX:");
  Serial.print(GPS_RX);
  Serial.print(", TX:");
  Serial.println(GPS_TX);
  sensorData.gpsConnected = true;
  
  // Set up WiFi Access Point
  Serial.println("\n--- WiFi AP Setup ---");
  Serial.print("Setting up Access Point with SSID: ");
  Serial.println(ssid);
  WiFi.softAP(ssid, password);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  // Set up web server routes
  Serial.println("\n--- Web Server Setup ---");
  server.on("/", handleRoot);
  Serial.println("Route '/' registered");
  server.on("/data", handleData);
  Serial.println("Route '/data' registered");
  server.on("/update", HTTP_GET, []() {
    server.send(200, "application/json", getJsonData());
  });
  Serial.println("Route '/update' registered");
  
  // Start server
  server.begin();
  Serial.println("HTTP server started");
  
  Serial.println("\n==============================================");
  Serial.println("System initialization complete!");
  Serial.println("Sensor Status:");
  Serial.print("- GPS: ");
  Serial.println(sensorData.gpsConnected ? "Connected" : "Not connected");
  Serial.print("- MPU6050: ");
  Serial.println(sensorData.mpuConnected ? "Connected" : "Not connected");
  Serial.print("- MAX30100: ");
  Serial.println(sensorData.maxConnected ? "Connected" : "Not connected");
  Serial.print("- OLED: ");
  Serial.println(sensorData.oledConnected ? "Connected" : "Not connected");
  Serial.println("==============================================\n");
}

// Initialize the sensor data structure
void initSensorData() {
  sensorData.gpsConnected = false;
  sensorData.latitude = 0;
  sensorData.longitude = 0;
  sensorData.altitude = 0;
  sensorData.speed = 0;
  sensorData.satellites = 0;
  
  sensorData.mpuConnected = false;
  sensorData.accelX = 0;
  sensorData.accelY = 0;
  sensorData.accelZ = 0;
  sensorData.gyroX = 0;
  sensorData.gyroY = 0;
  sensorData.gyroZ = 0;
  sensorData.temperature = 0;
  
  sensorData.maxConnected = false;
  sensorData.heartRate = 0;
  sensorData.spO2 = 0;
  sensorData.heartbeat = false;
  
  sensorData.oledConnected = false;
  sensorData.lastUpdate = 0;
}

// Function to update OLED display
void updateOLEDDisplay() {
  if (!sensorData.oledConnected) return;
  
  // El display usa I2CDisplay, asegurar usar el bus correcto
  Wire = I2CDisplay;
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Title
  display.println("ESP32 Sensor Hub");
  display.println("---------------");
  
  // GPS Status
  display.print("GPS: ");
  if (sensorData.gpsConnected) {
    display.print("OK ");
    display.print(sensorData.satellites);
    display.println(" sats");
    display.print("Lat:");
    display.println(sensorData.latitude, 4);
    display.print("Lng:");
    display.println(sensorData.longitude, 4);
  } else {
    display.println("Disconnected");
  }
  
  // MPU6050 Status
  display.print("MPU: ");
  if (sensorData.mpuConnected) {
    display.println("OK");
    display.print("Acc:");
    display.print(sensorData.accelX, 1);
    display.print(",");
    display.print(sensorData.accelY, 1);
    display.print(",");
    display.println(sensorData.accelZ, 1);
  } else {
    display.println("Disconnected");
  }
  
  // MAX30100 Status
  display.print("HR: ");
  if (sensorData.maxConnected) {
    display.print(sensorData.heartRate, 0);
    display.print("BPM SpO2:");
    display.print(sensorData.spO2, 0);
    display.println("%");
  } else {
    display.println("Disconnected");
  }
  
  display.display();
}

void loop() {
  server.handleClient();
  
  // Update all sensor data
  updateAllSensors();
  
  // Only update MAX30100 if connected - this needs frequent polling
  if (sensorData.maxConnected) {
    // El MAX30100 usa internamente Wire, aseguramos que esté configurado al bus correcto
    Wire = I2CMax30100;
    pox.update();
  }
  
  // Print logs and update OLED periodically
  if (millis() - lastLogTime > LOG_INTERVAL) {
    printSensorData();
    updateOLEDDisplay();
    lastLogTime = millis();
  }
  
  // Reset heartbeat flag after a second
  if (millis() - lastHeartbeatDetection > 1000) {
    sensorData.heartbeat = false;
  }
}

void printSensorData() {
  Serial.println("\n--- Sensor Readings ---");
  
  // GPS data
  Serial.println("GPS Data:");
  if (sensorData.gpsConnected) {
    Serial.print("  Latitude: ");
    Serial.print(sensorData.latitude, 6);
    Serial.print(", Longitude: ");
    Serial.println(sensorData.longitude, 6);
    Serial.print("  Altitude: ");
    Serial.print(sensorData.altitude, 2);
    Serial.print(" m, Speed: ");
    Serial.print(sensorData.speed, 2);
    Serial.println(" km/h");
    Serial.print("  Satellites: ");
    Serial.println(sensorData.satellites);
  } else {
    Serial.println("  Disconnected");
  }
  
  // MPU6050 data
  Serial.println("MPU6050 Data:");
  if (sensorData.mpuConnected) {
    Serial.print("  Accel X: ");
    Serial.print(sensorData.accelX, 2);
    Serial.print(" m/s², Y: ");
    Serial.print(sensorData.accelY, 2);
    Serial.print(" m/s², Z: ");
    Serial.print(sensorData.accelZ, 2);
    Serial.println(" m/s²");
    
    Serial.print("  Gyro X: ");
    Serial.print(sensorData.gyroX, 2);
    Serial.print(" rad/s, Y: ");
    Serial.print(sensorData.gyroY, 2);
    Serial.print(" rad/s, Z: ");
    Serial.print(sensorData.gyroZ, 2);
    Serial.println(" rad/s");
    
    Serial.print("  Temperature: ");
    Serial.print(sensorData.temperature, 2);
    Serial.println("°C");
  } else {
    Serial.println("  Disconnected");
  }
  
  // MAX30100 data
  Serial.println("MAX30100 Data:");
  if (sensorData.maxConnected) {
    Serial.print("  Heart Rate: ");
    Serial.print(sensorData.heartRate, 1);
    Serial.print(" BPM, SpO2: ");
    Serial.print(sensorData.spO2, 1);
    Serial.println("%");
  } else {
    Serial.println("  Disconnected");
  }
  
  Serial.println("------------------------");
}

// Centralized function to update all sensor data
void updateAllSensors() {
  // Update every 500ms
  if (millis() - sensorData.lastUpdate > UPDATE_INTERVAL) {
    sensorData.lastUpdate = millis();
    
    // Try to reconnect OLED if it's disconnected
    tryReconnectOLED();
    
    // Update GPS data
    updateGPSData();
    
    // Update MPU6050 data
    updateMPUData();
    
    // Update MAX30100 data
    updateMAXData();
  }
}

// Function to try to reconnect the OLED display
void tryReconnectOLED() {
  static unsigned long lastOLEDReconnect = 0;
  
  if (!sensorData.oledConnected && (millis() - lastOLEDReconnect > 10000)) {
    lastOLEDReconnect = millis();
    Serial.println("Attempting to reconnect OLED display...");
    
    // Asegurarse de usar el bus correcto para el OLED
    Wire = I2CDisplay;
    
    // Try common OLED addresses on Display Bus
    byte oledAddresses[] = {0x3C, 0x3D, 0x78, 0x7A};
    
    for (int i = 0; i < 4; i++) {
      Serial.printf("Trying OLED at address 0x%02X on Display Bus... ", oledAddresses[i]);
      if (display.begin(SSD1306_SWITCHCAPVCC, oledAddresses[i])) {
        Serial.println("SUCCESS!");
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("ESP32 Sensor Hub");
        display.println("Reconnected!");
        display.display();
        sensorData.oledConnected = true;
        Serial.printf("OLED reconnected at 0x%02X\n", oledAddresses[i]);
        break;
      } else {
        Serial.println("Failed");
      }
    }
  }
}

// Function to update GPS data
void updateGPSData() {
  if (sensorData.gpsConnected) {
    int availableBytes = gpsSerial.available();
    if (availableBytes > 0) {
      Serial.print("GPS bytes available: ");
      Serial.println(availableBytes);
    }
    
    while (gpsSerial.available() > 0) {
      char c = gpsSerial.read();
      if (gps.encode(c)) {
        if (gps.location.isUpdated()) {
          Serial.println("GPS location updated");
        }
        
        if (gps.location.isValid()) {
          sensorData.latitude = gps.location.lat();
          sensorData.longitude = gps.location.lng();
        }
        if (gps.altitude.isValid()) {
          sensorData.altitude = gps.altitude.meters();
        }
        if (gps.speed.isValid()) {
          sensorData.speed = gps.speed.kmph();
        }
        if (gps.satellites.isValid()) {
          sensorData.satellites = gps.satellites.value();
        }
      }
    }
  }
}

// Function to update MPU6050 data
void updateMPUData() {
  static unsigned long lastMPUReconnect = 0;
  static unsigned long lastErrorTime = 0;
  static int errorCount = 0;
  static int busResetCount = 0;
  
  // Try to reconnect if disconnected (but not too frequently)
  if (!sensorData.mpuConnected && (millis() - lastMPUReconnect > 5000)) {
    lastMPUReconnect = millis();
    
    // Reset the I2C bus first
    if (busResetCount < 3) { // Only try bus reset a few times
      resetI2CBus(I2C1_SDA, I2C1_SCL);
      busResetCount++;
    }
    
    // Reinitialize the I2C bus
    I2CSensors.begin(I2C1_SDA, I2C1_SCL, I2C_FREQ);
    I2CSensors.setTimeout(1000);
    
    Wire = I2CSensors; // Switch to the sensors bus
    
    Serial.println("Attempting to reconnect MPU6050...");
    
    // Try both possible addresses
    byte mpuAddresses[] = {0x68, 0x69};
    for (int i = 0; i < 2; i++) {
      if (checkI2CDevice(I2CSensors, mpuAddresses[i], "Sensors Bus")) {
        if (mpu.begin(mpuAddresses[i], &I2CSensors, 1)) {
          Serial.printf("MPU6050 reconnected at address 0x%02X!\n", mpuAddresses[i]);
          
          // Reinitialize settings
          mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
          mpu.setGyroRange(MPU6050_RANGE_500_DEG);
          mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
          
          sensorData.mpuConnected = true;
          errorCount = 0;
          busResetCount = 0;  // Reset the bus reset counter on success
          break;
        }
      }
    }
  }
  
  // Read data if connected
  if (sensorData.mpuConnected) {
    // Ensure we're using the correct bus
    Wire = I2CSensors;
    
    sensors_event_t a, g, temp;
    bool success = false;
    
    // Try to get data with error handling
    try {
      success = mpu.getEvent(&a, &g, &temp);
      
      // Additional validation to make sure data seems reasonable
      if (success) {
        success = isMPUDataValid(a.acceleration.x, a.acceleration.y, a.acceleration.z);
        if (!success) {
          Serial.println("MPU6050 data out of expected range, might be invalid");
        }
      }
    } catch (...) {
      Serial.println("Exception during MPU6050 read");
      success = false;
    }
    
    if (success) {
      // Get accelerometer data (in m/s^2)
      sensorData.accelX = a.acceleration.x;
      sensorData.accelY = a.acceleration.y;
      sensorData.accelZ = a.acceleration.z;
      
      // Get gyroscope data (in rad/s)
      sensorData.gyroX = g.gyro.x;
      sensorData.gyroY = g.gyro.y;
      sensorData.gyroZ = g.gyro.z;
      
      // Get temperature (in °C)
      sensorData.temperature = temp.temperature;
      
      // Reset error counter on successful read
      errorCount = 0;
    } else {
      // If we fail to read, count errors
      errorCount++;
      
      // Only print error message once per second to avoid flooding
      if (millis() - lastErrorTime > 1000) {
        lastErrorTime = millis();
        Serial.printf("MPU6050 read error, count: %d\n", errorCount);
      }
      
      // After a few errors, try resetting the I2C bus
      if (errorCount == 5) {
        Serial.println("Attempting I2C bus reset due to errors");
        resetI2CBus(I2C1_SDA, I2C1_SCL);
        I2CSensors.begin(I2C1_SDA, I2C1_SCL, I2C_FREQ);
      }
      
      // If we have many consecutive errors, mark as disconnected
      if (errorCount > 10) {
        Serial.println("Too many MPU6050 errors, marking as disconnected");
        sensorData.mpuConnected = false;
        errorCount = 0;
      }
    }
  }
}

// Function to update MAX30100 data
void updateMAXData() {
  // Try to reconnect if disconnected
  if (!sensorData.maxConnected) {
    Wire = I2CMax30100; // Switch to the MAX30100 bus
    if (pox.begin()) {
      Serial.println("MAX30100 connected!");
      pox.setOnBeatDetectedCallback(onBeatDetected);
      sensorData.maxConnected = true;
    }
  }
  
  if (sensorData.maxConnected) {
    // Ensure we're using the correct bus
    Wire = I2CMax30100;
    
    // Call update (already done in loop but we also do it here for coherence)
    pox.update();
    
    // Check for valid readings
    float newHeartRate = pox.getHeartRate();
    float newSpO2 = pox.getSpO2();
    
    // Only update if we have valid values (non-zero)
    if (newHeartRate > 0) {
      sensorData.heartRate = newHeartRate;
    }
    if (newSpO2 > 0) {
      sensorData.spO2 = newSpO2;
    }
    
    if (sensorData.heartRate > 0 || sensorData.spO2 > 0) {
      Serial.print("MAX30100 Update - Heart rate: ");
      Serial.print(sensorData.heartRate);
      Serial.print(" BPM, SpO2: ");
      Serial.print(sensorData.spO2);
      Serial.println("%");
    }
  }
}

String getJsonData() {
  String json = "{";
  
  // GPS data
  json += "\"gps\":{";
  json += "\"connected\":" + String(sensorData.gpsConnected ? "true" : "false") + ",";
  json += "\"lat\":" + String(sensorData.latitude, 6) + ",";
  json += "\"lng\":" + String(sensorData.longitude, 6) + ",";
  json += "\"alt\":" + String(sensorData.altitude, 2) + ",";
  json += "\"speed\":" + String(sensorData.speed, 2) + ",";
  json += "\"satellites\":" + String(sensorData.satellites);
  json += "},";
  
  // MPU6050 data
  json += "\"mpu\":{";
  json += "\"connected\":" + String(sensorData.mpuConnected ? "true" : "false") + ",";
  json += "\"accelX\":" + String(sensorData.accelX, 2) + ",";
  json += "\"accelY\":" + String(sensorData.accelY, 2) + ",";
  json += "\"accelZ\":" + String(sensorData.accelZ, 2) + ",";
  json += "\"gyroX\":" + String(sensorData.gyroX, 2) + ",";
  json += "\"gyroY\":" + String(sensorData.gyroY, 2) + ",";
  json += "\"gyroZ\":" + String(sensorData.gyroZ, 2) + ",";
  json += "\"temp\":" + String(sensorData.temperature, 2);
  json += "},";
  
  // MAX30100 data
  json += "\"heart\":{";
  json += "\"connected\":" + String(sensorData.maxConnected ? "true" : "false") + ",";
  json += "\"rate\":" + String(sensorData.heartRate, 1) + ",";
  json += "\"spo2\":" + String(sensorData.spO2, 1) + ",";
  json += "\"beat\":" + String(sensorData.heartbeat ? 1 : 0);
  json += "}";
  
  json += "}";
  
  // Debug JSON data to serial
  Serial.println("JSON Data for Web Client:");
  Serial.println(json);
  
  return json;
}

void handleWebRequest() {
  Serial.print("Web request received from client: ");
  Serial.println(server.client().remoteIP());
  Serial.print("Request URI: ");
  Serial.println(server.uri());
}

void handleRoot() {
  handleWebRequest();
  Serial.println("Serving root page...");
  String html = "<!DOCTYPE html>"
                "<html lang='en'>"
                "<head>"
                "  <meta charset='UTF-8'>"
                "  <meta name='viewport' content='width=device-width, initial-scale=1.0'>"
                "  <title>ESP32 Sensor Dashboard</title>"
                "  <style>"
                "    body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background-color: #f4f4f4; }"
                "    h1 { color: #333; text-align: center; }"
                "    .dashboard { display: flex; flex-wrap: wrap; justify-content: space-around; }"
                "    .card { background: white; border-radius: 8px; padding: 15px; margin: 10px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); width: 300px; }"
                "    .card h2 { margin-top: 0; color: #2c3e50; border-bottom: 1px solid #eee; padding-bottom: 10px; }"
                "    .data-row { display: flex; justify-content: space-between; margin: 8px 0; }"
                "    .label { font-weight: bold; color: #7f8c8d; }"
                "    .value { color: #2c3e50; }"
                "    .disconnected { color: #e74c3c; }"
                "    .connected { color: #27ae60; }"
                "    .heartbeat { color: red; animation: pulse 1s infinite; display: none; }"
                "    @keyframes pulse { 0% { opacity: 1; } 50% { opacity: 0.3; } 100% { opacity: 1; } }"
                "    #status { position: fixed; top: 10px; right: 10px; padding: 10px; background: #333; color: white; border-radius: 5px; }"
                "    #refresh-button {"
                "      position: fixed;"
                "      top: 10px;"
                "      left: 10px;"
                "      padding: 10px 20px;"
                "      background-color: #4CAF50;"
                "      color: white;"
                "      border: none;"
                "      border-radius: 5px;"
                "      cursor: pointer;"
                "      font-size: 16px;"
                "    }"
                "    #refresh-button:hover {"
                "      background-color: #45a049;"
                "    }"
                "    #refresh-button:active {"
                "      background-color: #3e8e41;"
                "    }"
                "  </style>"
                "</head>"
                "<body>"
                "  <button id='refresh-button'>Refresh Data</button>"
                "  <div id='status'>Page Loaded</div>"
                "  <h1>ESP32 Sensor Dashboard</h1>"
                "  <div class='dashboard'>"
                "    <div class='card'>"
                "      <h2>GPS Data <span id='gps-status' class='disconnected'>(Disconnected)</span></h2>"
                "      <div class='data-row'><span class='label'>Latitude:</span> <span id='gps-lat' class='value'>--.------</span></div>"
                "      <div class='data-row'><span class='label'>Longitude:</span> <span id='gps-lng' class='value'>--.------</span></div>"
                "      <div class='data-row'><span class='label'>Altitude:</span> <span id='gps-alt' class='value'>--.-</span> m</div>"
                "      <div class='data-row'><span class='label'>Speed:</span> <span id='gps-speed' class='value'>--.-</span> km/h</div>"
                "      <div class='data-row'><span class='label'>Satellites:</span> <span id='gps-sat' class='value'>--</span></div>"
                "    </div>"
                "    <div class='card'>"
                "      <h2>MPU6050 Data <span id='mpu-status' class='disconnected'>(Disconnected)</span></h2>"
                "      <div class='data-row'><span class='label'>Accel X:</span> <span id='mpu-ax' class='value'>--.-</span> m/s²</div>"
                "      <div class='data-row'><span class='label'>Accel Y:</span> <span id='mpu-ay' class='value'>--.-</span> m/s²</div>"
                "      <div class='data-row'><span class='label'>Accel Z:</span> <span id='mpu-az' class='value'>--.-</span> m/s²</div>"
                "      <div class='data-row'><span class='label'>Gyro X:</span> <span id='mpu-gx' class='value'>--.-</span> rad/s</div>"
                "      <div class='data-row'><span class='label'>Gyro Y:</span> <span id='mpu-gy' class='value'>--.-</span> rad/s</div>"
                "      <div class='data-row'><span class='label'>Gyro Z:</span> <span id='mpu-gz' class='value'>--.-</span> rad/s</div>"
                "      <div class='data-row'><span class='label'>Temperature:</span> <span id='mpu-temp' class='value'>--.-</span> °C</div>"
                "    </div>"
                "    <div class='card'>"
                "      <h2>Heart Rate Data <span id='heart-status' class='disconnected'>(Disconnected)</span></h2>"
                "      <div class='data-row'>"
                "        <span class='label'>Heart Rate:</span> "
                "        <span id='heart-rate' class='value'>--.-</span> BPM "
                "        <span id='heartbeat' class='heartbeat'>♥</span>"
                "      </div>"
                "      <div class='data-row'><span class='label'>SpO2:</span> <span id='heart-spo2' class='value'>--.-</span> %</div>"
                "    </div>"
                "  </div>"
                "<script>\n"
                "document.getElementById('status').textContent='Script Running';\n"
                "document.getElementById('refresh-button').addEventListener('click',function(){updateSensorData();});\n"
                "function updateSensorData(){\n"
                "document.getElementById('status').textContent='Updating...';\n"
                "fetch('/update').then(response=>{\n"
                "if(!response.ok){throw new Error('Network response was not ok');}\n"
                "return response.json();\n"
                "}).then(data=>{\n"
                "document.getElementById('status').textContent='Last Update: '+new Date().toLocaleTimeString();\n"
                "document.getElementById('gps-status').textContent=data.gps.connected?'(Connected)':'(Disconnected)';\n"
                "document.getElementById('gps-status').className=data.gps.connected?'connected':'disconnected';\n"
                "document.getElementById('mpu-status').textContent=data.mpu.connected?'(Connected)':'(Disconnected)';\n"
                "document.getElementById('mpu-status').className=data.mpu.connected?'connected':'disconnected';\n"
                "document.getElementById('heart-status').textContent=data.heart.connected?'(Connected)':'(Disconnected)';\n"
                "document.getElementById('heart-status').className=data.heart.connected?'connected':'disconnected';\n"
                "document.getElementById('gps-lat').textContent=data.gps.lat.toFixed(6);\n"
                "document.getElementById('gps-lng').textContent=data.gps.lng.toFixed(6);\n"
                "document.getElementById('gps-alt').textContent=data.gps.alt.toFixed(2);\n"
                "document.getElementById('gps-speed').textContent=data.gps.speed.toFixed(2);\n"
                "document.getElementById('gps-sat').textContent=data.gps.satellites;\n"
                "document.getElementById('mpu-ax').textContent=data.mpu.accelX.toFixed(2);\n"
                "document.getElementById('mpu-ay').textContent=data.mpu.accelY.toFixed(2);\n"
                "document.getElementById('mpu-az').textContent=data.mpu.accelZ.toFixed(2);\n"
                "document.getElementById('mpu-gx').textContent=data.mpu.gyroX.toFixed(2);\n"
                "document.getElementById('mpu-gy').textContent=data.mpu.gyroY.toFixed(2);\n"
                "document.getElementById('mpu-gz').textContent=data.mpu.gyroZ.toFixed(2);\n"
                "document.getElementById('mpu-temp').textContent=data.mpu.temp.toFixed(2);\n"
                "document.getElementById('heart-rate').textContent=data.heart.rate.toFixed(1);\n"
                "document.getElementById('heart-spo2').textContent=data.heart.spo2.toFixed(1);\n"
                "if(data.heart.beat===1){\n"
                "document.getElementById('heartbeat').style.display='inline';\n"
                "}else{\n"
                "document.getElementById('heartbeat').style.display='none';\n"
                "}\n"
                "}).catch(error=>{\n"
                "setTimeout(updateSensorData,1000);\n"
                "});\n"
                "}\n"
                "setInterval(updateSensorData,1000);\n"
                "updateSensorData();\n"
                "</script>\n"
                "</body>"
                "</html>";
  server.send(200, "text/html", html);
  Serial.println("Home page served");
}

void handleData() {
  handleWebRequest();
  Serial.println("Serving JSON data...");
  String jsonData = getJsonData();
  Serial.println("JSON data length: " + String(jsonData.length()));
  server.send(200, "application/json", jsonData);
  Serial.println("JSON data served");
}

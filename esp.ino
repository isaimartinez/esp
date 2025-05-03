#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
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

// OLED Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1  // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x7A  // Common OLED display address
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// GPS module connected to Serial2
#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600

// Create an instance of HardwareSerial for GPS
HardwareSerial gpsSerial(2);

// Variables to store sensor data
float latitude = 0, longitude = 0, altitude = 0, speed = 0;
int satellites = 0;
float accelX = 0, accelY = 0, accelZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float temperature = 0;
float heartRate = 0;
float spO2 = 0;

// Sensor status flags
bool mpuConnected = false;
bool maxConnected = false;
bool gpsConnected = false;
bool oledConnected = false;

// Timer variables
unsigned long lastDataUpdate = 0;
unsigned long lastHeartbeatDetection = 0;
unsigned long lastLogTime = 0;
const unsigned long LOG_INTERVAL = 1000; // Log every second

// Callback for pulse detection
void onBeatDetected() {
  lastHeartbeatDetection = millis();
  Serial.println("♥ Heartbeat detected!");
}

// Function to check if an I2C device is connected at a specific address
bool checkI2CDevice(byte address) {
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();
  Serial.print("I2C Device 0x");
  Serial.print(address, HEX);
  Serial.print(": ");
  Serial.println(error == 0 ? "Connected" : "Not found");
  return (error == 0); // Return true if device exists (error = 0)
}

// Function to scan all I2C addresses
void scanI2CDevices() {
  byte error, address;
  int nDevices = 0;
  
  Serial.println("Scanning for I2C devices...");
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
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
    Serial.println("No I2C devices found\n");
  } else {
    Serial.print("Found ");
    Serial.print(nDevices);
    Serial.println(" device(s)\n");
  }
}

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  Serial.println("\n\n");
  Serial.println("==============================================");
  Serial.println("ESP32 Sensor Web Server - Starting up...");
  Serial.println("==============================================");

  // Initialize I2C communication with specific pins and frequency
  Serial.println("Initializing I2C bus...");
  
  // ESP32 default I2C pins are GPIO21 (SDA) and GPIO22 (SCL)
  // But we can explicitly set them - try GPIO16 (SDA) and GPIO17 (SCL) as alternatives
  #define I2C_SDA 21
  #define I2C_SCL 22
  #define I2C_FREQ 100000  // Try a lower frequency (100kHz) for better reliability
  
  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
  Serial.printf("I2C bus initialized on pins SDA=%d, SCL=%d at %d Hz\n", I2C_SDA, I2C_SCL, I2C_FREQ);
  
  // Add a delay to stabilize I2C
  delay(100);
  
  // Scan for I2C devices to find all available addresses
  scanI2CDevices();
  
  // Initialize OLED display first
  Serial.println("\n--- OLED Display Initialization ---");
  // Try common OLED addresses
  byte oledAddresses[] = {0x3C, 0x3D, 0x78, 0x7A};
  
  oledConnected = false;
  for (int i = 0; i < 4 && !oledConnected; i++) {
    Serial.printf("Trying OLED at address 0x%02X... ", oledAddresses[i]);
    if (display.begin(SSD1306_SWITCHCAPVCC, oledAddresses[i])) {
      Serial.println("SUCCESS!");
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("ESP32 Sensor Hub");
      display.println("Initializing...");
      display.display();
      oledConnected = true;
      Serial.printf("OLED connected at 0x%02X\n", oledAddresses[i]);
    } else {
      Serial.println("Failed");
    }
  }
  
  if (!oledConnected) {
    Serial.println("Could not find an OLED display");
  }
  
  // Initialize MPU6050 if connected (address 0x68)
  Serial.println("\n--- MPU6050 Initialization ---");
  Serial.println("Checking for MPU6050...");
  if (checkI2CDevice(0x68)) {
    Serial.println("MPU6050 found, initializing...");
    if (mpu.begin()) {
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
      
      mpuConnected = true;
    } else {
      Serial.println("MPU6050 initialization failed");
    }
  } else {
    Serial.println("MPU6050 not found");
  }
  
  // Initialize MAX30100
  Serial.println("\n--- MAX30100 Initialization ---");
  Serial.print("Initializing pulse oximeter..");
  
  // Initialize the PulseOximeter instance and handle failures
  if (!pox.begin()) {
    Serial.println("FAILED");
    Serial.println("MAX30100 initialization failed - check wiring or I2C address");
    maxConnected = false;
  } else {
    Serial.println("SUCCESS");
    Serial.println("MAX30100 initialized successfully");
    // Register callback for the beat detection
    pox.setOnBeatDetectedCallback(onBeatDetected);
    Serial.println("Heartbeat detection callback set");
    maxConnected = true;
  }
  
  // Initialize GPS
  Serial.println("\n--- GPS Initialization ---");
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.print("GPS initialized on pins RX:");
  Serial.print(GPS_RX);
  Serial.print(", TX:");
  Serial.println(GPS_TX);
  gpsConnected = true;
  
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
  Serial.println(gpsConnected ? "Connected" : "Not connected");
  Serial.print("- MPU6050: ");
  Serial.println(mpuConnected ? "Connected" : "Not connected");
  Serial.print("- MAX30100: ");
  Serial.println(maxConnected ? "Connected" : "Not connected");
  Serial.print("- OLED: ");
  Serial.println(oledConnected ? "Connected" : "Not connected");
  Serial.println("==============================================\n");
}

// Function to update OLED display
void updateOLEDDisplay() {
  if (!oledConnected) return;
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Title
  display.println("ESP32 Sensor Hub");
  display.println("---------------");
  
  // GPS Status
  display.print("GPS: ");
  if (gpsConnected) {
    display.print("OK ");
    display.print(satellites);
    display.println(" sats");
    display.print("Lat:");
    display.println(latitude, 4);
    display.print("Lng:");
    display.println(longitude, 4);
  } else {
    display.println("Disconnected");
  }
  
  // MPU6050 Status
  display.print("MPU: ");
  if (mpuConnected) {
    display.println("OK");
    display.print("Acc:");
    display.print(accelX, 1);
    display.print(",");
    display.print(accelY, 1);
    display.print(",");
    display.println(accelZ, 1);
  } else {
    display.println("Disconnected");
  }
  
  // MAX30100 Status
  display.print("HR: ");
  if (maxConnected) {
    display.print(heartRate, 0);
    display.print("BPM SpO2:");
    display.print(spO2, 0);
    display.println("%");
  } else {
    display.println("Disconnected");
  }
  
  display.display();
}

void loop() {
  server.handleClient();
  updateSensorData();
  
  // Only update MAX30100 if connected
  if (maxConnected) {
    pox.update();
  }
  
  // Print logs and update OLED periodically
  if (millis() - lastLogTime > LOG_INTERVAL) {
    printSensorData();
    updateOLEDDisplay();
    lastLogTime = millis();
  }
}

void printSensorData() {
  Serial.println("\n--- Sensor Readings ---");
  
  // GPS data
  Serial.println("GPS Data:");
  if (gpsConnected) {
    Serial.print("  Latitude: ");
    Serial.print(latitude, 6);
    Serial.print(", Longitude: ");
    Serial.println(longitude, 6);
    Serial.print("  Altitude: ");
    Serial.print(altitude, 2);
    Serial.print(" m, Speed: ");
    Serial.print(speed, 2);
    Serial.println(" km/h");
    Serial.print("  Satellites: ");
    Serial.println(satellites);
  } else {
    Serial.println("  Disconnected");
  }
  
  // MPU6050 data
  Serial.println("MPU6050 Data:");
  if (mpuConnected) {
    Serial.print("  Accel X: ");
    Serial.print(accelX, 2);
    Serial.print(" m/s², Y: ");
    Serial.print(accelY, 2);
    Serial.print(" m/s², Z: ");
    Serial.print(accelZ, 2);
    Serial.println(" m/s²");
    
    Serial.print("  Gyro X: ");
    Serial.print(gyroX, 2);
    Serial.print(" rad/s, Y: ");
    Serial.print(gyroY, 2);
    Serial.print(" rad/s, Z: ");
    Serial.print(gyroZ, 2);
    Serial.println(" rad/s");
    
    Serial.print("  Temperature: ");
    Serial.print(temperature, 2);
    Serial.println("°C");
  } else {
    Serial.println("  Disconnected");
  }
  
  // MAX30100 data
  Serial.println("MAX30100 Data:");
  if (maxConnected) {
    Serial.print("  Heart Rate: ");
    Serial.print(heartRate, 1);
    Serial.print(" BPM, SpO2: ");
    Serial.print(spO2, 1);
    Serial.println("%");
  } else {
    Serial.println("  Disconnected");
  }
  
  Serial.println("------------------------");
}

void updateSensorData() {
  // Update every 500ms
  if (millis() - lastDataUpdate > 500) {
    lastDataUpdate = millis();
    
    // Try to reconnect OLED if it's disconnected (only try once every 10 seconds)
    static unsigned long lastOLEDReconnect = 0;
    if (!oledConnected && (millis() - lastOLEDReconnect > 10000)) {
      lastOLEDReconnect = millis();
      Serial.println("Attempting to reconnect OLED display...");
      
      // Try common OLED addresses
      byte oledAddresses[] = {0x3C, 0x3D, 0x78, 0x7A};
      
      for (int i = 0; i < 4; i++) {
        Serial.printf("Trying OLED at address 0x%02X... ", oledAddresses[i]);
        if (display.begin(SSD1306_SWITCHCAPVCC, oledAddresses[i])) {
          Serial.println("SUCCESS!");
          display.clearDisplay();
          display.setTextSize(1);
          display.setTextColor(SSD1306_WHITE);
          display.setCursor(0, 0);
          display.println("ESP32 Sensor Hub");
          display.println("Reconnected!");
          display.display();
          oledConnected = true;
          Serial.printf("OLED reconnected at 0x%02X\n", oledAddresses[i]);
          break;
        } else {
          Serial.println("Failed");
        }
      }
    }
    
    // Read GPS data if connected
    if (gpsConnected) {
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
            latitude = gps.location.lat();
            longitude = gps.location.lng();
          }
          if (gps.altitude.isValid()) {
            altitude = gps.altitude.meters();
          }
          if (gps.speed.isValid()) {
            speed = gps.speed.kmph();
          }
          if (gps.satellites.isValid()) {
            satellites = gps.satellites.value();
          }
        }
      }
    }
    
    // Try to read MPU6050 data even if it was initially disconnected
    // This allows recovery if the sensor was connected after boot
    
    // First, check if we need to reconnect
    if (!mpuConnected) {
      // Try to initialize the sensor if it was previously disconnected
      if (mpu.begin()) {
        Serial.println("MPU6050 connected!");
        mpuConnected = true;
      }
    }
    
    // Read data if connected
    if (mpuConnected) {
      sensors_event_t a, g, temp;
      if (mpu.getEvent(&a, &g, &temp)) {
        // Get accelerometer data (in m/s^2)
        accelX = a.acceleration.x;
        accelY = a.acceleration.y;
        accelZ = a.acceleration.z;
        
        // Get gyroscope data (in rad/s)
        gyroX = g.gyro.x;
        gyroY = g.gyro.y;
        gyroZ = g.gyro.z;
        
        // Get temperature (in °C)
        temperature = temp.temperature;
      } else {
        // If we fail to read, mark as disconnected
        mpuConnected = false;
      }
    }
    
    // Try to read MAX30100 data even if it was initially disconnected
    // This allows recovery if the sensor was connected after boot
    
    // First, check if we need to reconnect
    if (!maxConnected) {
      // Try to initialize the sensor if it was previously disconnected
      if (pox.begin()) {
        Serial.println("MAX30100 connected!");
        pox.setOnBeatDetectedCallback(onBeatDetected);
        maxConnected = true;
      }
    }
    
    if (maxConnected) {
      // Make sure to call update as fast as possible
      pox.update(); // This is void, doesn't return a value
      
      // Check for valid readings
      float newHeartRate = pox.getHeartRate();
      float newSpO2 = pox.getSpO2();
      
      // Only update if we have valid values (non-zero)
      if (newHeartRate > 0) {
        heartRate = newHeartRate;
      }
      if (newSpO2 > 0) {
        spO2 = newSpO2;
      }
      
      if (heartRate > 0 || spO2 > 0) {
        Serial.print("MAX30100 Update - Heart rate: ");
        Serial.print(heartRate);
        Serial.print(" BPM, SpO2: ");
        Serial.print(spO2);
        Serial.println("%");
      }
    }
  }
}

String getJsonData() {
  String json = "{";
  
  // GPS data
  json += "\"gps\":{";
  json += "\"connected\":" + String(gpsConnected ? "true" : "false") + ",";
  json += "\"lat\":" + String(latitude, 6) + ",";
  json += "\"lng\":" + String(longitude, 6) + ",";
  json += "\"alt\":" + String(altitude, 2) + ",";
  json += "\"speed\":" + String(speed, 2) + ",";
  json += "\"satellites\":" + String(satellites);
  json += "},";
  
  // MPU6050 data
  json += "\"mpu\":{";
  json += "\"connected\":" + String(mpuConnected ? "true" : "false") + ",";
  json += "\"accelX\":" + String(accelX, 2) + ",";
  json += "\"accelY\":" + String(accelY, 2) + ",";
  json += "\"accelZ\":" + String(accelZ, 2) + ",";
  json += "\"gyroX\":" + String(gyroX, 2) + ",";
  json += "\"gyroY\":" + String(gyroY, 2) + ",";
  json += "\"gyroZ\":" + String(gyroZ, 2) + ",";
  json += "\"temp\":" + String(temperature, 2);
  json += "},";
  
  // MAX30100 data
  json += "\"heart\":{";
  json += "\"connected\":" + String(maxConnected ? "true" : "false") + ",";
  json += "\"rate\":" + String(heartRate, 1) + ",";
  json += "\"spo2\":" + String(spO2, 1) + ",";
  json += "\"beat\":" + String((millis() - lastHeartbeatDetection < 1000) ? 1 : 0);
  json += "}";
  
  json += "}";
  return json;
}

void handleWebRequest() {
  Serial.print("Web request received: ");
  Serial.println(server.uri());
}

void handleRoot() {
  handleWebRequest();
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
                "  </style>"
                "</head>"
                "<body>"
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
                "  <script>"
                "    function updateSensorData() {"
                "      fetch('/update')"
                "        .then(response => response.json())"
                "        .then(data => {"
                "          // Update connection status"
                "          document.getElementById('gps-status').textContent = data.gps.connected ? '(Connected)' : '(Disconnected)';"
                "          document.getElementById('gps-status').className = data.gps.connected ? 'connected' : 'disconnected';"
                "          document.getElementById('mpu-status').textContent = data.mpu.connected ? '(Connected)' : '(Disconnected)';"
                "          document.getElementById('mpu-status').className = data.mpu.connected ? 'connected' : 'disconnected';"
                "          document.getElementById('heart-status').textContent = data.heart.connected ? '(Connected)' : '(Disconnected)';"
                "          document.getElementById('heart-status').className = data.heart.connected ? 'connected' : 'disconnected';"
                "          "
                "          // Update GPS data"
                "          document.getElementById('gps-lat').textContent = data.gps.lat.toFixed(6);"
                "          document.getElementById('gps-lng').textContent = data.gps.lng.toFixed(6);"
                "          document.getElementById('gps-alt').textContent = data.gps.alt.toFixed(2);"
                "          document.getElementById('gps-speed').textContent = data.gps.speed.toFixed(2);"
                "          document.getElementById('gps-sat').textContent = data.gps.satellites;"
                "          "
                "          // Update MPU6050 data"
                "          document.getElementById('mpu-ax').textContent = data.mpu.accelX.toFixed(2);"
                "          document.getElementById('mpu-ay').textContent = data.mpu.accelY.toFixed(2);"
                "          document.getElementById('mpu-az').textContent = data.mpu.accelZ.toFixed(2);"
                "          document.getElementById('mpu-gx').textContent = data.mpu.gyroX.toFixed(2);"
                "          document.getElementById('mpu-gy').textContent = data.mpu.gyroY.toFixed(2);"
                "          document.getElementById('mpu-gz').textContent = data.mpu.gyroZ.toFixed(2);"
                "          document.getElementById('mpu-temp').textContent = data.mpu.temp.toFixed(2);"
                "          "
                "          // Update MAX30100 data"
                "          document.getElementById('heart-rate').textContent = data.heart.rate.toFixed(1);"
                "          document.getElementById('heart-spo2').textContent = data.heart.spo2.toFixed(1);"
                "          "
                "          // Show heartbeat animation"
                "          if (data.heart.beat === 1) {"
                "            document.getElementById('heartbeat').style.display = 'inline';"
                "          } else {"
                "            document.getElementById('heartbeat').style.display = 'none';"
                "          }"
                "        })"
                "        .catch(error => console.error('Error fetching data:', error));"
                "    }"
                "    "
                "    // Update data every second"
                "    setInterval(updateSensorData, 1000);"
                "    "
                "    // Initial update"
                "    updateSensorData();"
                "  </script>"
                "</body>"
                "</html>";
  server.send(200, "text/html", html);
  Serial.println("Home page served");
}

void handleData() {
  handleWebRequest();
  server.send(200, "application/json", getJsonData());
  Serial.println("JSON data served");
}

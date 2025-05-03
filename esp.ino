#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <MPU6050.h>
#include <MAX30100_PulseOximeter.h>

// WiFi credentials
const char* ssid = "ESP32_AP";
const char* password = "12345678";

// Create objects
WebServer server(80);
TinyGPSPlus gps;
MPU6050 mpu;
PulseOximeter pox;

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

// Timer variables
unsigned long lastDataUpdate = 0;
unsigned long lastHeartbeatDetection = 0;

// Callback for pulse detection
void onBeatDetected() {
  lastHeartbeatDetection = millis();
}

// Function to check if an I2C device is connected at a specific address
bool checkI2CDevice(byte address) {
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();
  return (error == 0); // Return true if device exists (error = 0)
}

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  Serial.println("ESP32 Sensor Web Server");

  // Initialize I2C communication
  Wire.begin();
  
  // Initialize MPU6050 if connected (address 0x68)
  Serial.println("Checking for MPU6050...");
  if (checkI2CDevice(0x68)) {
    Serial.println("MPU6050 found, initializing...");
    mpu.initialize();
    if (mpu.testConnection()) {
      Serial.println("MPU6050 initialized successfully");
      mpuConnected = true;
    } else {
      Serial.println("MPU6050 initialization failed");
    }
  } else {
    Serial.println("MPU6050 not found");
  }
  
  // Initialize MAX30100 if connected (address 0x57)
  Serial.println("Checking for MAX30100...");
  if (checkI2CDevice(0x57)) {
    Serial.println("MAX30100 found, initializing...");
    if (pox.begin()) {
      Serial.println("MAX30100 initialized successfully");
      pox.setOnBeatDetectedCallback(onBeatDetected);
      maxConnected = true;
    } else {
      Serial.println("MAX30100 initialization failed");
    }
  } else {
    Serial.println("MAX30100 not found");
  }
  
  // Initialize GPS
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS initialized");
  gpsConnected = true;
  
  // Set up WiFi Access Point
  Serial.println("Setting up Access Point...");
  WiFi.softAP(ssid, password);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  // Set up web server routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/update", HTTP_GET, []() {
    server.send(200, "application/json", getJsonData());
  });
  
  // Start server
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
  updateSensorData();
  
  // Only update MAX30100 if connected
  if (maxConnected) {
    pox.update();
  }
}

void updateSensorData() {
  // Update every 500ms
  if (millis() - lastDataUpdate > 500) {
    lastDataUpdate = millis();
    
    // Read GPS data if connected
    if (gpsConnected) {
      while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
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
    
    // Read MPU6050 data if connected
    if (mpuConnected) {
      int16_t ax, ay, az, gx, gy, gz;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      
      // Convert to meaningful units
      // Accelerometer LSB sensitivity is 16384 LSB/g for ±2g range (default)
      accelX = ax / 16384.0; // in g
      accelY = ay / 16384.0;
      accelZ = az / 16384.0;
      
      // Gyro LSB sensitivity is 131 LSB/(°/s) for ±250°/s range (default)
      gyroX = gx / 131.0; // in °/s
      gyroY = gy / 131.0;
      gyroZ = gz / 131.0;
      
      // Read temperature from MPU6050
      temperature = mpu.getTemperature() / 340.0 + 36.53; // Formula from datasheet
    }
    
    // Read MAX30100 data if connected
    if (maxConnected) {
      heartRate = pox.getHeartRate();
      spO2 = pox.getSpO2();
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

void handleRoot() {
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
                "      <div class='data-row'><span class='label'>Accel X:</span> <span id='mpu-ax' class='value'>--.-</span> g</div>"
                "      <div class='data-row'><span class='label'>Accel Y:</span> <span id='mpu-ay' class='value'>--.-</span> g</div>"
                "      <div class='data-row'><span class='label'>Accel Z:</span> <span id='mpu-az' class='value'>--.-</span> g</div>"
                "      <div class='data-row'><span class='label'>Gyro X:</span> <span id='mpu-gx' class='value'>--.-</span> °/s</div>"
                "      <div class='data-row'><span class='label'>Gyro Y:</span> <span id='mpu-gy' class='value'>--.-</span> °/s</div>"
                "      <div class='data-row'><span class='label'>Gyro Z:</span> <span id='mpu-gz' class='value'>--.-</span> °/s</div>"
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
}

void handleData() {
  server.send(200, "application/json", getJsonData());
}

#include <Wire.h>

void setup() {
  Wire.begin(); // Initialize I2C bus
  
  Serial.begin(115200);
  while (!Serial);
  
  delay(1000); // Wait for serial monitor to open
  
  Serial.println("I2C Scanner");
}

void loop() {
  byte error, address;
  int nDevices = 0;
  
  Serial.println("Scanning for I2C devices...");
  
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      
      // Print device description for known addresses
      if (address == 0x68 || address == 0x69) {
        Serial.print(" (MPU6050)");
      } else if (address == 0x57) {
        Serial.print(" (MAX30100)");
      }
      
      Serial.println();
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  
  if (nDevices == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.print("Found ");
    Serial.print(nDevices);
    Serial.println(" device(s)");
  }
  
  Serial.println();
  delay(5000); // Wait 5 seconds before scanning again
} 
#include <Arduino.h>
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  
  // Set WiFi mode to station
  WiFi.mode(WIFI_MODE_STA);
  
  // Get MAC address as string
  String macStr = WiFi.macAddress();
  
  // Print standard format
  Serial.print("ESP32 MAC Address: ");
  Serial.println(macStr);
  Serial.println();
  
  // Parse and print in C array format
  Serial.println("Copy this into your code:");
  Serial.print("uint8_t broadcastAddress[] = {");
  
  // Parse each byte from the MAC address string
  for (int i = 0; i < 6; i++) {
    // Extract each hex pair (e.g., "08", "D1", etc.)
    String hexByte = macStr.substring(i * 3, i * 3 + 2);
    
    // Convert to integer and print with 0x prefix
    Serial.print("0x");
    Serial.print(hexByte);
    
    // Add comma except for last element
    if (i < 5) {
      Serial.print(",");
    }
  }
  
  Serial.println("};");
}

void loop() {
  // Empty loop
}

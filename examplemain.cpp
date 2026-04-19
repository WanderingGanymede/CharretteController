#include <Arduino.h>

#define ESP_SERIAL Serial1
#define ESP_BAUD 115200

const char* ap_ssid = "PicoTrailer";
const char* ap_password = "12345678";
const int tcp_port = 8888;

// Helper: send AT command and wait for response (basic)
bool sendAT(const char* cmd, unsigned long timeout = 2000) {
  ESP_SERIAL.println(cmd);
  unsigned long start = millis();
  while (millis() - start < timeout) {
    if (ESP_SERIAL.available()) {
      String resp = ESP_SERIAL.readString();
      if (resp.indexOf("OK") != -1 || resp.indexOf(">") != -1 || resp.indexOf("CONNECT") != -1) {
        return true;
      }
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== ESP8285 TCP Server (No Echo) ===");

  ESP_SERIAL.begin(ESP_BAUD);
  delay(1000);

  // Step 1: Disable command echo
  ESP_SERIAL.println("ATE0");
  delay(500);
  Serial.println("Echo disabled (ATE0)");

  // Step 2: Test AT
  if (!sendAT("AT")) {
    Serial.println("No response from ESP");
    while (1);
  }
  Serial.println("ESP OK");

  // Step 3: Set SoftAP mode
  sendAT("AT+CWMODE=2");
  delay(500);

  // Step 4: Configure AP
  String apCmd = "AT+CWSAP=\"" + String(ap_ssid) + "\",\"" + String(ap_password) + "\",6,3";
  sendAT(apCmd.c_str());
  delay(500);

  // Step 5: Enable multiple connections
  sendAT("AT+CIPMUX=1");
  delay(500);

  // Step 6: Start TCP server
  String serverCmd = "AT+CIPSERVER=1," + String(tcp_port);
  sendAT(serverCmd.c_str());
  delay(500);

  Serial.println("TCP Server running on port " + String(tcp_port));
  Serial.println("Connect laptop to 'PicoTrailer' and run Python client");
}

void loop() {
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 5000) {
    // The message you want the client to see
    String message = "Hello from Pico! Time: " + String(millis()) + " ms\r\n";

    // Send command to send data over connection 0
    ESP_SERIAL.print("AT+CIPSEND=0,");
    ESP_SERIAL.println(message.length());

    // Wait for '>' prompt
    unsigned long start = millis();
    bool gotPrompt = false;
    while (millis() - start < 1000) {
      while (ESP_SERIAL.available()) {
        if (ESP_SERIAL.read() == '>') {
          gotPrompt = true;
          break;
        }
      }
      if (gotPrompt) break;
    }

    if (gotPrompt) {
      // Send the actual data (no extra newlines)
      ESP_SERIAL.print(message);
      Serial.println("Sent: " + message);
    } else {
      Serial.println("No client connected or CIPSEND failed");
    }
    lastSend = millis();
  }

  // Optional: forward any ESP debug output to Serial
  while (ESP_SERIAL.available()) {
    char c = ESP_SERIAL.read();
    Serial.write(c);
  }
}

#include <esp_now.h>
#include <WiFi.h>

uint8_t carMAC[] = {0x70, 0x021, 0x84, 0x7C, 0xC1, 0x56};  // Replace with your car ESP32 MAC address

typedef struct struct_message {
  char command[32];
} struct_message;

struct_message myData;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, carMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  Serial.println("ESP-NOW initialized, peer added");
}

void loop() {
  strcpy(myData.command, "FWD");  // Example command
  esp_err_t result = esp_now_send(carMAC, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }

  delay(1000);  // Adjust the delay as necessary
}

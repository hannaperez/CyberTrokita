#include <esp_now.h>
#include <WiFi.h>

const int motorPin1 = 12;  // Adjust according to your setup
const int motorPin2 = 14;

typedef struct struct_message {
  char command[32];
} struct_message;

struct_message myData;

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Received: ");
  Serial.println(myData.command);

  if (strcmp(myData.command, "FWD") == 0) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else if (strcmp(myData.command, "STOP") == 0) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
  }
  // Add more commands as needed
}

void setup() {
  Serial.begin(115200);
  
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(onDataRecv);
  Serial.println("ESP-NOW initialized, waiting for data");
}

void loop() {}

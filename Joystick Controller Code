
#include <BluetoothSerial.h>
//BLuetooth serial object 
BluetoothSerial SerialBT;

//Define joystick pins
 int joyXPin = 35; 
 int joyYPin = 32;
 int joyButtonPin = 33;

void setup() {
  Serial.begin(115200); //same bps as motor driver code 
  
  // Initialize the joystick pins
  pinMode(joyXPin, INPUT);
  pinMode(joyYPin, INPUT);
  pinMode(joyButtonPin, INPUT_PULLUP); //button is includd in a lot but will it work 

//connect 
  SerialBT.begin("ESP32_Sender"); //bluetooth device name

  //Connect to motor driver ESP32 
  String slaveMAC = "78:21:84:7C:C1:56"; // this is the MAC address of your motor driver ESP32 
  if (SerialBT.connect(slaveMAC)) {
    Serial.println("Connected to motor driver");
  } else {
    Serial.println("u cant connect bc i hate you");
  }
}

void loop() {
  // Read joystick values
  int joyX = analogRead(joyXPin);
  int joyY = analogRead(joyYPin);
  int joyButton = digitalRead(joyButtonPin);

  //print to look at what values im recieving 
  Serial.print("X: ");
  Serial.print(joyX);
  Serial.print(", Y: ");
  Serial.print(joyY);
  Serial.print(", Button: ");
  Serial.println(joyButton);

  //Send joystick values over Bluetooth to motor driver ESP32 
  if (SerialBT.connected()) {
    SerialBT.print(joyX);
    SerialBT.print(",");
    SerialBT.print(joyY);
    SerialBT.print(",");
    SerialBT.println(joyButton);
  } else {
    Serial.println("Bluetooth not connected");
  }

  // Delay for stability
  delay(100);
}



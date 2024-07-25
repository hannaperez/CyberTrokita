//controller code, takes analog input from joystick and sends it to motor driver 
//MASTER JOYSTICK ESP32 (sender)

#include <BluetoothSerial.h>

BluetoothSerial SerialBT; //Bluetooth serial object to be able to use bluetooth 

//Define joystick pins
 int joyXPin = 35; 
 int joyYPin = 32;
 
 //not using joybutton pin rn
 //int joyButtonPin = 21; //include this in case you need something later that requires a buttonstate 
 

String MACadd = "40:22:D8:3C:38:52"; //MAC address for JOYSTICK SENDER ESP32
uint8_t address[6] = {0x40, 0x22, 0xD8, 0x3C, 0x38, 0x52}; 
String slaveESP32name = "MotorDriverESP32"; //this is the name of the other (slave/reciever) ESP32 
bool connected = false; 

void setup() {
  Serial.begin(115200); //same bps as motor driver code 

  
  // Initialize the joystick pins
  pinMode(joyXPin, INPUT);
  pinMode(joyYPin, INPUT);
  //pinMode(joyButtonPin, INPUT_PULLUP); //button is included in a lot but will it work 

  //Starting bluetooth connection  
  SerialBT.begin("JoystickESP32", true); //this is my master esp32, sets device name 
  Serial.println("This device is your master device.");
  connected = SerialBT.connect(slaveESP32name); //connecting specifically to the other esp32

  if (connected) {
    Serial.println("Yay!!! Joystick is connected to Motor driver");
  } 
  else {
    while(!SerialBT.connected(10000)){
    Serial.println("u cant connect bc i hate you. restart or kill yourself");
    }
  }

}


//Use Serial.begin(115200);: For serial communication with your computer, typically for debugging purposes.
//Use SerialBT: For Bluetooth communication between your ESP32 and other Bluetooth devices.

void loop() {
  // Read joystick values
  int joyX = analogRead(joyXPin);
  int joyY = analogRead(joyYPin);
  //int joyButton = digitalRead(joyButtonPin);

  //print to look at what values im recieving 
  Serial.print("X: ");
  Serial.print(joyX);
  Serial.print(", Y: ");
  Serial.print(joyY);
  Serial.print(", Button: ");
  //Serial.println(joyButton);

  //Send joystick values over Bluetooth to motor driver ESP32 
  if (SerialBT.connected()) {
    SerialBT.print("X:");
    SerialBT.println(joyX);
    SerialBT.print(",Y:");
    SerialBT.print(joyY);
    SerialBT.print(",B:");
    //SerialBT.println(joyButton);
  } 
  
  else {
    Serial.println("Bluetooth not connected, info is not being sent");
  }

  // Delay for stability
  delay(100);
}

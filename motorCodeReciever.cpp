#include "BluetoothSerial.h"
//micro controller -> motor driver -> dual motors 

// Motor Control Pins 
// Motor A
#define IN1 13 //
#define IN2 14 //
#define ENA 25 // PWM pin for speed control
// Motor B 
#define IN3 12
#define IN4 27
#define ENB 33 // PWM pin for speed control

//speeds 
int motorSpeedA = 0;
int motorSpeedB = 0;

//Bluetooth Serial object
BluetoothSerial SerialBT;

void setup() { //setup code runs once 

  Serial.begin(115200); //bits per second, too fast?

  //set motor control pins as outputs
  pinMode(IN1, OUTPUT); // A
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT); 
  pinMode(IN3, OUTPUT); // B
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  //Initialize motor state, everything is off/not moving
  digitalWrite(IN1, LOW); // A
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 255);
  digitalWrite(IN3, LOW); // B
  analogWrite(IN4, LOW);
  digitalWrite(ENB, 255);

  SerialBT.begin("ESP32_Motor_Control"); //bluetooth device name
  Serial.println("Bluetooth Connection Successful");
}

//maybe change the code so that the movements are in functions outside of the loop 

void loop() {//main

  if (SerialBT.available()) { //if this ESP32 has recieved info to move stuff around 
    // Read joystick values from Bluetooth, can turn into a separate function
    String joystickData = SerialBT.readStringUntil('\n');
    int delimiterIndex = joystickData.indexOf(',');
    int xAxis = joystickData.substring(0, delimiterIndex).toInt(); //uses analog joystick works on X and Y axis 
    int yAxis = joystickData.substring(delimiterIndex + 1).toInt();

    //Y-Axis, forward/backward
    if (yAxis < 470) { //10-bit reading (0-1023) of the voltage (5v or 3.3v double check your hardware)
      // Motor A
      digitalWrite(IN1, HIGH); //charge is on one side so that it can go backward
      digitalWrite(IN2, LOW);
      //MotorB
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);

    //declining y-axis readings go backward from 470 to 0
    //map(value, fromLow, fromHigh, toLow, toHigh)
    // value: the number to map.
    // fromLow: the lower bound of the value’s current range.
    // fromHigh: the upper bound of the value’s current range.
    // toLow: the lower bound of the value’s target range.
    // toHigh: the upper bound of the value’s target range.
    //converting ranges

      motorSpeedA = map(yAxis, 470, 0, 0, 255);
      motorSpeedB = map(yAxis, 470, 0, 0, 255);
    } 
    
    else if (yAxis > 550) { // Move forward
      // Motor A
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      // Motor B
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);

      // Converting ranges
      motorSpeedA = map(yAxis, 550, 1023, 0, 255);
      motorSpeedB = map(yAxis, 550, 1023, 0, 255);
    } 
    
    //in middle of range, motors are not moving 
    else { 
      motorSpeedA = 0;
      motorSpeedB = 0;
    }

    //X-Axis: left and right
    if (xAxis < 470) { 
    // Convert the declining X-axis readings from 470 to 0 into increasing 0 to 255 value

      int xMapped = map(xAxis, 470, 0, 0, 255);

    // Move to left - decrease left motor speed, increase right motor speed

      motorSpeedA = motorSpeedA - xMapped;
      motorSpeedB = motorSpeedB + xMapped;

      // Confine the range from 0 to 255
      if (motorSpeedA < 0) 
        motorSpeedA = 0;
      if (motorSpeedB > 255)
        motorSpeedB = 255;
    } 
    
    else if (xAxis > 550) { //move right
      int xMapped = map(xAxis, 550, 1023, 0, 255);
      // Convert the increasing X-axis readings from 550 to 1023 into 0 to 255 value 
      motorSpeedA = motorSpeedA + xMapped;
      motorSpeedB = motorSpeedB - xMapped;
      // Confine the range from 0 to 255
      if (motorSpeedA > 255) 
        motorSpeedA = 255;
      if (motorSpeedB < 0) 
        motorSpeedB = 0;
    }

    //prevent issues at low speeds, adjust this value based on when your motors start moving
    if (motorSpeedA < 70)  //70 value is to test can be readjusted 
      motorSpeedA = 0;
    if (motorSpeedB < 70) 
      motorSpeedB = 0;

    analogWrite(ENA, motorSpeedA); // Send PWM signal to motor A
    analogWrite(ENB, motorSpeedB); // Send PWM signal to motor B
  }
}

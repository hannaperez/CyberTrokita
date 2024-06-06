//basic code 
//double check libraries needed (to be installed) - L298N motor controller 
//micro controller -> motor driver -> dual motors 


// Motor Control Pins 
//Motor A
#define IN1 pin#
#define IN2 differentPin#
#define ENA differentPin# //PWM pin for speed control
//Motor B 
#define IN3 pin#
#define IN4 differentPin#
#define ENB differentPin# //PWM pin for speed control
//speeds 
int motorSpeedA= 0;
int motorSpeedB = 0;

void setup() { //set up code, runs once 
  Serial.begin(115300); //bits per second, too fast?

  //set motor control pins as outputs 
  pinMode(IN1, OUTPUT); //A
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); //B
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  //Initialize motor state, everything is off/not moving 
  digitalWrite(IN1, LOW); //A
  digitalWrite(IN2, LOW);
  digitalWrite(ENA, 0); 
  digitalWrite(IN3, LOW); //B
  digitalWrite(IN4, LOW);
  digitalWrite(ENB, 0); 
}

//maybe change code so that the movements are in functions outside loop
void loop() { //main

  //written using (analog)joystick, works on the X and Y axis 
  int xAxis = analogRead(A0); 
  int yAxis = analogRead(A1);

  //Y-Axis, forward/backward

  //move backward
  if(yAxis < 470){ //10-bit reading (0-1023) of the voltage (5v or 3.3v double check your hardware)
    //Motor A
    digitalWrite(IN1, HIGH); //charge is on one side so that it can go backwards
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

    else if (yAxis > 550){
      //motor A
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      //motor B
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);

      //converting ranges 
      motorSpeedA = map(yAxis, 550, 1023, 0, 255);
      motorSpeedB = map(yAxis, 550, 1023, 0, 255);
    }

    //in middle of range, motors are not moving 
    else{
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
    if (motorSpeedA < 0) {
      motorSpeedA = 0;
    }
    if (motorSpeedB > 255) {
      motorSpeedB = 255;
    }
}

if (xAxis > 550) {
    // Convert the increasing X-axis readings from 550 to 1023 into 0 to 255 value
    int xMapped = map(xAxis, 550, 1023, 0, 255);
    // Move right - decrease right motor speed, increase left motor speed
    motorSpeedA = motorSpeedA + xMapped;
    motorSpeedB = motorSpeedB - xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedA > 255) {
      motorSpeedA = 255;
    }
    if (motorSpeedB < 0) {
      motorSpeedB = 0;
    }
  }

  //prevent issues at low speeds, adjust this value based on when your motors start moving
  if (motorSpeedA < 70) { //70 is value to test 
    motorSpeedA = 0;
  }
  if (motorSpeedB < 70) {
    motorSpeedB = 0;
  }
  analogWrite(ENA, motorSpeedA); // Send PWM signal to motor A
  analogWrite(ENB, motorSpeedB); // Send PWM signal to motor B
}

/*
  ESP32 PS3 Controller Robot Car
  esp32-ps3-final.ino
  Control multiple devices with PS3 controller
  Requires ESP32-PS3 Library - https://github.com/jvpernis/esp32-ps3
  
  DroneBot Workshop 2023
  https://dronebotworkshop.com
*/

// Include required libraries
#include <Ps3Controller.h>
#include <ESP32Servo.h>
#include <Wire.h>

// Define LED Pins
#define LED1_PIN 4
#define LED2_PIN 16
#define LED3_PIN 15

// Define motor driver pins
#define PWMA_PIN 22
#define AIN1_PIN 18
#define AIN2_PIN 5
#define PWMB_PIN 23
#define BIN1_PIN 19
#define BIN2_PIN 21

// Define Servo Pins
#define SERVO_PIN 13
#define MOT1_PIN 9
#define MOT2_PIN 10
#define LOA_PIN 5

// Variables to hold LED states
bool led1State = false;
bool led2State = false;
bool led3State = false;

// Define motor PWM Parameters
const int motorFreq = 1000;
const int motorResolution = 8;

// Define channels for each motor
const int motorAChannel = 3;
const int motorBChannel = 4;

// Variables for Motor PWM values
int motorAPWM = 0;
int motorBPWM = 0;

// Variables for motor direction - true=forward
bool motorDir = true;

// Variables for left joystick values
int leftX = 0;
int leftY = 0;

// Create servo object
Servo tiltServo;

// Controller Right Joystick Values
int rightX;
int rightY;

// Servo Position
int servoPos = 90;

// Define additional servo motors
Servo mot1; // First motor (connected to pin 9)
Servo mot2; // Second motor (connected to pin 10)
Servo loa;  // Loading servo motor (connected to pin 5)

boolean startSequenceA = false;
boolean startSequenceB = false;
boolean startSequenceC = false;
boolean loaSequenceA = false;

void setup() {
  // Setup Serial Monitor for testing
  Serial.begin(115200);

  // Define Callback Function
  Ps3.attach(notify);
  // Define On Connection Function
  Ps3.attachOnConnect(onConnect);
  // Emulate console as specific MAC address (change as required)
  Ps3.begin("38:4f:f0:00:e8:0a");

  // Set LED pins as outputs
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);

  // Attach Servo to pin
  tiltServo.attach(SERVO_PIN);

  // Home at 90 degrees
  tiltServo.write(servoPos);

  // Set motor controller pins as outputs
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);

  // Set channel Parameters for each motor
  ledcSetup(motorAChannel, motorFreq, motorResolution);
  ledcSetup(motorBChannel, motorFreq, motorResolution);

  // Attach Motor PWM pins to corresponding channels
  ledcAttachPin(PWMA_PIN, motorAChannel);
  ledcAttachPin(PWMB_PIN, motorBChannel);

  // Initialize additional servos
  mot1.attach(MOT1_PIN);
  mot2.attach(MOT2_PIN);
  loa.attach(LOA_PIN);

  loa.write(0); // Move the servo to the home position
  
  // Initialize the signals to 1000 (original direction)
  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);

  Serial.println("Ready.");
}

void loop() {}

void notify() {
  // Get Left Joystick value (for driving motors)
  leftX = (Ps3.data.analog.stick.lx);
  leftY = (Ps3.data.analog.stick.ly);

  // Get Right Joystick value (for controlling servo tilt)
  rightX = (Ps3.data.analog.stick.rx);
  rightY = (Ps3.data.analog.stick.ry);

  // Control the tiltServo with the right joystick Y axis
  if (rightY < -100) {
    servoPos = 90; // Return to home (90 degrees)
    tiltServo.write(servoPos);
    delay(10);
  } else {
    if (rightY < -10 && servoPos < 180) {
      servoPos++;
      tiltServo.write(servoPos);
      delay(10);
    }
    if (rightY > 10 && servoPos > 0) {
      servoPos--;
      tiltServo.write(servoPos);
      delay(10);
    }
  }

  // Determine motor direction from left joystick Y axis position
  if (leftY < 0) {
    // Direction is forward
    motorDir = true;
  } else {
    // Direction is reverse
    motorDir = false;
  }

  // Convert joystick values to positive 0 - 255
  int speedX = (abs(leftX) * 2);
  int speedY = (abs(leftY) * 2);

  // Factor in the X axis value to determine motor speeds (assume Motor A is Left motor going forward)
  if (leftX < -10) {
    // Motor B faster than Motor A
    motorAPWM = speedY - speedX;
    motorBPWM = speedY + speedX;

  } else if (leftX > 10) {
    // Motor A faster than Motor B
    motorAPWM = speedY + speedX;
    motorBPWM = speedY - speedX;

  } else {
    // Control is in middle, both motors same speed
    motorAPWM = speedY;
    motorBPWM = speedY;
  }

  // Ensure that speed values remain in range of 0 - 255
  motorAPWM = constrain(motorAPWM, 0, 255);
  motorBPWM = constrain(motorBPWM, 0, 255);

  // Drive the motors
  moveMotors(motorAPWM, motorBPWM, motorDir);

  // Print to Serial Monitor
  Serial.print("X value = ");
  Serial.print(leftX);
  Serial.print(" - Y value = ");
  Serial.print(leftY);
  Serial.print(" - Motor A = ");
  Serial.print(motorAPWM);
  Serial.print(" - Motor B = ");
  Serial.println(motorBPWM);

  // Circle Button (Button A) - LED1 control
  if (Ps3.event.button_down.circle) {
    Serial.println("Circle pressed, performing Full auto sequence");
    led1State = true;
    digitalWrite(LED1_PIN, led1State);
    performAction1(); // Full auto sequence
  } else {
    led1State = false;
    digitalWrite(LED1_PIN, led1State);
  }

  // Square Button (Button B) - LED2 control
  if (Ps3.event.button_down.square) {
    Serial.println("Square pressed, performing 3-round burst sequence");
    led2State = true;
    digitalWrite(LED2_PIN, led2State);
    performAction2(); // 3-round burst sequence
  } else {
    led2State = false;
    digitalWrite(LED2_PIN, led2State);
  }

  // Cross Button (Button C) - LED3 control
  if (Ps3.event.button_down.cross) {
    Serial.println("Cross pressed, performing One-shot sequence");
    led3State = true;
    digitalWrite(LED3_PIN, led3State);
    performAction3(); // One-shot sequence
  } else {
    led3State = false;
    digitalWrite(LED3_PIN, led3State);
  }
}

// On Connection function
void onConnect() {
  // Print to Serial Monitor
  Serial.println("Connected.");
}

// Motor movement function
void moveMotors(int mtrAspeed, int mtrBspeed, bool mtrdir) {
  // Set direction pins
  if (!mtrdir) {
    // Move in reverse
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);

  } else {
    // Move Forward
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
  }

  // Drive motors with PWM
  ledcWrite(motorAChannel, mtrAspeed);
  ledcWrite(motorBChannel, mtrBspeed);
}

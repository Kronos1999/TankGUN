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

// Variables to hold LED states
bool led1State = false;
bool led2State = false;
bool led3State = false;

// Define motor driver pins
#define PWMA_PIN 22
#define AIN1_PIN 18
#define AIN2_PIN 5
#define PWMB_PIN 23
#define BIN1_PIN 19
#define BIN2_PIN 21

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

// Variables for right joystick values
int rightX = 0;
int rightY = 0;

// RGB LED Connections
const int ledRed = 25;
const int ledGreen = 26;
const int ledBlue = 27;

// RGB LED PWM Properties
const int ledFreq = 5000;
const int ledResolution = 8;

// Define channels for each RGB LED color
const int redChannel = 5;
const int greenChannel = 1;
const int blueChannel = 2;

// Variables for RGB LED PWM values
int redPWM = 0;
int greenPWM = 0;
int bluePWM = 0;

// Create servo object
Servo srvmtr;

// Controller Left Joystick Values
int leftX;
int leftY;

// Servo Position
int servoPos = 90;

// Servo Pin
#define SERVO_PIN 13

// Callback Function
void notify() {

  // Get Left Joystick value
  leftX = (Ps3.data.analog.stick.lx);
  leftY = (Ps3.data.analog.stick.ly);

  // Get Right Joystick value
  rightX = (Ps3.data.analog.stick.rx);
  rightY = (Ps3.data.analog.stick.ry);

  // Check if left joystick was moved upward, indicating return to home (90 degrees)
  if (leftY < -100) {
    servoPos = 90;
    srvmtr.write(servoPos);
    delay(10);
  } else {

    // See if left joystick was moved left or right, and in what direction. If moved, move servo in that direction
    if (leftX < -10 && servoPos < 180) {
      servoPos++;
      srvmtr.write(servoPos);
      delay(10);
    }
    if (leftX > 10 && servoPos > 0) {
      servoPos--;
      srvmtr.write(servoPos);
      delay(10);
    }
  }

  //Determine motor direction from right joystick Y axis position
  if (rightY < 0) {
    // Direction is forward
    motorDir = true;
  } else {
    // Direction is reverse
    motorDir = false;
  }

  // Convert joystick values to positive 0 - 255
  int speedX = (abs(rightX) * 2);
  int speedY = (abs(rightY) * 2);

  // Factor in the X axis value to determine motor speeds (assume Motor A is Left motor going forward)
  if (rightX < -10) {
    // Motor B faster than Motor A
    motorAPWM = speedY - speedX;
    motorBPWM = speedY + speedX;

  } else if (rightX > 10) {
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
  Serial.print(rightX);
  Serial.print(" - Y value = ");
  Serial.print(rightY);
  Serial.print(" - Motor A = ");
  Serial.print(motorAPWM);
  Serial.print(" - Motor B = ");
  Serial.println(motorBPWM);

  // Cross button - LED1 momentary control
  if (Ps3.event.button_down.cross) {
    Serial.println("Cross pressed");
    led1State = true;
    digitalWrite(LED1_PIN, led1State);
  }
  if (Ps3.event.button_up.cross) {
    Serial.println("Cross released");
    led1State = false;
    digitalWrite(LED1_PIN, led1State);
  }

  // Triangle Button - LED2 toggle control
  if (Ps3.event.button_down.triangle) {
    Serial.println("Triangle presssed");
    led2State = !led2State;
    digitalWrite(LED2_PIN, led2State);
  }

  // Square Button - LED3 on
  if (Ps3.event.button_down.square) {
    Serial.println("Square pressed");
    led3State = true;
    digitalWrite(LED3_PIN, led3State);
  }

  // Circle Button - LED3 off
  if (Ps3.event.button_down.circle) {
    Serial.println("Circle pressed");
    led3State = false;
    digitalWrite(LED3_PIN, led3State);
  }

  // Shoulder & Trigger button changes for RGB LED
  // Set RGB values based upon analog button values
  if (abs(Ps3.event.analog_changed.button.l1)) {
    // Left Shoulder - Red
    redPWM = int(Ps3.data.analog.button.l1);
  }

  if (abs(Ps3.event.analog_changed.button.l2)) {
    // Left Trigger - Green
    greenPWM = int(Ps3.data.analog.button.l2);
  }

  if (abs(Ps3.event.analog_changed.button.r2)) {
    // Right Trigger - Blue
    bluePWM = int(Ps3.data.analog.button.r2);
  }

  // Right Shoulder button turns on all LED segments full, for white
  if (abs(Ps3.event.analog_changed.button.r1)) {
    // Right Shoulder - White
    redPWM = 255;
    greenPWM = 255;
    bluePWM = 255;
  }

  // Write LED values to RGB LED
  ledcWrite(redChannel, redPWM);
  ledcWrite(greenChannel, greenPWM);
  ledcWrite(blueChannel, bluePWM);

  // Print to Serial Monitor
  Serial.print("R = ");
  Serial.print(redPWM);
  Serial.print(" - G = ");
  Serial.print(greenPWM);
  Serial.print(" - B = ");
  Serial.println(bluePWM);
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

// Read the TOF Sensor
void sensorTOFRead(unsigned char addr, unsigned char* datbuf, unsigned char cnt) {
  unsigned short result = 0;
  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(82);  // transmit to device #82 (0x52)
  // the address specified in the datasheet is 164 (0xa4)
  // but i2c adressing uses the high 7 bits so it's 82
  Wire.write(byte(addr));  // sets distance data address (addr)
  Wire.endTransmission();  // stop transmitting
  // step 2: wait for readings to happen
  delay(1);  // datasheet suggests at least 30uS
  // step 3: request reading from sensor
  Wire.requestFrom(82, cnt);  // request cnt bytes from slave device #82 (0x52)
  // step 5: receive reading from sensor
  if (cnt <= Wire.available()) {  // if two bytes were received
    *datbuf++ = Wire.read();      // receive high byte (overwrites previous reading)
    *datbuf++ = Wire.read();      // receive low byte as lower 8 bits
  }
}

// Get distance from TOF sensor
int readTOFDistance() {
  sensorTOFRead(0x00, i2c_rx_buf, 2);
  lenth_val = i2c_rx_buf[0];
  lenth_val = lenth_val << 8;
  lenth_val |= i2c_rx_buf[1];
  delay(300);
  return lenth_val;
}

int serial_putc(char c, struct __file*) {
  Serial.write(c);
  return c;
}

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

  // Set RGB LED Parameters for each color
  ledcSetup(redChannel, ledFreq, ledResolution);
  ledcSetup(greenChannel, ledFreq, ledResolution);
  ledcSetup(blueChannel, ledFreq, ledResolution);

  // Attach RGB LED pins to corresponding channels
  ledcAttachPin(ledRed, redChannel);
  ledcAttachPin(ledGreen, greenChannel);
  ledcAttachPin(ledBlue, blueChannel);

  // Attach Servo to pin
  srvmtr.attach(SERVO_PIN);

  // Home at 90 degrees
  srvmtr.write(servoPos);

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

  // Start Wire library for I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // Print to Serial Monitor
  Serial.println("Ready.");
}

void loop() {}

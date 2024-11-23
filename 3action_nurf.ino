#include <Servo.h>

Servo esc1;
Servo esc2;
Servo myServo;

const int button3 = A3;
const int button4 = A4;
const int button5 = A5;

void setup() {
  Serial.begin(9600);

  esc1.attach(9);
  esc2.attach(10);
  myServo.attach(5);

  pinMode(button3, INPUT_PULLUP);  //Wire the switch so that one side is hot. 
  pinMode(button4, INPUT_PULLUP);  //The controller is looking for high voltage.
  pinMode(button5, INPUT_PULLUP);  //when the button is pressed button 5 will go high.

  myServo.write(30); // Initial position of the servo

    // Initialize the signals to 1000 (original direction)
esc1.writeMicroseconds(1000);
esc2.writeMicroseconds(1000);
}

void loop() {
  if (digitalRead(button3) == LOW) { // One button press one shot.
    Serial.println("Button 3 pressed");
    startESCs();
    moveServoOnce();
    rampDownESCs();
  }

  if (digitalRead(button4) == LOW) { // 3 round burst
    Serial.println("Button 4 pressed");
    startESCs();
    moveServoMultipleTimes(3);
    rampDownESCs();
  }

  if (digitalRead(button5) == LOW) { // full auto, it will keep firing until the button is released.
    Serial.println("Button 5 pressed");
    startESCs();
    while (digitalRead(button5) == LOW) {
      moveServoOnce();
    }
    rampDownESCs();
  }
}

void startESCs() {   // This is the code for the start sequence.
  for (int i = 1000; i <= 1500; i += 10) {
    esc1.writeMicroseconds(i);
    esc2.writeMicroseconds(i);
    delay(20);
  }
  Serial.println("ESCs at half throttle");
}

void rampDownESCs() { //This is the code for the shut down sequence.
  for (int i = 1500; i >= 1000; i -= 10) {
    esc1.writeMicroseconds(i);
    esc2.writeMicroseconds(i);
    delay(20);
  }
  Serial.println("ESCs ramped down");
}
// I did't want the drone motors draw a lot of amps by going to full speed and full stop abruptly.

void moveServoOnce() { //like it says move the dart plunger once.
  myServo.write(130);
  delay(300);
  myServo.write(30);
  delay(300);
  Serial.println("Servo moved once");
}

void moveServoMultipleTimes(int times) { // this is 3 round burst
  for (int i = 0; i < times; i++) {
    moveServoOnce();
  }
  Serial.println("Servo moved multiple times");
}
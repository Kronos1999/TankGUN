//Grok is king C++23 with Arduino: PS3 Controlled Tank with Servo and ESCs
#include <Arduino.h>
#include <Ps3Controller.h>
#include <ESP32Servo.h>

// --- PIN ASSIGNMENTS ---
const int motorL_PWM = 25; // Speed L (Level Shifter)
const int motorR_PWM = 27; // Speed R (Level Shifter)
const int motorL_IN1 = 26; // Dir L1 (Direct)
const int motorL_IN2 = 14; // Dir L2 (Direct)
const int motorR_IN3 = 33; // Dir R1 (Direct)
const int motorR_IN4 = 32; // Dir R2 (Direct)

const int servoPin = 18;
const int escPin1  = 19;
const int escPin2  = 21;
const int elevPin  = 22; 

// --- OBJECTS & VARIABLES ---
Servo myServo, myESC1, myESC2, elevServo;
float elevPos = 90.0;
const float elevMin = 45.0, elevMax = 135.0, moveSpeed = 0.5;

// --- MOTOR DRIVE LOGIC ---
void driveMotor(String side, int pwmPin, int in1, int in2, int speed) {
    int deadzone = 20; // Increased to handle drift
    int finalPWM = map(abs(speed), 0, 128, 0, 255);

    if (abs(speed) < deadzone) {
        analogWrite(pwmPin, 0);
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        return; 
    }

    analogWrite(pwmPin, finalPWM);

    if (speed > 0) { // Forward Logic
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        Serial.printf("[%s] FORWARD | Speed: %d | PWM: %d\n", side.c_str(), speed, finalPWM);
    } else { // Reverse Logic
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        Serial.printf("[%s] REVERSE | Speed: %d | PWM: %d\n", side.c_str(), speed, finalPWM);
    }
}

void stopDriveMotors() {
    analogWrite(motorL_PWM, 0);
    analogWrite(motorR_PWM, 0);
    digitalWrite(motorL_IN1, LOW); digitalWrite(motorL_IN2, LOW);
    digitalWrite(motorR_IN3, LOW); digitalWrite(motorR_IN4, LOW);
    Serial.println(">>> MOTORS ESTOPPED <<<");
}

// --- PS3 SEQUENCES ---
void executeCrossSequence() {
    stopDriveMotors();
    Serial.println("EVENT: Cross Sequence Started");

    // Spin up ESCs
    myESC1.write(120); 
    myESC2.write(120); 
    delay(140); 

    // SNAP TO LOADED (Full Speed)
    myServo.write(145); 
    delay(300); // Direct dwell for MG996R travel time

    // SNAP TO CENTER (Full Speed)
    myServo.write(90); 
    delay(300); // Direct dwell for MG996R travel time

    // Optimized Ramp Down
    Serial.println("EVENT: Ramping down ESCs...");
    for (int s = 120; s >= 0; s -= 10) { // Increased step to 10 for faster recovery
        myESC1.write(s); 
        myESC2.write(s); 
        delay(20); 
    }

    Serial.println("EVENT: Cross Sequence Complete");
}


void executeCircleSequence() {
    stopDriveMotors();
    Serial.println("EVENT: Circle Sequence Started");
    myESC1.write(120); myESC2.write(120); delay(200); 
    for (int i = 0; i < 3; i++) {
        myServo.write(145); delay(300);// dwell at loaded position
        myServo.write(90);
        if (i < 2) delay(400); // Shorter delay between spins
    }
    for (int s = 120; s >= 0; s-=5) { myESC1.write(s); myESC2.write(s); delay(30); }
    Serial.println("EVENT: Circle Sequence Complete");
}
void onConnect() {
    Serial.println("SYSTEM: PS3 Controller Connected!");
    Ps3.setPlayer(1); // Light only Player 1 LED
}


void notify() {
    Serial.println("DEBUG: Notify callback triggered");
    if (Ps3.event.button_down.cross) {
        Serial.println("DEBUG: Cross button down event");
        executeCrossSequence();
    }
    if (Ps3.event.button_down.circle) {
        Serial.println("DEBUG: Circle button down event");
        executeCircleSequence();
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(motorL_PWM, OUTPUT); pinMode(motorL_IN1, OUTPUT); pinMode(motorL_IN2, OUTPUT);
    pinMode(motorR_PWM, OUTPUT); pinMode(motorR_IN3, OUTPUT); pinMode(motorR_IN4, OUTPUT);
    
    // Explicitly initialize motors to stopped state
    digitalWrite(motorL_IN1, LOW); digitalWrite(motorL_IN2, LOW);
    digitalWrite(motorR_IN3, LOW); digitalWrite(motorR_IN4, LOW);
    analogWrite(motorL_PWM, 0);
    analogWrite(motorR_PWM, 0);

    ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
      
    myServo.attach(servoPin, 500, 2400);
    myESC1.attach(escPin1, 1000, 2000);
    myESC2.attach(escPin2, 1000, 2000);
    elevServo.attach(elevPin, 500, 2400);

    // ESC arming sequence
    myESC1.write(0); myESC2.write(0); // Min pulse
    delay(2000); // Wait for beeps
    myESC1.write(180); myESC2.write(180); // Max briefly
    delay(100);
    myESC1.write(0); myESC2.write(0); // Back to min
    delay(1000);
    Serial.println("SYSTEM: ESCs armed");

    myServo.write(90); elevServo.write(90);
    
    Ps3.attach(notify);
    Ps3.attachOnConnect(onConnect);
    Ps3.begin("38:4f:f0:00:e8:0a"); // Change to your MAC
    Serial.println("SYSTEM: Initialization Complete. Awaiting PS3 Controller...");
}

void loop() {
    if (!Ps3.isConnected()) {
        static unsigned long lastMsg = 0;
        if (millis() - lastMsg > 2000) {
            Serial.println("SYSTEM: Searching for Controller...");
            lastMsg = millis();
        }
        return;
    }

    // --- STEERING ---
    int lx = Ps3.data.analog.stick.lx;
    int ly = -Ps3.data.analog.stick.ly;
    int leftSpeed = constrain(ly + lx, -128, 128);
    int rightSpeed = constrain(ly - lx, -128, 128);
    Serial.printf("DEBUG: lx=%d, ly=%d, Left Speed=%d, Right Speed=%d\n", lx, ly, leftSpeed, rightSpeed); // Print every loop for diag
    driveMotor("LEFT", motorL_PWM, motorL_IN1, motorL_IN2, leftSpeed);
    driveMotor("RIGHT", motorR_PWM, motorR_IN3, motorR_IN4, rightSpeed);

    // --- ELEVATION ---
    int ry = Ps3.data.analog.stick.ry; 
    if (abs(ry) > 10) {// Deadzone
        elevPos -= (ry / 128.0) * moveSpeed;// change -/+= to invert
        elevPos = constrain(elevPos, elevMin, elevMax);
        elevServo.write((int)elevPos);
        Serial.printf("DEBUG: Elevation Servo at %d\n", (int)elevPos);
    }

    // --- TRIANGLE CONTINUOUS SERVO MOTION ---
    if (Ps3.data.button.triangle) {
    stopDriveMotors();
    myESC1.write(130); myESC2.write(130); delay(150);
    bool held = true;
    while (held) {
        // Go to 150 degrees at maximum speed
        myServo.write(145); 
        delay(300); // Dwell for 300 milliseconds to ensure full travel

        if (!Ps3.data.button.triangle) { held = false; break; }

        // Go to 90 degrees at maximum speed
        myServo.write(90); 
        delay(300); // Dwell for 300 milliseconds

        if (!Ps3.data.button.triangle) { held = false; break; }
    }
    // ... rest of your ESC ramp down code

        if (!Ps3.data.button.triangle) {
            Serial.println("EVENT: Triangle Released. Ramping down ESCs...");
            myServo.write(90);
            for (int s = 130; s >= 0; s--) { myESC1.write(s); myESC2.write(s); delay(10); }
        }
    }
    delay(10); 
}

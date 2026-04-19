// ── motor_driver.ino ────────────────────────────────────────────────────────
// Arduino Uno sketch: receives serial commands from Rubik Pi 3 over USB.
// Drives two DC motors via L298N + sweeps a servo for ultrasonic scanning.
//
// Serial protocol (9600 baud, newline-terminated):
//   "F"      → drive forward
//   "B"      → drive backward
//   "L"      → turn left  (left backward, right forward)
//   "R"      → turn right (left forward, right backward)
//   "S"      → stop motors
//   "V<deg>" → servo to angle (0–180), e.g. "V90" = center
//
// Pin wiring:
//   L298N ENA → pin 13  (Motor A enable — always HIGH)
//   L298N IN1 → pin 12  (Motor A direction)
//   L298N IN2 → pin 11  (Motor A direction)
//   L298N ENB → pin 10  (Motor B enable — always HIGH)
//   L298N IN3 → pin 9   (Motor B direction)
//   L298N IN4 → pin 8   (Motor B direction)
//   Servo sig → pin 6   (PWM for ultrasonic sweep servo)

#include <Servo.h>

const int enA = 13;
const int in1 = 12;
const int in2 = 11;
const int enB = 10;
const int in3 = 9;
const int in4 = 8;
const int servoPin = 6;

Servo sweepServo;

// ── Motor orientation ─────────────────────────────────────────────────────────
// The two rear motors are mirror-mounted so one must be logically inverted.
// If the rover circles LEFT  → set REVERSE_MOTOR_B 1 (try this first)
// If the rover circles RIGHT → set REVERSE_MOTOR_B 0 and REVERSE_MOTOR_A 1
// If it still circles after either change, swap the two wires on that motor.
#define REVERSE_MOTOR_A 0
#define REVERSE_MOTOR_B 1

// Macro: resolve actual HIGH/LOW for a pin given inversion flag
#define PIN_A(a, b, rev) digitalWrite(in1, (rev) ? (b) : (a)); digitalWrite(in2, (rev) ? (a) : (b))
#define PIN_B(a, b, rev) digitalWrite(in3, (rev) ? (b) : (a)); digitalWrite(in4, (rev) ? (a) : (b))

void driveForward() {
  PIN_A(LOW,  HIGH, REVERSE_MOTOR_A);
  PIN_B(HIGH, LOW,  REVERSE_MOTOR_B);
}

void driveBackward() {
  PIN_A(HIGH, LOW,  REVERSE_MOTOR_A);
  PIN_B(LOW,  HIGH, REVERSE_MOTOR_B);
}

void turnLeft() {
  // Left motor backward, right motor forward
  PIN_A(HIGH, LOW,  REVERSE_MOTOR_A);
  PIN_B(HIGH, LOW,  REVERSE_MOTOR_B);
}

void turnRight() {
  // Left motor forward, right motor backward
  PIN_A(LOW,  HIGH, REVERSE_MOTOR_A);
  PIN_B(LOW,  HIGH, REVERSE_MOTOR_B);
}

void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// ─── setup / loop ───────────────────────────────────────────────────────────

void setup() {
  Serial.begin(9600);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Enable both motors (full speed — pin 13 can't do PWM)
  digitalWrite(enA, HIGH);
  digitalWrite(enB, HIGH);

  sweepServo.attach(servoPin);
  sweepServo.write(90);  // center (straight ahead)

  stopMotors();
  Serial.println("READY");
}

String inputBuffer = "";

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        handleCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

void handleCommand(String cmd) {
  char action = cmd.charAt(0);

  switch (action) {
    case 'F':
      driveForward();
      Serial.println("OK:FWD");
      break;
    case 'B':
      driveBackward();
      Serial.println("OK:BWD");
      break;
    case 'L':
      turnLeft();
      Serial.println("OK:LEFT");
      break;
    case 'R':
      turnRight();
      Serial.println("OK:RIGHT");
      break;
    case 'S':
      stopMotors();
      Serial.println("OK:STOP");
      break;
    case 'V': {
      int angle = cmd.substring(1).toInt();
      angle = constrain(angle, 0, 180);
      sweepServo.write(angle);
      Serial.print("OK:SERVO:");
      Serial.println(angle);
      break;
    }
    default:
      Serial.println("ERR");
      break;
  }
}

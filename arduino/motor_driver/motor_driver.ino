// ── motor_driver.ino ────────────────────────────────────────────────────────
// Arduino Uno sketch: receives serial commands from Rubik Pi 3 over USB
// and drives two DC motors via an L298N dual H-bridge.
//
// Serial protocol (9600 baud, newline-terminated):
//   "F"  → drive forward
//   "B"  → drive backward
//   "L"  → turn left  (left backward, right forward)
//   "R"  → turn right (left forward, right backward)
//   "S"  → stop
//
// Pin wiring (matching existing hardware):
//   L298N ENA → pin 13  (Motor A enable — not PWM, always HIGH)
//   L298N IN1 → pin 12  (Motor A direction)
//   L298N IN2 → pin 11  (Motor A direction)
//   L298N ENB → pin 10  (Motor B enable — not PWM, always HIGH)
//   L298N IN3 → pin 9   (Motor B direction)
//   L298N IN4 → pin 8   (Motor B direction)

const int enA = 13;
const int in1 = 12;
const int in2 = 11;
const int enB = 10;
const int in3 = 9;
const int in4 = 8;

// ─── motor helpers (direction logic from working code) ──────────────────────

void driveForward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void driveBackward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void turnLeft() {
  // Left motor backward, right motor forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void turnRight() {
  // Left motor forward, right motor backward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
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
    default:
      Serial.println("ERR");
      break;
  }
}

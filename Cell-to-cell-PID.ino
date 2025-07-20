#include <PID_v1.h>
#include <Encoder.h>

// --- Encoder Pins ---
#define ENCODER_LEFT_A  34
#define ENCODER_LEFT_B  35
#define ENCODER_RIGHT_A 32
#define ENCODER_RIGHT_B 33

Encoder encoderLeft(ENCODER_LEFT_A, ENCODER_LEFT_B);
Encoder encoderRight(ENCODER_RIGHT_A, ENCODER_RIGHT_B);

// --- Motor Pins ---
#define MOTOR_LEFT_PWM   25
#define MOTOR_LEFT_DIR   26
#define MOTOR_RIGHT_PWM  27
#define MOTOR_RIGHT_DIR  14

// --- Motion Constants ---
const float WHEEL_DIAMETER = 43.0;                     // mm
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI; // ≈ 135.72 mm
const int   COUNTS_PER_REV = 1200;                     // 3 PPR × 4 × 100:1
const float DIST_PER_COUNT = WHEEL_CIRCUMFERENCE / COUNTS_PER_REV; // ≈ 0.1131 mm
const float CELL_SIZE = 180.0;                         // mm (adjust to your maze)
const float WHEEL_BASE = 100.0;                        // mm (distance between wheels)

// --- PID Parameters ---
double Kp = 1.5, Ki = 0.0, Kd = 0.1;
double leftSetpoint, leftInput, leftOutput;
double rightSetpoint, rightInput, rightOutput;

PID pidLeft(&leftInput, &leftOutput, &leftSetpoint, Kp, Ki, Kd, DIRECT);
PID pidRight(&rightInput, &rightOutput, &rightSetpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);

  pidLeft.SetMode(AUTOMATIC);
  pidRight.SetMode(AUTOMATIC);
  pidLeft.SetOutputLimits(-255, 255);
  pidRight.SetOutputLimits(-255, 255);
}

void loop() {
  moveForwardOneCell();
  delay(500);
  turn90Left();
  delay(500);
  turn90Right();
  delay(500);
  turn180();
  delay(2000);
}

// --- Movement Functions ---

void moveForwardOneCell() {
  encoderLeft.write(0);
  encoderRight.write(0);

  double targetCounts = CELL_SIZE / DIST_PER_COUNT;

  leftSetpoint = targetCounts;
  rightSetpoint = targetCounts;

  while (true) {
    leftInput = encoderLeft.read();
    rightInput = encoderRight.read();

    pidLeft.Compute();
    pidRight.Compute();

    setMotor(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR, leftOutput);
    setMotor(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR, rightOutput);

    if (abs(leftInput - leftSetpoint) < 10 && abs(rightInput - rightSetpoint) < 10)
      break;
  }
  stopMotors();
}

void turn90Left() {
  encoderLeft.write(0);
  encoderRight.write(0);

  float arcLen = (PI * WHEEL_BASE) / 4.0;     // 90° arc
  float targetCounts = arcLen / DIST_PER_COUNT;

  leftSetpoint = -targetCounts;
  rightSetpoint = targetCounts;

  while (true) {
    leftInput = encoderLeft.read();
    rightInput = encoderRight.read();

    pidLeft.Compute();
    pidRight.Compute();

    setMotor(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR, leftOutput);
    setMotor(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR, rightOutput);

    if (abs(leftInput - leftSetpoint) < 10 && abs(rightInput - rightSetpoint) < 10)
      break;
  }
  stopMotors();
}

void turn90Right() {
  encoderLeft.write(0);
  encoderRight.write(0);

  float arcLen = (PI * WHEEL_BASE) / 4.0;
  float targetCounts = arcLen / DIST_PER_COUNT;

  leftSetpoint = targetCounts;
  rightSetpoint = -targetCounts;

  while (true) {
    leftInput = encoderLeft.read();
    rightInput = encoderRight.read();

    pidLeft.Compute();
    pidRight.Compute();

    setMotor(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR, leftOutput);
    setMotor(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR, rightOutput);

    if (abs(leftInput - leftSetpoint) < 10 && abs(rightInput - rightSetpoint) < 10)
      break;
  }
  stopMotors();
}

void turn180() {
  encoderLeft.write(0);
  encoderRight.write(0);

  float arcLen = (PI * WHEEL_BASE) / 2.0;  // 180° arc
  float targetCounts = arcLen / DIST_PER_COUNT;

  leftSetpoint = -targetCounts;
  rightSetpoint = targetCounts;

  while (true) {
    leftInput = encoderLeft.read();
    rightInput = encoderRight.read();

    pidLeft.Compute();
    pidRight.Compute();

    setMotor(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR, leftOutput);
    setMotor(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR, rightOutput);

    if (abs(leftInput - leftSetpoint) < 10 && abs(rightInput - rightSetpoint) < 10)
      break;
  }
  stopMotors();
}

// --- Motor Control Helpers ---

void setMotor(int pwmPin, int dirPin, double speed) {
  bool dir = speed >= 0;
  analogWrite(pwmPin, abs(speed));
  digitalWrite(dirPin, dir ? HIGH : LOW);
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
}

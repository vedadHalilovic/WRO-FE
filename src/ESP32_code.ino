#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <cmath>

HardwareSerial mySerial(2);
Servo steeringServo;
MPU6050 mpu(Wire);

const int ledGreen = 2;
const int button_start = 15;
const int pwma = 13;
const int ain1 = 25;
const int ain2 = 12;

const double k = 0.33;
const double minL = 76.0;
const double minR = 114.0;

int driveSpeed = 130;
float yaw = 0;
bool isTurning = false;
float targetYaw = 0;
int turnDirection = 0;

void stopAll();
void driveForward(int speed);
void updateYaw();
void startTurn();
void checkTurnProgress();

void setup() {
  digitalWrite(ledGreen, LOW);
  pinMode(button_start, INPUT_PULLUP);

  while(button_start == HIGH){
    digitalWrite(ledGreen, HIGH);
    delay(500);
    digitalWrite(ledGreen, LOW);
    delay(500);
  }
  delay(1000);
  Serial.begin(115200);

  mySerial.begin(9600, SERIAL_8N1, 27, 14);

  Wire.begin(21, 22);
  mpu.begin();

  Serial.println("Calibrating... Do not move robot!");
  mpu.calcGyroOffsets(true);

  steeringServo.attach(33);
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(pwma, OUTPUT);
  pinMode(ledGreen, OUTPUT);

  stopAll();
  steeringServo.write(96);
  Serial.println("MPU Ready and Calibrated!");
  digitalWrite(ledGreen, HIGH);
  mySerial.println("a");
}

void loop() {
  updateYaw();

  if (mySerial.available() > 0 && !isTurning) {
    String data = mySerial.readStringUntil('\n');
    data.trim();

    if (data.length() > 0) {
      if (data == "l") {
        targetYaw = yaw - 90.0;
        turnDirection = 1;
        startTurn();
      } else if (data == "r") {
        targetYaw = yaw + 90.0;
        turnDirection = -1;
        startTurn();
      } else if (data == "n") {
        targetYaw = yaw - 45.0;
        turnDirection = 1;
        startTurn();
      } else if (data == "m") {
        targetYaw = yaw + 45.0;
        turnDirection = -1;
        startTurn();
      } else if (data == "t") {
        stopAll();
        delay(1100);
      } else if (data == "e") {
        delay(500);
        stopAll();
      } else {
        int incomingAngle = data.toInt();
        int servoAngle = map(incomingAngle, -90, 90, 40, 150);
        steeringServo.write(servoAngle);
        driveForward(driveSpeed);
      }
    }
  }

  if (isTurning) {
    checkTurnProgress();
  }
}

void updateYaw() {
  mpu.update();
  yaw = mpu.getGyroAngleZ();

  static float lastPrintedYaw = 0;
  if (abs(yaw - lastPrintedYaw) > 0.5) {
    Serial.print("Current Yaw: ");
    Serial.println(yaw);
    lastPrintedYaw = yaw;
  }
}

void startTurn() {
  isTurning = true;
  driveForward(driveSpeed - 35);
  if (turnDirection == 1) steeringServo.write(40);
  else if (turnDirection == -1) steeringServo.write(150);
}

void checkTurnProgress() {
  bool turnComplete = false;

  if (turnDirection == 1) {
    if (yaw > targetYaw) {
      int turnAngle = round(minL - (k * (yaw - targetYaw)));
      steeringServo.write(constrain(turnAngle, 40, 80));
    } else {
      turnComplete = true;
    }
  } else if (turnDirection == -1) {
    if (yaw < targetYaw) {
      int turnAngle = round(minR - (k * (yaw - targetYaw)));
      steeringServo.write(constrain(turnAngle, 110, 150));
    } else {
      turnComplete = true;
    }
  }

  if (turnComplete) {
    //turnCount++;
    steeringServo.write(96);
    isTurning = false;
    delay(500);
    turnDirection = 0;
    mySerial.println("h");
    stopAll();
    delay(1000);
  }
}

void driveForward(int speed) {
  digitalWrite(ain1, LOW);
  digitalWrite(ain2, HIGH);
  analogWrite(pwma, speed);
}

void stopAll() {
  analogWrite(pwma, 0);
  digitalWrite(ain1, LOW);
  digitalWrite(ain2, LOW);
}

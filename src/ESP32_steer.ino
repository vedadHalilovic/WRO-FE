#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>

// ===== Hardware Setup =====
HardwareSerial mySerial(2); // UART2: RX=16, TX=17
Servo steeringServo;
Adafruit_ICM20948 icm;

const int pwma = 13;
const int ain1 = 22;
const int ain2 = 12;

// PWM Settings
const int pwmFreq = 1000;
const int pwmResolution = 8;
int driveSpeed = 200;  // Speed during normal driving
//int turnSpeed = 160;   // Slightly slower for better turning accuracy

// ===== IMU / Navigation Variables =====
unsigned long lastTime;
float yaw = 0;
float gyroBiasZ = 0;
bool isTurning = false;
float targetYaw = 0;
int turnDirection = 0; // 1 for Right, -1 for Left

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, 16, 17);
  
  steeringServo.attach(33);
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  ledcAttach(pwma, pwmFreq, pwmResolution);
  stopAll();

  // Initialize IMU
  if (!icm.begin_I2C(0x68)) {
    Serial.println("Failed to find ICM20948 chip!");
    while (1) delay(10);
  }

  // --- CALIBRATION ---
  Serial.println("Calibrating Gyro... DO NOT MOVE");
  for (int i = 0; i < 200; i++) {
    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &mag, &temp);
    gyroBiasZ += gyro.gyro.z;
    delay(5);
  }
  gyroBiasZ /= 200.0;
  
  lastTime = micros();
  Serial.println("Ready!");
}

void loop() {
  updateYaw();
  
  if (mySerial.available() && !isTurning) {
    String data = mySerial.readStringUntil('\n');
    data.trim();

    if (data == "L") {
      targetYaw = 90;
      turnDirection = -1;
      startTurn();
    } 
    else if (data == "R") {
      targetYaw = -90.0;
      turnDirection = 1;
      startTurn();
    } 
    else if (data.length() > 0) {
      // Normal Steering Mode 
      int angle = data.toInt();
      steeringServo.write(angle);
      driveForward(driveSpeed);
    }
  }

  // 3. Monitor Turn Progress
  if (isTurning) {
    checkTurnProgress();
  }
}

// ================= NAVIGATION FUNCTIONS =================

void updateYaw() {
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &mag, &temp);

  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;

  float gz = (gyro.gyro.z - gyroBiasZ) * 180.0 / PI;
  
  yaw += gz * dt;
}

void startTurn() {
  isTurning = true;
  
  if (turnDirection == 1) {
    steeringServo.write(135); // Hard Right
  } else {
    steeringServo.write(45);  // Hard Left
  }
  
  driveForward(turnSpeed);
  Serial.print("Current Yaw: "); Serial.print(yaw);
  Serial.print(" | Target Yaw: "); Serial.println(targetYaw);
}

void checkTurnProgress() {
  bool turnComplete = false;
  float tolerance = 2.0; // Stop 2 degrees early to account for coasting momentum

  if (turnDirection == 1) { // Right Turn
    if (yaw >= (targetYaw - tolerance)) {
      turnComplete = true;
    }
  } 
  else if (turnDirection == -1) { // Left Turn
    if (yaw <= (targetYaw + tolerance)) {
      turnComplete = true;
    }
  }

  if (turnComplete) {
    stopAll();
    steeringServo.write(90); // Straighten wheels
    isTurning = false;
    
    Serial.print("Turn Complete. Final Yaw: "); Serial.println(yaw);
    
    yaw = 0; 
    Serial.println("Yaw reset to 0 for next operation.");
  }
}

// ================= MOTOR CONTROL =================

void driveForward(int speed) {
  digitalWrite(ain1, HIGH);
  digitalWrite(ain2, LOW);
  ledcWrite(pwma, speed);
}

void stopAll() {
  ledcWrite(pwma, 0);
  digitalWrite(ain1, LOW);
  digitalWrite(ain2, LOW);
}

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h> 
#include <Adafruit_Sensor.h>

//Hardware Setup 
HardwareSerial mySerial(2); 
Servo steeringServo;
Adafruit_MPU6050 mpu; // Changed to MPU6050 object

const int pwma = 13;
const int ain1 = 25;
const int ain2 = 12;

int driveSpeed = 100;
//MPU variables
unsigned long lastTime;
float yaw = 0;
float gyroBiasZ = 0;
bool isTurning = false;
float targetYaw = 0;
int turnDirection = 0; 

void stopAll();
void driveForward(int speed);
void updateYaw();
void startTurn();
void checkTurnProgress();

void setup() {
  delay(1000);
  Serial.begin(115200); 
  
  // RPi Communication
  mySerial.begin(9600, SERIAL_8N1, 27, 14);

  steeringServo.attach(33);
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(pwma, OUTPUT); 

  stopAll();
  steeringServo.write(105); // Center

  // MPU6050 Initialization
  if (!mpu.begin()) { // 0x68
    Serial.println("MPU6050 error!");
    //while (1) delay(10);
  }

  // Setup MPU settings
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Calibration
  int samples = 750;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroBiasZ += g.gyro.z;
    delay(2);
  }
  gyroBiasZ /= (float)samples;
  
  lastTime = micros();
  Serial.println("MPU6050 Ready at 9600 baud!");
}

void loop() {
  updateYaw();

  if (mySerial.available() > 0 && !isTurning) {
    String data = mySerial.readStringUntil('\n');
    data.trim();

    if (data.length() > 0) {
      if (data == "l") {
        targetYaw = 90;
        turnDirection = 1;
        startTurn();
      } 
      else if (data == "r") {
        targetYaw = -90.0;
        turnDirection = -1;
        startTurn();
      } 
      else if(data == "e"){
        stopAll();
      }
      else {
        int incomingAngle = data.toInt();
        int servoAngle = map(incomingAngle, -90, 90, 30, 180);
        Serial.println(servoAngle);
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
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;

  // Convert Rad/s to Deg/s and apply bias
  float gz = (g.gyro.z - gyroBiasZ) * 180.0 / PI;
  
  //Noise filter
  if (abs(gz) < 0.1) gz = 0;

  yaw += gz * dt;
}

void startTurn() {
  isTurning = true;
  driveForward(driveSpeed - 20);
  if (turnDirection == 1) steeringServo.write(30);
  else if (turnDirection == -1) steeringServo.write(180);
}

void checkTurnProgress() {
  bool turnComplete = false;
  if (turnDirection == 1 && yaw >= (targetYaw - 3.0)) turnComplete = true;
  else if (turnDirection == -1 && yaw <= (targetYaw + 3.0)) turnComplete = true;

  if (turnComplete) {
    steeringServo.write(130);
    isTurning = false;
    yaw = 0; 
    turnDirection = 0;
    mySerial.println("h");
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

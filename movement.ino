#include <Arduino.h>
#include <SoftwareSerial.h>

#define RX_PIN 27
#define TX_PIN 14

SoftwareSerial mySerial(RX_PIN, TX_PIN);  // RX, TX

// ===== Pin Definitions =====
const int pwma = 13;
const int ain1 = 22;
const int ain2 = 12;

const int pwmb = 33;
const int bin1 = 26;
const int bin2 = 25;

// ===== PWM Settings =====
const int pwmFreq = 1000;
const int pwmResolution = 8;

int driveSpeed = 150;
int steerSpeed = 200;

int dataInt = 0;

void driveForward();
void steerLeft();
void steerRight();
void steerStraight();
void stopAll();
void controlCar(int value);

void setup() {

  Serial.begin(115200);       
  delay(1000);
  Serial.println("ESP32 Ready (9600)");

  mySerial.begin(9600);      

  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);

  ledcAttach(pwma, pwmFreq, pwmResolution);
  ledcAttach(pwmb, pwmFreq, pwmResolution);

  stopAll();
}

void loop() {

  if (mySerial.available()) {

    String data = mySerial.readStringUntil('\n');
    if(!isDigit(data[0])){
      if(data == "l"){
        driveForward();
        steerLeft();
        delay(2000);
        steerStraight()
        stopAll()
        data = "";
      }
      else if(data == "r"){
        driveForward();
        steerRight();
        delay(2000);
        steerStraight()
        stopAll()
        data = "";
      }
    }
    data.trim();
    dataInt = data.toInt();

    Serial.print("Received: ");
    
    Serial.println(dataInt);

    controlCar(dataInt);
  }
}

// ================= CONTROL =================
void controlCar(int value) {

  if (value == 0) {
    stopAll();
    return;
  }

  value = constrain(value, 0, 180);

  if (value > 85 && value < 95) {
    steerStraight();
  }
  else if (value <= 85) {
    steerRight();
  }
  else {
    steerLeft();
  }

  driveForward();
}

void driveForward() {
  digitalWrite(ain1, HIGH);
  digitalWrite(ain2, LOW);
  ledcWrite(pwma, driveSpeed);
}

void steerLeft() {
  digitalWrite(bin1, HIGH);
  digitalWrite(bin2, LOW);
  ledcWrite(pwmb, steerSpeed);
}

void steerRight() {
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, HIGH);
  ledcWrite(pwmb, steerSpeed);
}

void steerStraight() {
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, LOW);
  ledcWrite(pwmb, 0);
}

void stopAll() {
  ledcWrite(pwma, 0);
  ledcWrite(pwmb, 0);

  digitalWrite(ain1, LOW);
  digitalWrite(ain2, LOW);
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, LOW);
}

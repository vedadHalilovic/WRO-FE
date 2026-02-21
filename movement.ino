#include <dummy.h>
HardwareSerial mySerial(2);  // UART2
//lib za uart HardwareSerial.h
// ===== Pin Definitions =====
const int pwma = 13;   // Drive motor PWM
const int ain1 = 22; //zamijenjeni pnovi radi smjera
const int ain2 = 12;

const int pwmb = 33;  // Steering motor PWM
const int bin1 = 26;
const int bin2 = 25;

// ===== Speed Settings (0–255 za ESP32 standardni analogWrite) =====
int driveSpeed = 200; 
int steerSpeed = 200;

void setup() {
  mySerial.begin(9600, SERIAL_8N1, 27, 14); //16RX 17TX
  pinMode(pwma, OUTPUT);
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);

  pinMode(pwmb, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);

  // Pokretanje Serial komunikacije
  Serial.begin(115200); 
  delay(1000); // Kratka pauza da se Serial stabilizuje
  Serial.println("--- Sistem pokrenut: ESP32 spreman ---");
  
  stopAll();
}


int dataInt;
String data;
void loop() {
  if (mySerial.available()) {
   data = mySerial.readStringUntil('\n');  
}

  dataInt = data.toInt();
  while(dataInt >85 && dataInt<95 ) driveForward();
  while(dataInt <85 && dataInt != 0) {
    steerRight();
    driveForward();
  }
  while(dataInt >95) {
    steerLeft();
    driveForward();
  }
}

// ===== Drive Motor Functions (Motor A) =====
void driveForward() {
  Serial.print("Drive Motor: FORWARD (AIN1: HIGH, AIN2: LOW) | Speed: ");
  Serial.println(driveSpeed);
  digitalWrite(ain1, HIGH);
  digitalWrite(ain2, LOW);
  analogWrite(pwma, driveSpeed);
}

void driveBackward() {
  Serial.print("Drive Motor: BACKWARD (AIN1: LOW, AIN2: HIGH) | Speed: ");
  Serial.println(driveSpeed);
  digitalWrite(ain1, LOW);
  digitalWrite(ain2, HIGH);
  analogWrite(pwma, driveSpeed);
}

// ===== Steering Motor Functions (Motor B) =====
void steerLeft() {
  Serial.print("Steer Motor: LEFT (BIN1: HIGH, BIN2: LOW) | Speed: ");
  Serial.println(steerSpeed);
  digitalWrite(bin1, HIGH);
  digitalWrite(bin2, LOW);
  analogWrite(pwmb, steerSpeed);
}

void steerRight() {
  Serial.print("Steer Motor: RIGHT (BIN1: LOW, BIN2: HIGH) | Speed: ");
  Serial.println(steerSpeed);
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, HIGH);
  analogWrite(pwmb, steerSpeed);
}

void steerStraight() {
  Serial.println("Steer Motor: STRAIGHT (OFF)");
  analogWrite(pwmb, 0);
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, LOW);
}

// ===== Stop Everything =====
void stopAll() {
  Serial.println("!!! STOP ALL MOTORS !!!");
  analogWrite(pwma, 0);
  analogWrite(pwmb, 0);
  digitalWrite(ain1, LOW);
  digitalWrite(ain2, LOW);
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, LOW);
}

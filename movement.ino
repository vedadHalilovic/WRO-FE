#include <dummy.h>

// ===== Pin Definitions =====
const int pwma = 13;   // Drive motor PWM
const int ain1 = 22; //zamijenjeni pnovi radi smjera
const int ain2 = 12;

const int pwmb = 33;  // Steering motor PWM
const int bin1 = 26;
const int bin2 = 25;

// ===== Speed Settings (0–255 za ESP32 standardni analogWrite) =====
// Napomena: Na ESP32 analogWrite radi do 255 osim ako nije drugačije definisano
int driveSpeed = 150; 
int steerSpeed = 150;

void setup() {
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

void loop() {
  // 1️⃣ Pravo naprijed
  Serial.println(">> KRETANJE: Naprijed - Steer: Pravo");
  driveForward();
  steerStraight();
  delay(1000);
  
  stopAll();
  delay(1000);

  // 2️⃣ Naprijed i Lijevo
  Serial.println(">> KRETANJE: Naprijed - Steer: LIJEVO");
  driveForward();
  steerLeft();
  delay(1000);
  
  stopAll();
  delay(1000);

  // 3️⃣ Naprijed i Desno
  Serial.println(">> KRETANJE: Naprijed - Steer: DESNO");
  driveForward();
  steerRight();
  delay(1000);

  stopAll();
  delay(1000);

  // 4️⃣ Nazad pravo
  Serial.println(">> KRETANJE: Nazad - Steer: Pravo");
  driveBackward();
  steerStraight();
  delay(1000);

  // Kraj programa
  stopAll();
  Serial.println("--- Program zavrsen ---");
  
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

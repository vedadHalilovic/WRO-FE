int pwm = 3;   
int in1 = 4;   
int in2 = 2;   

int encoder1 = A0; 
int encoder2 = A2; 

// Variables for Speed Calculation
volatile long pulseCount = 0;
unsigned long prevTime = 0;
float rpm = 0;

// Change this based on your specific 25GA371 gear ratio
// Common values: 11 pulses per motor revolution * gear ratio
const float pulsesPerRev = 11.0 * 30.0; // Example for 1:30 ratio

void setup() {
  Serial.begin(9600);
  
  pinMode(pwm, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  pinMode(encoder1, INPUT);
  Serial.println("Motor Initialized. Starting speed test...");
}

void loop() {
  // 1. Move Motor Forward at half speed
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(pwm, 150); 

  //Calculate RPM every 500ms
  unsigned long currentTime = millis();
  if (currentTime - prevTime >= 500) {
  
    // RPM = (pulses / time) * (60s / pulses_per_rev)
    noInterrupts(); // Stop interrupts to read pulseCount safely
    rpm = (pulseCount / pulsesPerRev) * (60000.0 / (currentTime - prevTime));
    pulseCount = 0; // Reset count for next interval
    interrupts();
    
    prevTime = currentTime;

    // 3. Print Speed to Serial Monitor
    Serial.print("Current Speed: ");
    Serial.print(rpm);
    Serial.println(" RPM");
  }

  // Simulate pulse counting if not using hardware interrupts (Manual polling)
  static bool lastState = LOW;
  bool currentState = digitalRead(encoder1);
  if (currentState == HIGH && lastState == LOW) {
    pulseCount++;
  }
  lastState = currentState;
}
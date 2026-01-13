// Clean Bot Arduino Driver
// Controls: 2x DC Motor (L298N), 1x Relay (Pin 5), 1x Servo (Pin 3), 1x Ultrasonic (HC-SR04)
// Communication: Serial at 57600 baud
// Cleaning system: Servo + Relay sequences handled internally

#include <Servo.h>

// --- 1. PIN DEFINITIONS ---

// Left Motor (L)
const int ENA = 9;   
const int IN1 = 6;   
const int IN2 = 7;   

// Right Motor (R)
const int ENB = 10;  
const int IN3 = 8;   
const int IN4 = 12;  

// Ultrasonic
const int TRIG_PIN = 11;
const int ECHO_PIN = 13;

// Relay
const int RELAY_PIN = 5;

// Servo
const int SERVO_PIN = 3;
Servo cleaningServo;

// --- GLOBALS ---
unsigned long lastCommandTime = 0;
unsigned long lastSensorTime = 0;
String inputBuffer = "";

// Cleaning sequence state
bool cleaningSequenceRunning = false;
int cleaningStep = 0;
unsigned long cleaningStepStartTime = 0;
bool isStartSequence = true; // true = start, false = stop

// --- SETUP ---
void setup() {
  Serial.begin(57600);

  // Motors
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Relay - start OFF (HIGH for Active Low relay)
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // OFF (Active Low)
  
  // Servo - start at minimum position
  cleaningServo.attach(SERVO_PIN);
  cleaningServo.write(0); // MIN position
  
  // Stop motors initially
  setMotor(1, 0);
  setMotor(2, 0);
  
  Serial.println("Arduino Ready");
}

// --- LOOP ---
void loop() {
  unsigned long currentMillis = millis();

  // 1. Read Commands from Serial
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  // 2. Safety Watchdog - Stop motors if no command for 1 second
  if (currentMillis - lastCommandTime > 1000) {
    setMotor(1, 0); 
    setMotor(2, 0);
  }

  // 3. Run cleaning sequence state machine (non-blocking)
  if (cleaningSequenceRunning) {
    runCleaningSequence(currentMillis);
  }

  // 4. Send Ultrasonic Data at 20Hz
  if (currentMillis - lastSensorTime > 50) {
    long dist = readUltrasonic();
    Serial.println(dist);
    lastSensorTime = currentMillis;
  }
}

// --- CLEANING SEQUENCE STATE MACHINE ---
void runCleaningSequence(unsigned long currentMillis) {
  unsigned long elapsed = currentMillis - cleaningStepStartTime;
  
  if (isStartSequence) {
    // START sequence: Servo MIN, then Relay ON(2s)->OFF(1s)->ON(0.5s)->OFF
    switch (cleaningStep) {
      case 0: // Servo MIN
        cleaningServo.write(0);
        cleaningStep = 1;
        cleaningStepStartTime = currentMillis;
        break;
      case 1: // Relay ON for 2 seconds
        digitalWrite(RELAY_PIN, LOW); // ON
        if (elapsed >= 2000) {
          cleaningStep = 2;
          cleaningStepStartTime = currentMillis;
        }
        break;
      case 2: // Relay OFF for 1 second
        digitalWrite(RELAY_PIN, HIGH); // OFF
        if (elapsed >= 1000) {
          cleaningStep = 3;
          cleaningStepStartTime = currentMillis;
        }
        break;
      case 3: // Relay ON for 0.5 seconds
        digitalWrite(RELAY_PIN, LOW); // ON
        if (elapsed >= 500) {
          cleaningStep = 4;
          cleaningStepStartTime = currentMillis;
        }
        break;
      case 4: // Relay OFF - done
        digitalWrite(RELAY_PIN, HIGH); // OFF
        cleaningSequenceRunning = false;
        Serial.println("CLEAN_START_DONE");
        break;
    }
  } else {
    // STOP sequence: Servo MAX, then Relay ON(4s)->OFF
    switch (cleaningStep) {
      case 0: // Servo MAX
        cleaningServo.write(180);
        cleaningStep = 1;
        cleaningStepStartTime = currentMillis;
        break;
      case 1: // Relay ON for 4 seconds
        digitalWrite(RELAY_PIN, LOW); // ON
        if (elapsed >= 4000) {
          cleaningStep = 2;
          cleaningStepStartTime = currentMillis;
        }
        break;
      case 2: // Relay OFF - done
        digitalWrite(RELAY_PIN, HIGH); // OFF
        cleaningSequenceRunning = false;
        Serial.println("CLEAN_STOP_DONE");
        break;
    }
  }
}

// --- COMMAND PROCESSING ---
void processCommand(String cmd) {
  cmd.trim();
  
  // Cleaning commands
  if (cmd == "CLEAN_START") {
    cleaningSequenceRunning = true;
    isStartSequence = true;
    cleaningStep = 0;
    cleaningStepStartTime = millis();
    return;
  }
  if (cmd == "CLEAN_STOP") {
    cleaningSequenceRunning = true;
    isStartSequence = false;
    cleaningStep = 0;
    cleaningStepStartTime = millis();
    return;
  }
  
  // Motor commands: "pwm_left,pwm_right"
  int commaIndex = cmd.indexOf(',');
  if (commaIndex > 0) {
    int pwmLeft = cmd.substring(0, commaIndex).toInt();
    int pwmRight = cmd.substring(commaIndex + 1).toInt();
    
    setMotor(1, pwmLeft);
    setMotor(2, pwmRight);
    lastCommandTime = millis();
  }
}

// --- HELPER FUNCTIONS ---
void setMotor(int motorID, int pwm) {
  int inA, inB, enPin;
  if (motorID == 1) { inA = IN1; inB = IN2; enPin = ENA; } 
  else              { inA = IN3; inB = IN4; enPin = ENB; }

  // Constrain to PWM range
  if (pwm > 255) pwm = 255;
  if (pwm < -255) pwm = -255;

  if (pwm > 0) {
    digitalWrite(inA, HIGH); digitalWrite(inB, LOW); analogWrite(enPin, pwm);
  } else if (pwm < 0) {
    digitalWrite(inA, LOW); digitalWrite(inB, HIGH); analogWrite(enPin, -pwm);
  } else {
    digitalWrite(inA, LOW); digitalWrite(inB, LOW); analogWrite(enPin, 0);
  }
}

long readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); 
  if (duration == 0) return 400; // Time out
  return duration * 0.034 / 2;
}

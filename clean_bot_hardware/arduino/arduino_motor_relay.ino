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

// ==================== AUTONOMOUS WALL-AVOID MODE (NO ENCODERS) ====================
// Triggered via Serial commands:
//   AUTO_START  -> autonomous wall avoidance (drive/backup/turn)
//   AUTO_STOP   -> back to normal PWM control from ROS

// Tuning constants (cm/ms/PWM)
const float WALL_DIST_CM = 5.0;         // keep at least this distance
const int DRIVE_SPEED = 160;            // base forward/back speed
const int TURN_SPEED = 180;             // turn speed
const float LEFT_BOOST = 1.3;           // left motor boost for straight driving
const int BACKUP_TIME_MS = 600;         // reverse time after wall detection
const float TURN_ANGLE_DEG = 30.0;      // nominal turn angle away from wall
const unsigned long STOP_WALL_MS = 500; // pause after wall hit
const unsigned long TURN_MIN_MS = 180;  // minimum turn time
const unsigned long TURN_TIME_MS_90 = 900; // empirical: time to turn ~90deg at TURN_SPEED
const unsigned long POST_TURN_PAUSE_MS = 500;

enum MainMode { MODE_MANUAL = 0, MODE_AUTO = 1 };
MainMode mainMode = MODE_MANUAL;

enum AutoState { AUTO_DRIVE_FWD = 0, AUTO_STOP_WALL = 1, AUTO_DRIVE_BACK = 2, AUTO_TURN_AWAY = 3 };
AutoState autoState = AUTO_DRIVE_FWD;

unsigned long autoStateStartTime = 0;
float lastDistanceCm = 400.0;

unsigned long computeTurnTimeMs(float angleDeg) {
  // Scale from 90deg reference
  float scaled = (TURN_TIME_MS_90 * (angleDeg / 90.0));
  if (scaled < (float)TURN_MIN_MS) scaled = (float)TURN_MIN_MS;
  return (unsigned long)(scaled);
}

void setManualCommandTime() {
  lastCommandTime = millis();
}

int clampPwm(int pwm) {
  if (pwm > 255) return 255;
  if (pwm < -255) return -255;
  return pwm;
}

void driveForwardAuto(int baseSpeed) {
  int leftSpeed = (int)(baseSpeed * LEFT_BOOST);
  if (leftSpeed > 255) leftSpeed = 255;
  setMotor(1, leftSpeed);
  setMotor(2, baseSpeed);
}

void driveBackwardAuto(int baseSpeed) {
  int leftSpeed = (int)(baseSpeed * LEFT_BOOST);
  if (leftSpeed > 255) leftSpeed = 255;
  setMotor(1, -leftSpeed);
  setMotor(2, -baseSpeed);
}

void turnRightAuto(int speed) {
  // Equal power during turns
  setMotor(1, speed);
  setMotor(2, -speed);
}

void stopMotorsAuto() {
  setMotor(1, 0);
  setMotor(2, 0);
}

void startAutoMode() {
  mainMode = MODE_AUTO;
  autoState = AUTO_DRIVE_FWD;
  autoStateStartTime = millis();
  Serial.println("AUTO_MODE_ON");
}

void stopAutoMode() {
  mainMode = MODE_MANUAL;
  stopMotorsAuto();
  Serial.println("AUTO_MODE_OFF");
}

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

  // 2. Safety Watchdog (manual mode only) - Stop motors if no command for 1 second
  if (mainMode == MODE_MANUAL) {
    if (currentMillis - lastCommandTime > 1000) {
      setMotor(1, 0);
      setMotor(2, 0);
    }
  }

  // 2b. Autonomous wall avoidance loop
  if (mainMode == MODE_AUTO) {
    // Read ultrasonic frequently when driving forward
    if (autoState == AUTO_DRIVE_FWD) {
      lastDistanceCm = readUltrasonic();
    }

    switch (autoState) {
      case AUTO_DRIVE_FWD:
        driveForwardAuto(DRIVE_SPEED);
        if (lastDistanceCm > 0 && lastDistanceCm < WALL_DIST_CM) {
          stopMotorsAuto();
          autoState = AUTO_STOP_WALL;
          autoStateStartTime = currentMillis;
        }
        break;

      case AUTO_STOP_WALL:
        stopMotorsAuto();
        if (currentMillis - autoStateStartTime >= STOP_WALL_MS) {
          autoState = AUTO_DRIVE_BACK;
          autoStateStartTime = currentMillis;
        }
        break;

      case AUTO_DRIVE_BACK:
        driveBackwardAuto(DRIVE_SPEED);
        if (currentMillis - autoStateStartTime >= (unsigned long)BACKUP_TIME_MS) {
          stopMotorsAuto();
          autoState = AUTO_TURN_AWAY;
          autoStateStartTime = currentMillis;
        }
        break;

      case AUTO_TURN_AWAY: {
        unsigned long turnMs = computeTurnTimeMs(TURN_ANGLE_DEG);
        // Turn right for turnMs, then pause and go forward
        if (currentMillis - autoStateStartTime < turnMs) {
          turnRightAuto(TURN_SPEED);
        } else if (currentMillis - autoStateStartTime < (turnMs + POST_TURN_PAUSE_MS)) {
          stopMotorsAuto();
        } else {
          autoState = AUTO_DRIVE_FWD;
          autoStateStartTime = currentMillis;
        }
        break;
      }
    }
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

  // Mode commands
  if (cmd == "AUTO_START") {
    startAutoMode();
    return;
  }
  if (cmd == "AUTO_STOP") {
    stopAutoMode();
    return;
  }
  
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

    // Ignore manual PWM commands while in AUTO mode
    if (mainMode == MODE_MANUAL) {
      setMotor(1, clampPwm(pwmLeft));
      setMotor(2, clampPwm(pwmRight));
      setManualCommandTime();
    }
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

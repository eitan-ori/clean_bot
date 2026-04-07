// Clean Bot Arduino Driver
// Controls: 2x DC Motor (Pololu G2 High Power), 1x Brush Servo (Pin 3), 1x Pump PWM (Pin 5), 1x Ultrasonic (HC-SR04)
// Communication: Serial at 57600 baud
// Cleaning system: brush servo + direct pump control via serial commands (MAX/OFF)

#include <Servo.h>

// --- 1. PIN DEFINITIONS ---

// Left Motor (L)
const int PWM_L = 9;   // PWM pin for Left Motor
const int DIR_L = 6;   // Direction pin for Left Motor

// Right Motor (R)
const int PWM_R = 10;  // PWM pin for Right Motor
const int DIR_R = 8;   // Direction pin for Right Motor

// Ultrasonic
const int TRIG_PIN = 11;
const int ECHO_PIN = 13;

// Brush servo (continuous rotation)
const int BRUSH_SERVO_PIN = 3;
Servo brushServo;
const int BRUSH_STOP_US = 1500;
const int BRUSH_RUN_US = 1800;

// Pump PWM (cleaning)
const int PUMP_PWM_PIN = 5;

// --- GLOBALS ---
unsigned long lastCommandTime = 0;
unsigned long lastSensorTime = 0;
String inputBuffer = "";

// --- SETUP ---
void setup() {
  Serial.begin(57600);

  // Motors
  pinMode(DIR_L, OUTPUT); pinMode(PWM_L, OUTPUT);
  pinMode(DIR_R, OUTPUT); pinMode(PWM_R, OUTPUT);

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Brush servo - stop at startup
  brushServo.attach(BRUSH_SERVO_PIN);
  brushServo.writeMicroseconds(BRUSH_STOP_US);

  // Pump PWM - force OFF on startup for safety
  pinMode(PUMP_PWM_PIN, OUTPUT);
  analogWrite(PUMP_PWM_PIN, 0);
  
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

  // 3. Send Ultrasonic Data at 20Hz
  if (currentMillis - lastSensorTime > 50) {
    long dist = readUltrasonic();
    Serial.println(dist);
    lastSensorTime = currentMillis;
  }
}

// --- COMMAND PROCESSING ---
void processCommand(String cmd) {
  cmd.trim();
  
  // Cleaning commands (direct pump PWM)
  // Keep compatibility with existing ROS messages: CLEAN_START/CLEAN_STOP
  if (cmd == "MAX" || cmd == "CLEAN_START") {
    brushServo.writeMicroseconds(BRUSH_RUN_US); // brush ON
    analogWrite(PUMP_PWM_PIN, 255); // 100% pump speed
    Serial.println("ACK: Brush+Pump ON (100%)");
    return;
  }
  if (cmd == "OFF" || cmd == "CLEAN_STOP") {
    brushServo.writeMicroseconds(BRUSH_STOP_US); // brush OFF
    analogWrite(PUMP_PWM_PIN, 0); // pump OFF
    Serial.println("ACK: Brush+Pump OFF");
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
  int dirPin, pwmPin;
  if (motorID == 1) { dirPin = DIR_L; pwmPin = PWM_L; } 
  else              { dirPin = DIR_R; pwmPin = PWM_R; }

  // Constrain to PWM range
  if (pwm > 255) pwm = 255;
  if (pwm < -255) pwm = -255;

  if (pwm > 0) {
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, pwm);
  } else if (pwm < 0) {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, -pwm);
  } else {
    analogWrite(pwmPin, 0);
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

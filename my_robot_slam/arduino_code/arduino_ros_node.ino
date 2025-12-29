// arduino_ros_node.ino
// ROS 2 Interface for Differential Drive Robot
// Compatible with: GB37-131 Motors, L298N Driver, Single Channel Encoders
#include <SoftwareSerial.h>
// --- PINS ---
// Right Motor
#define ENA 5
#define IN1 8
#define IN2 9
#define R_ENC_INT 2

// Left Motor
#define ENB 6
#define IN3 10
#define IN4 11
#define L_ENC_INT 3

// Ultrasonic Sensor
#define TRIG A3
#define ECHO A4

// Fan / Relay
#define RELAY_PIN A0 

// --- CONSTANTS ---
// Max speed in m/s (Estimate - Adjust this to calibrate!)
// If you send 0.5 m/s and robot goes too fast/slow, change this.
const float MAX_SPEED_M_S = 0.6; 
const int PWM_MIN = 60; // Minimum PWM to overcome friction

// --- GLOBALS ---
volatile long left_ticks = 0;
volatile long right_ticks = 0;

// Direction tracking (since single channel encoders can't read direction)
// 1 = Forward, -1 = Backward
volatile int left_dir = 1;
volatile int right_dir = 1;

unsigned long last_cmd_time = 0;
const unsigned long CMD_TIMEOUT = 1000; // Stop if no command for 1s

void setup() {
  Serial.begin(57600); // Match baud rate in Python script

  // Motor Pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  
  // Encoder Pins
  pinMode(L_ENC_INT, INPUT_PULLUP);
  pinMode(R_ENC_INT, INPUT_PULLUP);
  
  // Ultrasonic Pins
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
  // Interrupts
  attachInterrupt(digitalPinToInterrupt(L_ENC_INT), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_INT), rightISR, CHANGE);

  // Fan
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // Off by default
}

void loop() {
  // 1. Read Serial Commands
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    parseCommand(input);
    last_cmd_time = millis();
  }

  // 2. Safety Timeout
  if (millis() - last_cmd_time > CMD_TIMEOUT) {
    stopMotors();
  }

  // 3. Send Odometry + Ultrasonic (20Hz)
  static unsigned long last_odom_time = 0;
  if (millis() - last_odom_time > 50) {
    float distance = getUltrasonicDistance();
    
    // Format: "left_ticks,right_ticks,distance_cm"
    Serial.print(left_ticks);
    Serial.print(",");
    Serial.print(right_ticks);
    Serial.print(",");
    Serial.println(distance);
    
    last_odom_time = millis();
  }
}

// --- INTERRUPTS ---
void leftISR() {
  left_ticks += left_dir;
}

void rightISR() {
  right_ticks += right_dir;
}

// --- ULTRASONIC SENSOR ---
float getUltrasonicDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  // Disable interrupts briefly to get accurate reading
  noInterrupts();
  long duration = pulseIn(ECHO, HIGH, 23500); // Timeout ~4m
  interrupts();
  
  if (duration == 0 || duration < 100) return 400.0; // Max range or no echo
  return duration * 0.0343 / 2.0; // Convert to cm
}

// --- COMMAND PARSING ---
// Expected format: "v_left,v_right" (e.g. "0.25,-0.25")
void parseCommand(String input) {
  int commaIndex = input.indexOf(',');
  if (commaIndex == -1) return;

  String sLeft = input.substring(0, commaIndex);
  String sRight = input.substring(commaIndex + 1);

  float vLeft = sLeft.toFloat();
  float vRight = sRight.toFloat();

  setMotorSpeeds(vLeft, vRight);
}

// --- MOTOR CONTROL ---
void setMotorSpeeds(float vL, float vR) {
  // 1. Determine Direction
  left_dir = (vL >= 0) ? 1 : -1;
  right_dir = (vR >= 0) ? 1 : -1;

  // 2. Convert m/s to PWM (Open Loop)
  int pwmL = velocityToPWM(abs(vL));
  int pwmR = velocityToPWM(abs(vR));

  // 3. Apply "Boost" if needed (from your original code)
  // Note: In a ROS system, it's better to let the navigation stack handle corrections,
  // but if your hardware is very unbalanced, you can keep a factor here.
  // pwmL = pwmL * 1.3; 
  // if (pwmL > 255) pwmL = 255;

  // 4. Drive Motors
  driveLeft(pwmL, left_dir);
  driveRight(pwmR, right_dir);
}

int velocityToPWM(float v) {
  if (v < 0.01) return 0; // Deadzone
  
  // Map 0..MAX_SPEED to PWM_MIN..255
  int pwm = (v / MAX_SPEED_M_S) * 255.0;
  
  if (pwm > 255) pwm = 255;
  if (pwm < PWM_MIN) pwm = PWM_MIN;
  
  return pwm;
}

void driveLeft(int pwm, int dir) {
  if (pwm == 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0);
    return;
  }
  
  if (dir == 1) { // Forward
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  } else { // Backward
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, pwm);
}

void driveRight(int pwm, int dir) {
  if (pwm == 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
    return;
  }

  if (dir == 1) { // Forward
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  } else { // Backward
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, pwm);
}

void stopMotors() {
  driveLeft(0, 1);
  driveRight(0, 1);
}

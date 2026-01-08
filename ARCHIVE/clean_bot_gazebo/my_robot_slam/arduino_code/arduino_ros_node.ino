// Clean Bot Arduino Driver
// Controls: 2x DC Motor (L298N), 2x Encoders, 1x Ultrasonic (HC-SR04)
// Communication: Serial at 57600 baud

// --- 1. PIN DEFINITIONS ---

// Left Motor (L)
const int ENA = 9;   
const int IN1 = 6;   
const int IN2 = 7;   
const int ENC_L_A = 3; 
const int ENC_L_B = 5; 

// Right Motor (R)
const int ENB = 10;  
const int IN3 = 8;   
const int IN4 = 12;  
const int ENC_R_A = 2; 
const int ENC_R_B = 4; 

// Ultrasonic
const int TRIG_PIN = 11;
const int ECHO_PIN = 13;

// --- GLOBALS ---
volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;

unsigned long lastCommandTime = 0;
unsigned long lastSensorTime = 0;

// --- INTERRUPTS ---
void doEncoderLeft() {
  if (digitalRead(ENC_L_B) == HIGH) { encoderLeftCount++; } else { encoderLeftCount--; }
}

void doEncoderRight() {
  if (digitalRead(ENC_R_B) == HIGH) { encoderRightCount++; } else { encoderRightCount--; }
}

// --- SETUP ---
void setup() {
  Serial.begin(57600);

  // Encoders
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), doEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), doEncoderRight, RISING);

  // Motors
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

// --- LOOP ---
void loop() {
  unsigned long currentMillis = millis();

  // 1. Read Command (pwm_left,pwm_right)
  if (Serial.available() > 0) {
    int pwmLeft = Serial.parseInt();
    int pwmRight = Serial.parseInt();
    if (Serial.read() == '\n') {
      setMotor(1, pwmLeft);
      setMotor(2, pwmRight);
      lastCommandTime = currentMillis;
    }
  }

  // 2. Safety Watchdog - Stop if no command for 1 second
  if (currentMillis - lastCommandTime > 1000) {
    setMotor(1, 0); setMotor(2, 0);
  }

  // 3. Send Data (LeftTicks,RightTicks,Distance) at 20Hz
  if (currentMillis - lastSensorTime > 50) {
    long dist = readUltrasonic();
    Serial.print(encoderLeftCount); Serial.print(",");
    Serial.print(encoderRightCount); Serial.print(",");
    Serial.println(dist);
    lastSensorTime = currentMillis;
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

// --- הגדרות פינים ---

// אנקודרים
const int ENC_L_A = 3;
const int ENC_L_B = 5;
const int ENC_R_A = 2;
const int ENC_R_B = 4;

// מנוע שמאל (L298N)
const int IN1 = 12;
const int IN2 = 8;
const int ENA = 10; // חייב להיות פין PWM

// מנוע ימין (L298N)
const int IN3 = 6;
const int IN4 = 7;
const int ENB = 9; // חייב להיות פין PWM

// אולטרסוניק
const int TRIG_PIN = 11;
const int ECHO_PIN = 13;

// --- משתנים גלובליים ---
volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;

unsigned long lastCommandTime = 0; // למנגנון בטיחות
unsigned long lastSensorTime = 0;  // לתזמון שליחת מידע

// --- פסיקות (Interrupts) ---
void doEncoderLeft() {
  if (digitalRead(ENC_L_B) == HIGH) {
    encoderLeftCount++;
  } else {
    encoderLeftCount--;
  }
}

void doEncoderRight() {
  if (digitalRead(ENC_R_B) == HIGH) {
    encoderRightCount++;
  } else {
    encoderRightCount--;
  }
}

void setup() {
  Serial.begin(57600); // תואם לקוד הפייתון

  // הגדרת אנקודרים
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), doEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), doEncoderRight, RISING);

  // הגדרת מנועים
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // הגדרת אולטרסוניק
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. קריאת פקודות מהפאי
  if (Serial.available() > 0) {
    // הפאי שולח: pwm_left,pwm_right
    int pwmLeft = Serial.parseInt();
    int pwmRight = Serial.parseInt();

    // ניקוי שאריות מהבאפר (כמו תו שורה חדשה)
    if (Serial.read() == '\n') {
      // קיבלנו פקודה תקינה
      setMotor(1, pwmLeft);  // מנוע 1 שמאל
      setMotor(2, pwmRight); // מנוע 2 ימין
      lastCommandTime = currentMillis;
    }
  }

  // 2. מנגנון בטיחות (Watchdog): אם הפאי נתקע, לעצור מנועים
  if (currentMillis - lastCommandTime > 1000) {
    setMotor(1, 0);
    setMotor(2, 0);
  }

  // 3. שליחת נתונים לפאי (כל 50 מילישניות)
  if (currentMillis - lastSensorTime > 50) {
    long dist = readUltrasonic();
    
    // הפורמט: LeftTicks,RightTicks,Distance
    Serial.print(encoderLeftCount);
    Serial.print(",");
    Serial.print(encoderRightCount);
    Serial.print(",");
    Serial.println(dist);
    
    lastSensorTime = currentMillis;
  }
}

// --- פונקציית שליטה במנוע ---
// motorID: 1=Left, 2=Right
// pwm: -255 עד 255
void setMotor(int motorID, int pwm) {
  int inA, inB, enPin;

  if (motorID == 1) { // שמאל
    inA = IN1; inB = IN2; enPin = ENA;
  } else { // ימין
    inA = IN3; inB = IN4; enPin = ENB;
  }

  if (pwm > 0) {
    // נסיעה קדימה
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    analogWrite(enPin, pwm);
  } else if (pwm < 0) {
    // נסיעה אחורה
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
    analogWrite(enPin, -pwm); // הופכים לחיובי עבור analogWrite
  } else {
    // עצירה
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    analogWrite(enPin, 0);
  }
}

// --- פונקציית קריאת מרחק ---
long readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // timeout של 30 מילישניות (כ-5 מטר) כדי לא לתקוע את הקוד
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); 
  
  if (duration == 0) return 400; // אם לא חזר הד, נניח מרחק מקסימלי
  return duration * 0.034 / 2; // המרה לס"מ
}
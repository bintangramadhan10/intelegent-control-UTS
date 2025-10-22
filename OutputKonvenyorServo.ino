#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// === Pin motor & encoder ===
const int ENA = 11;
const int IN1 = 9;
const int IN2 = 10;
const int encoderA = 2;
const int encoderB = 3;

// === Servo ===
Servo myServo;
const int servoPin = 6;

// === Variabel encoder ===
volatile long encoderCount = 0;
unsigned long lastTime = 0;
float rpm = 0;
const int pulsesPerRevolution = 20;

// === LCD ===
LiquidCrystal_I2C lcd(0x27, 16, 2);

// === Variabel komunikasi ===
String inputData = "";
int pwmValue = 50;               // default PWM motor
const int pwmDefault = 50;
String pwmLabel = "N/A";
String jarakLabel = "N/A";
String servoLabel = "STANDBY";

// === Servo state ===
int targetAngle = 0;
int currentAngle = 0;

// === Timer komunikasi ===
unsigned long lastDataTime = 0;
unsigned long lastLCD = 0;

// === Deklarasi fungsi ===
void processData(String data);
void updateServo();
void countEncoder();

// === SETUP ===
void setup() {
  Serial.begin(9600);
  Serial.println("=== Sistem Motor + LCD + Servo READY ===");

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA), countEncoder, RISING);

  // Arah motor default
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, pwmValue);

  // Servo
  myServo.attach(servoPin);
  myServo.write(0);  // posisi awal 0 (naik)

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("YOLO+FUZZY READY");
  delay(1000);
  lcd.clear();
}

// === LOOP UTAMA ===
void loop() {
  // --- Baca data serial dari Python ---
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      inputData.trim();
      if (inputData.length() > 0) {
        processData(inputData);
        lastDataTime = millis();  // update waktu terakhir data diterima
      }
      inputData = "";
    } else {
      inputData += c;
    }
  }

  // === PWM FAILSAFE ===
  if (millis() - lastDataTime > 2000) {
    // Tidak ada data > 2 detik, kembali ke default
    pwmValue = pwmDefault;
    analogWrite(ENA, pwmDefault);
  }

  // === Update servo real-time ===
  updateServo();

  // === Hitung RPM setiap 500 ms ===
  if (millis() - lastTime >= 500) {
    noInterrupts();
    long countNow = encoderCount;
    encoderCount = 0;
    interrupts();

    rpm = (countNow / (float)pulsesPerRevolution) * (60.0 / 0.5);
    lastTime = millis();

    Serial.print("PWM=");
    Serial.print(pwmValue);
    Serial.print(", RPM=");
    Serial.println(rpm, 2);
  }
}

// === PARSING DATA DARI PYTHON ===
void processData(String data) {
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);
  if (firstComma == -1 || secondComma == -1) return;

  String pwmStr = data.substring(0, firstComma);
  String jarakStr = data.substring(firstComma + 1, secondComma);
  String servoStr = data.substring(secondComma + 1);

  // === Update nilai dari Python ===
  int pwmBaru = constrain(pwmStr.toInt(), 0, 255);
  pwmValue = pwmBaru;
  jarakLabel = jarakStr;
  servoLabel = servoStr;

  // === Jalankan motor ===
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, pwmValue);

  // === Label PWM ===
  if (pwmValue < 80) pwmLabel = "MIN";
  else if (pwmValue < 200) pwmLabel = "NORM";
  else pwmLabel = "MAX";

  // === Servo kontrol ===
  if (servoLabel == "TURUN") targetAngle = 90;
  else targetAngle = 0;

  // === LCD Update setiap 300ms ===
  if (millis() - lastLCD >= 300) {
    lastLCD = millis();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("PWM:");
    lcd.print(pwmValue);
    lcd.print("(");
    lcd.print(pwmLabel);
    lcd.print(")");
    lcd.setCursor(0, 1);
    lcd.print("Jrk:");
    lcd.print(jarakLabel);
    lcd.print(" ");
    lcd.print(servoLabel);
  }
}

// === SERVO REALTIME ===
void updateServo() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 15) return;
  lastUpdate = millis();

  if (currentAngle < targetAngle) currentAngle++;
  else if (currentAngle > targetAngle) currentAngle--;

  myServo.write(currentAngle);
}

// === ISR ENCODER ===
void countEncoder() {
  encoderCount++;
}

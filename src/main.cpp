#include <Arduino.h>
#include <SoftwareSerial.h>

//  Arduino UNO Alarm System
//  PIR HC-SR501  → D2 (digital)
//  Iduino SE053  → D3 (digital, LOW = vibration)
//  LDR light     → A0 (analog)
//  HM-10 BLE     → D11   (TX), D10 (RX)

// --- Forward declarations ---
void handleSerialCommand();
void handleBTCommand();
void checkSensors();
void resetAlarm();
void printSensorValues();
void sendBluetoothData();
void triggerAlarm(bool motion, bool vibration, bool light);
void recalibrateLight();
void armSystem();
void disarmSystem();
void updateBuzzer();
void initHM10();

// --- Bluetooth (HM-10 BLE module) ---
SoftwareSerial btSerial(10, 11); // RX=D10, TX=D11

// --- Pin definitions ---
const int PIR_PIN       = 2;
const int VIBRATION_PIN = 3;

const int LIGHT_PIN     = A0;
const int BUZZER_PIN    = 8;
const int LED_STATUS    = 13;

// --- Calibration ---
int  lightBaseline  = 0;
int  lightThreshold = 80;
bool pirEnabled     = true;

// --- Alarm state ---
bool          alarmArmed     = false;
bool          alarmTriggered = false;
unsigned long triggerTime    = 0;
unsigned long rearmTime      = 0;
const unsigned long ALARM_DURATION  = 1500;
const unsigned long REARM_DELAY     = 2000;

// --- PIR debounce ---
unsigned long lastPirTime = 0;
const unsigned long PIR_DEBOUNCE = 1000;

// --- Vibration debounce ---
volatile bool vibrationFlag = false;
unsigned long lastVibrationTime = 0;
const unsigned long VIBRATION_DEBOUNCE = 100;

void onVibration() {
  if (millis() - lastVibrationTime > VIBRATION_DEBOUNCE) {
    vibrationFlag = true;
    lastVibrationTime = millis();
  }
}

// --- Buzzer beep pattern (non-blocking) ---
unsigned long lastBeepToggle = 0;
bool          beepState      = false;
const int     BEEP_ON_MS     = 200;
const int     BEEP_OFF_MS    = 100;

// --- Periodic print ---
unsigned long lastPrint  = 0;
const int     PRINT_INTERVAL = 500;


void setup() {
  Serial.begin(9600);
  btSerial.begin(9600);

  pinMode(PIR_PIN,       INPUT);
  pinMode(VIBRATION_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN,    OUTPUT);
  pinMode(LED_STATUS,    OUTPUT);

  attachInterrupt(digitalPinToInterrupt(VIBRATION_PIN), onVibration, FALLING);

  Serial.println("=== Alarm System Initializing ===");
  initHM10();

  Serial.println("Calibrating light sensor...");
  long sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += analogRead(LIGHT_PIN);
    delay(20);
  }
  lightBaseline = sum / 100;
  Serial.print("Light baseline: ");
  Serial.println(lightBaseline);

  Serial.println("Waiting for PIR to stabilize (15s)...");
  for (int i = 15; i > 0; i--) {
    Serial.print(i); Serial.print("... ");
    delay(1000);
  }
  Serial.println("\nReady.");

  armSystem();
}


void loop() {
  handleSerialCommand();
  handleBTCommand();

  if (alarmArmed) {
    checkSensors();
  }

  if (alarmTriggered) {
    updateBuzzer();
    if (millis() - triggerTime > ALARM_DURATION) {
      Serial.println("[ALARM] Auto-reset.");
      resetAlarm();
    }
  }

  if (millis() - lastPrint > PRINT_INTERVAL) {
    printSensorValues();
    sendBluetoothData();
    lastPrint = millis();
  }
}


//  SENSOR LOGIC

void checkSensors() {
  if (millis() < rearmTime) return;

  bool motionDetected    = false;
  bool vibrationDetected = false;
  bool lightChanged      = false;

  if (pirEnabled && digitalRead(PIR_PIN) == HIGH) {
    if (millis() - lastPirTime > PIR_DEBOUNCE) {
      motionDetected = true;
      lastPirTime = millis();
      Serial.println("[PIR] Motion detected!");
    }
  }

  if (vibrationFlag) {
    vibrationFlag = false;
    vibrationDetected = true;
    Serial.println("[VIBRATION] Vibration detected!");
  }

  int lightNow = analogRead(LIGHT_PIN);
  if (abs(lightNow - lightBaseline) > lightThreshold) {
    lightChanged = true;
    Serial.print("[LIGHT] Change detected. Now: ");
    Serial.print(lightNow);
    Serial.print("  Baseline: ");
    Serial.println(lightBaseline);
  }

  if ((motionDetected || vibrationDetected || lightChanged) && !alarmTriggered) {
    triggerAlarm(motionDetected, vibrationDetected, lightChanged);
  }
}


//  ALARM CONTROL

void armSystem() {
  alarmArmed     = true;
  alarmTriggered = false;
  beepState      = false;
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_STATUS, LOW);
  Serial.println("[SYSTEM] Armed. Monitoring...");
}

void disarmSystem() {
  alarmArmed     = false;
  alarmTriggered = false;
  beepState      = false;
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_STATUS, HIGH);
  Serial.println("[SYSTEM] Disarmed.");
}

void triggerAlarm(bool motion, bool vibration, bool light) {
  alarmTriggered  = true;
  triggerTime     = millis();
  beepState       = true;
  lastBeepToggle  = millis();
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(LED_STATUS, HIGH);

  Serial.print("[ALARM] TRIGGERED — Causes: ");
  if (motion)    Serial.print("MOTION ");
  if (vibration) Serial.print("VIBRATION ");
  if (light)     Serial.print("LIGHT ");
  Serial.println();
}

// Nieblokujące pikanie buzzera podczas alarmu
void updateBuzzer() {
  unsigned long now = millis();
  if (beepState) {
    if (now - lastBeepToggle > BEEP_ON_MS) {
      digitalWrite(BUZZER_PIN, LOW);
      beepState      = false;
      lastBeepToggle = now;
    }
  } else {
    if (now - lastBeepToggle > BEEP_OFF_MS) {
      digitalWrite(BUZZER_PIN, HIGH);
      beepState      = true;
      lastBeepToggle = now;
    }
  }
}

void resetAlarm() {
  alarmTriggered = false;
  beepState      = false;
  rearmTime      = millis() + REARM_DELAY;
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_STATUS, LOW);
  Serial.println("[ALARM] Reset. Re-armed.");
}


//  RECALIBRATION

void recalibrateLight() {
  Serial.println("[CALIBRATE] Recalibrating light baseline...");
  long sum = 0;
  for (int i = 0; i < 20; i++) {
    sum += analogRead(LIGHT_PIN);
    delay(10);
  }
  lightBaseline = sum / 20;
  Serial.print("[CALIBRATE] New baseline: ");
  Serial.println(lightBaseline);
}


//  SERIAL COMMANDS (9600 baud)
//  a = arm          d = disarm       r = reset alarm
//  c = recalibrate  + = raise light threshold
//  - = lower light threshold         p = toggle PIR

void handleSerialCommand() {
  if (!Serial.available()) return;
  char cmd = Serial.read();

  switch (cmd) {
    case 'a': armSystem();        break;
    case 'd': disarmSystem();     break;
    case 'r': resetAlarm();       break;
    case 'c': recalibrateLight(); break;
    case '+':
      lightThreshold += 10;
      Serial.print("[CONFIG] Light threshold: "); Serial.println(lightThreshold);
      break;
    case '-':
      lightThreshold = max(10, lightThreshold - 10);
      Serial.print("[CONFIG] Light threshold: "); Serial.println(lightThreshold);
      break;
    case 'p':
      pirEnabled = !pirEnabled;
      Serial.print("[CONFIG] PIR "); Serial.println(pirEnabled ? "ENABLED" : "DISABLED");
      break;
  }
}


//  BLUETOOTH (HM-10 BLE)

// Sends AT command, waits, prints response to Serial
static void atCmd(const char* cmd) {
  btSerial.print(cmd);
  delay(200);
  Serial.print(cmd);
  Serial.print(" -> ");
  unsigned long t = millis();
  while (millis() - t < 300) {
    if (btSerial.available()) Serial.write(btSerial.read());
  }
  Serial.println();
}

// Configures HM-10: peripheral mode, immediate start, no UART noise
void initHM10() {
  Serial.println("[HM-10] Configuring...");
  delay(200);
  atCmd("AT");
  atCmd("AT+ROLE0");
  atCmd("AT+IMME0");
  delay(800);
  while (btSerial.available()) btSerial.read();  // flush garbage after reset
  Serial.println("[HM-10] Ready.");
}

// Sends compact CSV line: light,pir,vibration,armed,alarm
void sendBluetoothData() {
  int lightNow = analogRead(LIGHT_PIN);
  int pirState = digitalRead(PIR_PIN);
  int vibState = (millis() - lastVibrationTime < 1000) ? 1 : 0;

  btSerial.print(lightNow);
  btSerial.print(',');
  btSerial.print(pirState);
  btSerial.print(',');
  btSerial.print(vibState);
  btSerial.print(',');
  btSerial.print(alarmArmed     ? 1 : 0);
  btSerial.print(',');
  btSerial.println(alarmTriggered ? 1 : 0);
}

// Accepts the same single-char commands as the USB serial interface
void handleBTCommand() {
  if (!btSerial.available()) return;
  char cmd = btSerial.read();

  switch (cmd) {
    case 'a': armSystem();        break;
    case 'd': disarmSystem();     break;
    case 'r': resetAlarm();       break;
    case 'c': recalibrateLight(); break;
    case '+':
      lightThreshold += 10;
      break;
    case '-':
      lightThreshold = max(10, lightThreshold - 10);
      break;
    case 'p':
      pirEnabled = !pirEnabled;
      break;
  }
}


//  OUTPUT

void printSensorValues() {
  if (!alarmArmed) return;
  Serial.print("PIR:");  Serial.print(digitalRead(PIR_PIN));
  Serial.print("  VIB:"); Serial.print(!digitalRead(VIBRATION_PIN));
  Serial.print("  LIGHT:"); Serial.print(analogRead(LIGHT_PIN));
  Serial.print(" (base:"); Serial.print(lightBaseline);
  Serial.print(" ±");       Serial.print(lightThreshold);
  Serial.println(")");
}
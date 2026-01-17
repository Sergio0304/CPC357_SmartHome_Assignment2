/*
 * ============================================================================
 *  Smart Door System (Arduino UNO R4 WiFi) - GCP VM + MQTT + Node-RED
 * ============================================================================
 *  Features
 *  - NFC (PN532) authentication to lock/unlock door via servo (continuous servo)
 *  - PIR motion detection:
 *      * When LOCKED + armed  -> trigger alarm (red LED + beep pattern)
 *      * When UNLOCKED        -> turn on smart light (white LED) + fan timer
 *  - DHT11 temperature control for fan relay (hysteresis ON/OFF)
 *  - Ultrasonic (HC-SR04) outside presence detection:
 *      * Confirms presence only if near for >= 3 seconds
 *      * Plays friendly "ding-dong" once (does NOT trigger alarm)
 *  - Remote control from Node-RED via MQTT command topic (lock/unlock)
 *
 *  Cloud Setup
 *  - MQTT Broker: Mosquitto on GCP VM
 *  - Dashboard / logic: Node-RED on same VM
 *
 *  Hardware (Pin Mapping)
 *  - PN532 (SPI): SS=10  (plus SCK/MOSI/MISO per board SPI pins)
 *  - Servo (door): D6
 *  - Buzzer (LOW-trigger): D7
 *  - PIR sensor: D4
 *  - Alarm LED (red): D5
 *  - Smart light LED (white): D3
 *  - DHT11: D2
 *  - Fan relay: D8
 *  - Ultrasonic: TRIG=D9, ECHO=A0
 *
 *  Notes
 *  - Replace WiFi + MQTT config below before uploading.
 *  - For GitHub: do NOT commit real WiFi passwords (use placeholders).
 * ============================================================================
 */

#include <SPI.h>
#include <Adafruit_PN532.h>
#include <Servo.h>
#include <DHT.h>

#include <WiFiS3.h>
#include <PubSubClient.h>

// ============================================================================
// 1) CONFIG SECTION (EDIT THESE)
// ============================================================================

// --- WiFi credentials (DO NOT commit real passwords to GitHub) ---
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// --- MQTT broker (GCP VM public IP or domain) ---
const char* MQTT_SERVER = "YOUR_GCP_VM_IP";
const int   MQTT_PORT   = 1883;

// --- MQTT topics (match Node-RED) ---
const char* TOPIC_EVENT = "smartdoor/event";   // JSON event logs
const char* TOPIC_STATE = "smartdoor/state";   // plain door state strings
const char* TOPIC_CMD   = "smartdoor/cmd";     // command input (lock/unlock)

const char* TOPIC_ALARM = "smartdoor/alarm";   // alarm state
const char* TOPIC_LIGHT = "smartdoor/light";   // light state
const char* TOPIC_FAN   = "smartdoor/fan";     // fan state

// Optional: prevent multiple remote button taps
const unsigned long REMOTE_COOLDOWN_MS = 5000;

// ============================================================================
// 2) GLOBALS: WIFI + MQTT
// ============================================================================
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

unsigned long lastRemoteCmdMs = 0;

// ============================================================================
// 3) NFC + SERVO (DOOR LOCK)
// ============================================================================
#define PN532_SS 10
#define BUZZER_PIN 7
#define SERVO_PIN 6

Adafruit_PN532 nfc(PN532_SS);
Servo doorServo;

// Continuous servo values (tune to your hardware)
const int SERVO_STOP = 91;
const int SERVO_UNLOCK_SPEED = 120;
const int SERVO_LOCK_SPEED   = 60;

// How long to run servo to complete lock/unlock
const int UNLOCK_MOVE_MS = 600;
const int LOCK_MOVE_MS   = 480;

// Allowed NFC UID (change to your own card UID)
const uint8_t AUTH_UID[] = {0x69, 0x22, 0xE9, 0x18};
const uint8_t AUTH_LEN = 4;

bool isDoorLocked = true; // assumed starting state

// ============================================================================
// 4) PIR + LEDs (ALARM / SMART LIGHT)
// ============================================================================
#define PIR_PIN 4
#define LED_RED_PIN 5
#define LED_WHITE_PIN 3

bool alarmArmed = false;
bool alarmTriggered = false;

const unsigned long ALARM_DURATION_MS = 5000;
unsigned long alarmUntilMs = 0;

const unsigned long PIR_REARM_GAP_MS = 2000;
unsigned long lastPirTrigMs = 0;

bool lightOn = false;
const unsigned long LIGHT_DURATION_MS = 8000;
unsigned long lightUntilMs = 0;

// Alarm beep pattern (non-blocking using tone/noTone)
const int ALARM_FREQ_HZ = 2500;
const unsigned long BEEP_ON_MS  = 200;
const unsigned long BEEP_OFF_MS = 150;

bool alarmBeepIsOn = false;
unsigned long nextBeepChangeMs = 0;

// ============================================================================
// 5) FAN (RELAY + DHT11)
// ============================================================================
#define DHT_PIN   2
#define DHT_TYPE  DHT11
DHT dht(DHT_PIN, DHT_TYPE);

#define RELAY_FAN_PIN 8

// Relay behavior: set true if your relay is ACTIVE-LOW
static const bool RELAY_ACTIVE_LOW = false;

// Temperature hysteresis (prevents rapid ON/OFF)
static const float FAN_ON_AT_C  = 30.0f;
static const float FAN_OFF_AT_C = 29.0f;

static const unsigned long DHT_INTERVAL_MS = 2000;
static unsigned long lastDhtMs = 0;

bool fanTempRequest   = false;   // request from temperature
bool fanMotionRequest = false;   // request from motion (timer)
unsigned long fanMotionUntilMs = 0;
static const unsigned long FAN_MOTION_DURATION_MS = 10000;
bool fanOn = false;

// ============================================================================
// 6) ULTRASONIC (HC-SR04) OUTSIDE PRESENCE
// ============================================================================
#define US_TRIG_PIN 9
#define US_ECHO_PIN A0

static const unsigned int OUTSIDE_DISTANCE_CM = 100;   // within 100cm considered "near"
static const unsigned long OUTSIDE_CONFIRM_MS = 3000;  // must stay near >= 3 seconds
static const unsigned long US_SAMPLE_MS = 200;

static unsigned long lastUsMs = 0;
bool outsideNearNow = false;
unsigned long outsideNearSinceMs = 0;
bool outsideConfirmed = false;
bool outsideAlerted = false;

// small filter: require 2 consecutive "near" reads
uint8_t outsideNearStreak = 0;
static const uint8_t OUTSIDE_NEAR_STREAK_MIN = 2;

// publish "clear" only once after leaving near zone
static bool outsideWasNear = false;

// ============================================================================
// 7) BUZZER UTILITIES
// ============================================================================

// LOW-trigger buzzer helpers
void buzzerOn()  { digitalWrite(BUZZER_PIN, LOW); }
void buzzerOff() { digitalWrite(BUZZER_PIN, HIGH); }

// Stop any active tone pattern cleanly (prevents conflicts)
void stopBuzzerTone() {
  noTone(BUZZER_PIN);
  alarmBeepIsOn = false;
  nextBeepChangeMs = 0;
}

// Manual beep (blocking)
void beep(int freq, int durationMs) {
  pinMode(BUZZER_PIN, OUTPUT);
  for (int i = 0; i < durationMs * freq / 1000; i++) {
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(500000 / freq);
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(500000 / freq);
  }
}

void accessGrantedBeep() { stopBuzzerTone(); beep(2000, 200); }
void accessDeniedBeep()  { stopBuzzerTone(); for (int i = 0; i < 3; i++) { beep(1000, 120); delay(120); } }

// Friendly outside tune (blocking but short)
void toneBlocking(int freq, int ms) {
  stopBuzzerTone();
  tone(BUZZER_PIN, freq);
  delay(ms);
  noTone(BUZZER_PIN);
}

// Simple "ding-dong"
void outsidePresenceTune() {
  toneBlocking(988, 120); // B5
  delay(60);
  toneBlocking(784, 180); // G5
}

// ============================================================================
// 8) HELPERS: UID CHECK + SERVO
// ============================================================================
bool uidMatches(const uint8_t *uid, uint8_t uidLen) {
  if (uidLen != AUTH_LEN) return false;
  for (uint8_t i = 0; i < uidLen; i++) {
    if (uid[i] != AUTH_UID[i]) return false;
  }
  return true;
}

void servoStop() { doorServo.write(SERVO_STOP); }

void unlockDoor() {
  doorServo.write(SERVO_UNLOCK_SPEED);
  delay(UNLOCK_MOVE_MS);
  servoStop();
  isDoorLocked = false;
}

void lockDoor() {
  doorServo.write(SERVO_LOCK_SPEED);
  delay(LOCK_MOVE_MS);
  servoStop();
  isDoorLocked = true;
}

// MQTT-friendly delay (keeps mqtt.loop alive)
void mqttDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    mqtt.loop();
    delay(10);
  }
}

// ============================================================================
// 9) MQTT PUBLISH HELPERS
// ============================================================================
void publishEvent(const char* type, const char* result, const char* uidHex) {
  char msg[180];
  snprintf(msg, sizeof(msg),
           "{\"type\":\"%s\",\"result\":\"%s\",\"uid\":\"%s\"}",
           type, result, uidHex);
  mqtt.publish(TOPIC_EVENT, msg);
}

void publishState(const char* state)      { mqtt.publish(TOPIC_STATE, state); }
void publishAlarmState(const char* state) { mqtt.publish(TOPIC_ALARM, state); }
void publishLightState(const char* state) { mqtt.publish(TOPIC_LIGHT, state); }
void publishFanState(const char* state)   { mqtt.publish(TOPIC_FAN, state); }

// ============================================================================
// 10) FAN CONTROL
// ============================================================================
void relayFanWrite(bool on) {
  if (RELAY_ACTIVE_LOW) digitalWrite(RELAY_FAN_PIN, on ? LOW : HIGH);
  else                  digitalWrite(RELAY_FAN_PIN, on ? HIGH : LOW);
}

void applyFanOutput() {
  bool shouldOn = (fanTempRequest || fanMotionRequest);

  if (shouldOn && !fanOn) {
    fanOn = true;
    relayFanWrite(true);
    publishFanState("on");
    publishEvent("fan", "on", "-");
    Serial.println("Fan ON");
  } else if (!shouldOn && fanOn) {
    fanOn = false;
    relayFanWrite(false);
    publishFanState("off");
    publishEvent("fan", "off", "-");
    Serial.println("Fan OFF");
  }
}

void triggerFanFromMotion() {
  fanMotionRequest = true;
  fanMotionUntilMs = millis() + FAN_MOTION_DURATION_MS;
  applyFanOutput();
}

void updateFanMotionTimeout(unsigned long now) {
  if (fanMotionRequest && (long)(now - fanMotionUntilMs) >= 0) {
    fanMotionRequest = false;
    applyFanOutput();
  }
}

void updateDhtAndFanTemp(unsigned long now) {
  if (now - lastDhtMs < DHT_INTERVAL_MS) return;
  lastDhtMs = now;

  float t = dht.readTemperature();
  if (isnan(t)) {
    Serial.println("DHT11 read failed");
    publishEvent("dht", "read_failed", "-");
    return;
  }

  if (!fanTempRequest && t >= FAN_ON_AT_C) {
    fanTempRequest = true;
    applyFanOutput();
  } else if (fanTempRequest && t <= FAN_OFF_AT_C) {
    fanTempRequest = false;
    applyFanOutput();
  }
}

// ============================================================================
// 11) ULTRASONIC OUTSIDE PRESENCE
// ============================================================================
unsigned int readUltrasonicCm() {
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);

  unsigned long duration = pulseIn(US_ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return 9999; // timeout / no echo
  return (unsigned int)(duration / 58UL);
}

void resetOutsidePresenceState() {
  outsideNearNow = false;
  outsideNearSinceMs = 0;
  outsideConfirmed = false;
  outsideAlerted = false;
  outsideNearStreak = 0;
  outsideWasNear = false;
}

void updateOutsidePresence(unsigned long now) {
  // Presence detection only active when door is LOCKED
  if (!isDoorLocked) {
    resetOutsidePresenceState();
    return;
  }

  if (now - lastUsMs < US_SAMPLE_MS) return;
  lastUsMs = now;

  unsigned int cm = readUltrasonicCm();
  bool near = (cm != 9999 && cm <= OUTSIDE_DISTANCE_CM);

  if (near) {
    outsideWasNear = true;
    if (outsideNearStreak < 255) outsideNearStreak++;

    if (outsideNearStreak >= OUTSIDE_NEAR_STREAK_MIN) {
      if (!outsideNearNow) {
        outsideNearNow = true;
        outsideNearSinceMs = now;
        publishEvent("ultrasonic", "near_start", "-");
      }

      // Confirm only if near >= 3 seconds (once)
      if (!outsideConfirmed && (now - outsideNearSinceMs >= OUTSIDE_CONFIRM_MS)) {
        outsideConfirmed = true;
        Serial.println("Outside presence detected (>= 3 seconds)");
        publishEvent("ultrasonic", "outside_present_3s", "-");
      }

      // Friendly notification only once (no alarm)
      if (outsideConfirmed && !outsideAlerted) {
        outsideAlerted = true;
        outsidePresenceTune();
      }
    }
  } else {
    // Publish clear only once after leaving near zone
    if (outsideWasNear) publishEvent("ultrasonic", "clear", "-");
    resetOutsidePresenceState();
  }
}

// ============================================================================
// 12) ALARM + SMART LIGHT
// ============================================================================
void alarmOff() {
  digitalWrite(LED_RED_PIN, LOW);
  stopBuzzerTone();
  alarmTriggered = false;
  publishAlarmState(alarmArmed ? "armed" : "disarmed");
}

void alarmOn() {
  digitalWrite(LED_RED_PIN, HIGH);
  alarmTriggered = true;
  alarmUntilMs = millis() + ALARM_DURATION_MS;
  alarmBeepIsOn = false;
  nextBeepChangeMs = 0;

  publishEvent("alarm", "motion_locked", "-");
  publishAlarmState("alarm");
}

void smartLightOff() {
  digitalWrite(LED_WHITE_PIN, LOW);
  lightOn = false;
  publishLightState("off");
}

void smartLightOn() {
  digitalWrite(LED_WHITE_PIN, HIGH);
  lightOn = true;
  lightUntilMs = millis() + LIGHT_DURATION_MS;

  publishEvent("light", "motion_unlocked", "-");
  publishLightState("on");

  // Also request fan for a limited duration
  triggerFanFromMotion();
}

// ============================================================================
// 13) MQTT CALLBACK + RECONNECT
// ============================================================================
void onMqttMessage(char* topic, uint8_t* payload, unsigned int length) {
  String t = String(topic);

  String cmd;
  cmd.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) cmd += (char)payload[i];
  cmd.trim();
  cmd.toLowerCase();

  if (t != TOPIC_CMD) return;

  unsigned long now = millis();
  if (now - lastRemoteCmdMs < REMOTE_COOLDOWN_MS) {
    Serial.println("Remote command ignored (cooldown).");
    return;
  }
  lastRemoteCmdMs = now;

  if (cmd == "unlock" || cmd == "open") {
    Serial.println("Remote Unlock");
    accessGrantedBeep();
    publishEvent("remote", "unlock", "-");

    publishState("unlocking");
    unlockDoor();
    publishState("unlocked");

    alarmArmed = false;
    alarmOff();

  } else if (cmd == "lock") {
    Serial.println("Remote Lock");
    publishEvent("remote", "lock", "-");

    publishState("locking");
    lockDoor();
    publishState("locked");

    alarmArmed = true;
    publishAlarmState("armed");

    smartLightOff();

    fanMotionRequest = false;
    applyFanOutput();

  } else {
    Serial.println("Unknown command (use: unlock/open, lock)");
  }
}

void mqttReconnect() {
  while (!mqtt.connected()) {
    Serial.print("MQTT connecting... ");

    String clientId = "UNO_R4_SmartDoor_";
    clientId += String((uint32_t)millis(), HEX);

    if (mqtt.connect(clientId.c_str())) {
      Serial.println("OK");
      mqtt.publish(TOPIC_STATE, "online");
      mqtt.subscribe(TOPIC_CMD);

      // Publish current states after reconnect
      publishAlarmState(alarmTriggered ? "alarm" : (alarmArmed ? "armed" : "disarmed"));
      publishLightState(lightOn ? "on" : "off");
      publishFanState(fanOn ? "on" : "off");

    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" retry in 2s");
      delay(2000);
    }
  }
}

// ============================================================================
// 14) SETUP + LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // GPIO init
  pinMode(BUZZER_PIN, OUTPUT);
  buzzerOff();

  pinMode(PIR_PIN, INPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_WHITE_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_WHITE_PIN, LOW);

  pinMode(RELAY_FAN_PIN, OUTPUT);
  relayFanWrite(false);

  dht.begin();

  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);
  digitalWrite(US_TRIG_PIN, LOW);

  doorServo.attach(SERVO_PIN);
  servoStop();

  // Keep alarmArmed consistent with initial lock state
  alarmArmed = isDoorLocked;

  // WiFi connect
  Serial.print("Connecting WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // MQTT
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);

  // NFC
  Serial.println("Starting PN532...");
  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.println("PN532 not found (check wiring / SPI mode / CS pin).");
    while (1) { beep(500, 500); }
  }
  nfc.SAMConfig();
  Serial.println("Ready. Tap a card...");

  // Initial states (use retained states as needed on Node-RED side)
  publishState("ready");
  publishState(isDoorLocked ? "locked" : "unlocked");
  publishAlarmState(alarmArmed ? "armed" : "disarmed");
  publishLightState("off");
  publishFanState("off");
}

void loop() {
  if (!mqtt.connected()) mqttReconnect();
  mqtt.loop();

  unsigned long now = millis();

  // Alarm beep pattern (non-blocking)
  if (alarmTriggered) {
    if (nextBeepChangeMs == 0 || (long)(now - nextBeepChangeMs) >= 0) {
      if (alarmBeepIsOn) {
        noTone(BUZZER_PIN);
        alarmBeepIsOn = false;
        nextBeepChangeMs = now + BEEP_OFF_MS;
      } else {
        tone(BUZZER_PIN, ALARM_FREQ_HZ);
        alarmBeepIsOn = true;
        nextBeepChangeMs = now + BEEP_ON_MS;
      }
    }
  }

  // Sensors / automation updates
  updateDhtAndFanTemp(now);
  updateFanMotionTimeout(now);
  updateOutsidePresence(now);

  // Timers
  if (alarmTriggered && (long)(now - alarmUntilMs) >= 0) alarmOff();
  if (lightOn && (long)(now - lightUntilMs) >= 0) smartLightOff();

  // PIR motion detection
  int pir = digitalRead(PIR_PIN);

  // Motion while LOCKED -> Alarm
  if (alarmArmed && !alarmTriggered) {
    if (pir == HIGH && (now - lastPirTrigMs > PIR_REARM_GAP_MS)) {
      lastPirTrigMs = now;
      Serial.println("Motion detected while LOCKED -> Alarm");
      alarmOn();
    }
  }

  // Motion while UNLOCKED -> Smart light + refresh fan timer
  if (!alarmArmed && !isDoorLocked) {
    if (pir == HIGH && (now - lastPirTrigMs > PIR_REARM_GAP_MS)) {
      lastPirTrigMs = now;

      if (!lightOn) {
        Serial.println("Motion detected while UNLOCKED -> Light + Fan");
        smartLightOn();
      } else {
        // Refresh fan motion timer even if light already ON
        Serial.println("Motion detected (light ON) -> Fan timer refreshed");
        triggerFanFromMotion();
      }
    }
  }

  // NFC read (short timeout = non-blocking feel)
  uint8_t uid[7];
  uint8_t uidLength;
  const uint16_t NFC_TIMEOUT_MS = 50;

  if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, NFC_TIMEOUT_MS)) {
    // Convert UID to hex string for logging
    char uidHex[24] = {0};
    char* p = uidHex;
    for (uint8_t i = 0; i < uidLength; i++) {
      sprintf(p, "%02X", uid[i]);
      p += 2;
      if (i < uidLength - 1) { *p = ':'; p++; }
    }

    Serial.print("NFC UID: ");
    Serial.println(uidHex);

    if (uidMatches(uid, uidLength)) {
      Serial.println("Authorized card");
      accessGrantedBeep();
      publishEvent("nfc", "granted", uidHex);

      if (isDoorLocked) {
        publishState("unlocking");
        unlockDoor();
        publishState("unlocked");

        alarmArmed = false;
        alarmOff();

      } else {
        publishState("locking");
        lockDoor();
        publishState("locked");

        alarmArmed = true;
        publishAlarmState("armed");

        smartLightOff();
        fanMotionRequest = false;
        applyFanOutput();
      }

    } else {
      Serial.println("Access denied");
      accessDeniedBeep();
      publishEvent("nfc", "denied", uidHex);
    }

    // Keep MQTT alive during short post-scan delay
    mqttDelay(1500);
  }
}

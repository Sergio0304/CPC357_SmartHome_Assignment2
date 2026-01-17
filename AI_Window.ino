/*
 * ============================================================================
 *  AI Window + Drying Rack (ESP32-S3) - Edge Impulse Voice + MQTT (GCP VM)
 * ============================================================================
 *  What this firmware does
 *  - Voice control (Edge Impulse keyword spotting):
 *      * "open window"  -> opens window (continuous servo)
 *      * "close window" -> closes window (continuous servo)
 *  - Button control (GPIO3, INPUT_PULLUP):
 *      * Single press  -> toggle window
 *      * Double press  -> toggle drying rack (standard servo)
 *  - Rain protection (DO rain sensor):
 *      * Confirms rain after RAIN_CONFIRM_MS
 *      * Auto-closes window + retracts rack if rain_auto=ON
 *      * Locks out voice actions while raining (if rain_auto=ON)
 *  - Temperature automation (DHT22):
 *      * Opens window when Temp >= TEMP_OPEN_AT (if temp_auto=ON)
 *      * Uses latch + small hysteresis to avoid repeat toggles
 *  - MQTT (Node-RED / Mosquitto on GCP VM):
 *      * Publishes state JSON (retained) + event logs (non-retained)
 *      * Accepts commands: open/close/toggle, voice_on/off, temp_on/off, rain_on/off
 *
 *  Hardware / Pin Map (adjust to your wiring)
 *  - INMP441 Microphone (I2S, LEFT channel):
 *      * BCLK/SCK = GPIO14
 *      * WS/LRCLK = GPIO21
 *      * SD/DOUT  = GPIO47
 *      * L/R pin tied to GND (left channel)
 *  - Button (single/double press): GPIO3 (INPUT_PULLUP)
 *  - Rain Sensor (DO): GPIO4 (INPUT_PULLUP)
 *  - DHT22: GPIO6
 *  - Continuous Servo (window): GPIO38
 *  - Standard Servo (drying rack): GPIO39
 *
 *  Notes for GitHub
 *  - Do NOT commit real Wi-Fi passwords or broker IPs. Use placeholders.
 *  - Edge Impulse model header must be present in the Arduino project:
 *      Audio_Classification_-_Keyword_Spotting_inferencing.h
 * ============================================================================
 */

#define EIDSP_QUANTIZE_FILTERBANK 0

#include <Arduino.h>
#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"

#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include <Audio_Classification_-_Keyword_Spotting_inferencing.h>

// ============================================================================
// 1) CONFIG SECTION (EDIT THESE)
// ============================================================================

// ================== INMP441 I2S PINS ==================
#define I2S_BCLK  14
#define I2S_WS    21
#define I2S_DIN   47
// ======================================================

// ================== USER BUTTON (GPIO3) ==================
#define BUTTON_PIN  3
static const uint32_t BTN_DEBOUNCE_MS = 40;

// Single press = toggle window, double press = toggle rack
static const uint32_t DOUBLE_PRESS_MS = 400;
// ======================================================

// ================== RAIN SENSOR (DO) ==================
#define RAIN_DO_PIN  4
static const bool RAIN_ACTIVE_LOW = true;
static const uint32_t RAIN_CONFIRM_MS = 500;
static const uint32_t RAIN_CLEAR_MS   = 1500;
// ======================================================

// ================== DHT22 ==================
#define DHT_PIN   6
#define DHT_TYPE  DHT22
DHT dht(DHT_PIN, DHT_TYPE);

static const float TEMP_OPEN_AT = 40.0f;
static const uint32_t DHT_INTERVAL_MS = 2000;
// ======================================================

// ================== CONTINUOUS SERVO (WINDOW) ==================
#define SERVO_PIN  38
Servo windowServo;

static const int NEUTRAL_US = 1500;
static const int OPEN_US    = 1900;
static const int CLOSE_US   = 1100;

static const uint32_t OPEN_MOVE_MS  = 200;
static const uint32_t CLOSE_MOVE_MS = 140;
// ======================================================

// ================== DRYING RACK SERVO (NORMAL SERVO) ==================
#define RACK_SERVO_PIN  39
Servo rackServo;

static const int RACK_OUT_ANGLE = 180;
static const int RACK_IN_ANGLE  = 0;
// ======================================================

// ---------------- Voice detection behavior ----------------
// IMPORTANT: threshold must NOT be 0.00, or noise will trigger actions
static const float THRESHOLD = 0.80f;   // adjust 0.70–0.90 depending on your model
static const float MARGIN    = 0.15f;   // winner must beat other label by this margin

static const uint32_t COOLDOWN_MS = 1500;  // minimum time between voice actions
static const uint32_t HOLD_OFF_MS = 800;   // require confidence drop before re-trigger
// ----------------------------------------------------------

// ================== WIFI + MQTT ==================
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

const char* MQTT_HOST = "YOUR_GCP_VM_IP";
const int   MQTT_PORT = 1883;

const char* TOPIC_STATE = "aiwindow/state";
const char* TOPIC_EVENT = "aiwindow/event";
const char* TOPIC_CMD   = "aiwindow/cmd";
// ==================================================

// ============================================================================
// 2) RUNTIME STATE (do not edit unless needed)
// ============================================================================

WiFiClient espClient;
PubSubClient mqtt(espClient);

// Automation toggles (controlled via MQTT)
static bool voice_auto = true;
static bool temp_auto  = true;
static bool rain_auto  = true;

// Publish state periodically (so dashboard always shows something)
static uint32_t last_state_publish_ms = 0;
static const uint32_t STATE_PUBLISH_INTERVAL_MS = 1000;

// Button debounce + press logic
static bool last_btn_level = HIGH;
static uint32_t last_btn_change_ms = 0;
static uint8_t  btn_press_count = 0;
static uint32_t btn_first_press_ms = 0;

// Rain latch timings
static bool rain_latched = false;
static uint32_t rain_since_ms = 0;
static uint32_t dry_since_ms  = 0;

// DHT latch
static uint32_t last_dht_ms = 0;
static bool temp_open_latched = false;

// Servo state (window)
static bool servo_moving = false;
static uint32_t servo_stop_at_ms = 0;
static bool window_is_open = false;

// Rack state
static bool rack_is_out = false;

// Voice trigger control
static uint32_t last_toggle_ms = 0;
static uint32_t last_high_ms = 0;
static bool waiting_for_drop = false;

// Print-once latches (avoid serial spam)
static bool voice_ignored_latched = false;
static bool rain_voice_ignored_latched = false;

// ============================================================================
// 3) EDGE IMPULSE (I2S CAPTURE + INFERENCE BUFFERS)
// ============================================================================

typedef struct {
  int16_t *buffers[2];
  uint8_t  buf_select;
  volatile uint8_t buf_ready;
  uint32_t buf_count;
  uint32_t n_samples;
} inference_t;

static inference_t inference;
static bool debug_nn = false;
static volatile bool record_status = true;

// DMA read buffers
static const uint32_t sample_buffer_size_bytes = 2048;
static int32_t i2s_read_buf32[sample_buffer_size_bytes / 4];
static int16_t sampleBuffer16[sample_buffer_size_bytes / 2];

/**
 * Push samples into Edge Impulse inference buffers.
 * Safety: if the main loop hasn't consumed the current buffer yet, drop samples.
 * This prevents overwriting the inference window under heavy load.
 */
static void audio_inference_callback(uint32_t n_samples_16) {
  if (inference.buf_ready) return;

  for (uint32_t i = 0; i < n_samples_16; i++) {
    inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer16[i];

    if (inference.buf_count >= inference.n_samples) {
      inference.buf_select ^= 1;
      inference.buf_count = 0;
      inference.buf_ready = 1;
    }
  }
}

static int i2s_init(uint32_t sampling_rate) {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = sampling_rate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DIN
  };

  esp_err_t ret = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (ret != ESP_OK) return ret;

  ret = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (ret != ESP_OK) return ret;

  ret = i2s_zero_dma_buffer(I2S_NUM_0);
  return ret;
}

static void capture_samples(void *arg) {
  (void)arg;

  while (record_status) {
    size_t bytes_read = 0;

    esp_err_t err = i2s_read(I2S_NUM_0, (void*)i2s_read_buf32, sample_buffer_size_bytes,
                             &bytes_read, portMAX_DELAY);

    if (err != ESP_OK || bytes_read == 0) continue;

    // Convert I2S 32-bit samples to 16-bit samples
    const int SHIFT = 15;
    const int GAIN  = 8;

    uint32_t samples_32 = bytes_read / 4;
    if (samples_32 > (sample_buffer_size_bytes / 4)) samples_32 = (sample_buffer_size_bytes / 4);

    for (uint32_t i = 0; i < samples_32; i++) {
      int16_t v = (int16_t)(i2s_read_buf32[i] >> SHIFT);

      int32_t boosted = (int32_t)v * GAIN;
      if (boosted > 32767) boosted = 32767;
      if (boosted < -32768) boosted = -32768;

      sampleBuffer16[i] = (int16_t)boosted;
    }

    audio_inference_callback(samples_32);
  }

  vTaskDelete(NULL);
}

static bool microphone_inference_start(uint32_t n_samples) {
  inference.buffers[0] = (int16_t*)malloc(n_samples * sizeof(int16_t));
  if (!inference.buffers[0]) return false;

  inference.buffers[1] = (int16_t*)malloc(n_samples * sizeof(int16_t));
  if (!inference.buffers[1]) {
    free(inference.buffers[0]);
    return false;
  }

  inference.buf_select = 0;
  inference.buf_count = 0;
  inference.n_samples = n_samples;
  inference.buf_ready = 0;

  if (i2s_init(EI_CLASSIFIER_FREQUENCY) != ESP_OK) return false;

  record_status = true;
  xTaskCreate(capture_samples, "CaptureSamples", 1024 * 10, NULL, 10, NULL);
  return true;
}

static bool microphone_inference_record(void) {
  while (inference.buf_ready == 0) delay(1);
  inference.buf_ready = 0;
  return true;
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
  numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
  return 0;
}

// ============================================================================
// 4) MQTT HELPERS
// ============================================================================

static void mqttPublishState(const char* reason) {
  // NOTE: reading DHT here is convenient for dashboard, but adds sensor reads.
  // If you want fewer reads, cache DHT reading in updateDHT().
  float t = dht.readTemperature();
  if (isnan(t)) t = -999;

  char msg[320];
  snprintf(msg, sizeof(msg),
           "{\"open\":%d,\"rack\":%d,\"rain\":%d,\"temp\":%.1f,"
           "\"voice\":%d,\"tempAuto\":%d,\"rainAuto\":%d,"
           "\"reason\":\"%s\"}",
           window_is_open ? 1 : 0,
           rack_is_out ? 1 : 0,
           rain_latched ? 1 : 0,
           t,
           voice_auto ? 1 : 0,
           temp_auto ? 1 : 0,
           rain_auto ? 1 : 0,
           reason);
  mqtt.publish(TOPIC_STATE, msg, true); // retained
}

static void mqttPublishEvent(const char* event, const char* detail = "") {
  char msg[260];
  snprintf(msg, sizeof(msg),
           "{\"event\":\"%s\",\"detail\":\"%s\",\"open\":%d,\"rack\":%d,\"rain\":%d}",
           event, detail,
           window_is_open ? 1 : 0,
           rack_is_out ? 1 : 0,
           rain_latched ? 1 : 0);
  mqtt.publish(TOPIC_EVENT, msg, false); // not retained
}

// ============================================================================
// 5) ACTUATORS: WINDOW SERVO + RACK SERVO
// ============================================================================

static inline void servoStop() {
  windowServo.writeMicroseconds(NEUTRAL_US);
  servo_moving = false;
}

static void startOpenMove(const char* reason) {
  if (servo_moving) {
    Serial.printf("OPEN ignored (servo moving) | reason=%s\n", reason);
    return;
  }

  Serial.printf("OPEN window | reason=%s\n", reason);
  windowServo.writeMicroseconds(OPEN_US);
  servo_moving = true;
  servo_stop_at_ms = millis() + OPEN_MOVE_MS;
  window_is_open = true;

  mqttPublishEvent("action", (String("open: ") + reason).c_str());
  mqttPublishState(reason);
}

static void startCloseMove(const char* reason) {
  if (servo_moving) {
    Serial.printf("CLOSE ignored (servo moving) | reason=%s\n", reason);
    return;
  }

  Serial.printf("CLOSE window | reason=%s\n", reason);
  windowServo.writeMicroseconds(CLOSE_US);
  servo_moving = true;
  servo_stop_at_ms = millis() + CLOSE_MOVE_MS;
  window_is_open = false;

  mqttPublishEvent("action", (String("close: ") + reason).c_str());
  mqttPublishState(reason);
}

static void updateServo() {
  if (servo_moving && (int32_t)(millis() - servo_stop_at_ms) >= 0) {
    servoStop();
    Serial.println("SERVO stop");
    mqttPublishEvent("action", "servo_stop");
    mqttPublishState("servo_stop");
  }
}

static void toggleWindowNow(uint32_t now, const char* reason) {
  if (now - last_toggle_ms < COOLDOWN_MS) {
    Serial.printf("TOGGLE ignored (cooldown) | reason=%s\n", reason);
    return;
  }

  if (!window_is_open) startOpenMove(reason);
  else                 startCloseMove(reason);

  last_toggle_ms = now;
}

static void startRackExtend(const char* reason) {
  Serial.printf("RACK extend | reason=%s\n", reason);
  rackServo.write(RACK_OUT_ANGLE);
  rack_is_out = true;

  mqttPublishEvent("action", (String("rack_extend: ") + reason).c_str());
  mqttPublishState(reason);
}

static void startRackRetract(const char* reason) {
  Serial.printf("RACK retract | reason=%s\n", reason);
  rackServo.write(RACK_IN_ANGLE);
  rack_is_out = false;

  mqttPublishEvent("action", (String("rack_retract: ") + reason).c_str());
  mqttPublishState(reason);
}

// ============================================================================
// 6) NETWORK: WIFI + MQTT CONNECT + COMMANDS
// ============================================================================

static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  (void)topic;

  String cmd;
  cmd.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) cmd += (char)payload[i];
  cmd.trim();
  cmd.toLowerCase();

  Serial.print("MQTT cmd: ");
  Serial.println(cmd);

  // Window commands
  if (cmd == "open") {
    startOpenMove("mqtt:open");
  } else if (cmd == "close") {
    startCloseMove("mqtt:close");
  } else if (cmd == "toggle") {
    toggleWindowNow(millis(), "mqtt:toggle");

  // Automation toggles
  } else if (cmd == "voice_on") {
    voice_auto = true;
    voice_ignored_latched = false;
    mqttPublishEvent("cmd", "voice_on");
    mqttPublishState("voice_on");

  } else if (cmd == "voice_off") {
    voice_auto = false;
    voice_ignored_latched = false;
    mqttPublishEvent("cmd", "voice_off");
    mqttPublishState("voice_off");

  } else if (cmd == "temp_on") {
    temp_auto = true;
    mqttPublishEvent("cmd", "temp_on");
    mqttPublishState("temp_on");

  } else if (cmd == "temp_off") {
    temp_auto = false;
    mqttPublishEvent("cmd", "temp_off");
    mqttPublishState("temp_off");

  } else if (cmd == "rain_on") {
    rain_auto = true;
    rain_voice_ignored_latched = false;
    mqttPublishEvent("cmd", "rain_on");
    mqttPublishState("rain_on");

  } else if (cmd == "rain_off") {
    rain_auto = false;
    rain_voice_ignored_latched = false;
    mqttPublishEvent("cmd", "rain_off");
    mqttPublishState("rain_off");

  // Legacy combined toggle
  } else if (cmd == "auto_on") {
    voice_auto = true;
    temp_auto  = true;
    voice_ignored_latched = false;
    mqttPublishEvent("cmd", "auto_on");
    mqttPublishState("auto_on");

  } else if (cmd == "auto_off") {
    voice_auto = false;
    temp_auto  = false;
    voice_ignored_latched = false;
    mqttPublishEvent("cmd", "auto_off");
    mqttPublishState("auto_off");

  } else {
    mqttPublishEvent("cmd", "unknown");
  }
}

static void wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

static void mqttEnsureConnected() {
  if (mqtt.connected()) return;

  while (!mqtt.connected()) {
    Serial.print("MQTT connecting... ");
    String clientId = "aiwindow-" + String((uint32_t)ESP.getEfuseMac(), HEX);

    if (mqtt.connect(clientId.c_str())) {
      Serial.println("OK");
      mqtt.subscribe(TOPIC_CMD);
      mqttPublishEvent("boot", "mqtt_connected");
      mqttPublishState("boot");
    } else {
      Serial.print("FAIL rc=");
      Serial.print(mqtt.state());
      Serial.println(" retry in 2s");
      delay(2000);
    }
  }
}

// ============================================================================
// 7) INPUTS: BUTTON + RAIN + TEMPERATURE
// ============================================================================

static void updateButton(uint32_t now) {
  bool level = digitalRead(BUTTON_PIN);

  // Edge detection + debounce
  if (level != last_btn_level) {
    last_btn_level = level;
    last_btn_change_ms = now;
    return;
  }
  if (now - last_btn_change_ms < BTN_DEBOUNCE_MS) return;

  static bool pressed_latched = false;

  // Press edge
  if (level == LOW && !pressed_latched) {
    pressed_latched = true;

    btn_press_count++;

    // First press starts the double-press window
    if (btn_press_count == 1) {
      btn_first_press_ms = now;
    }

    // Second press within time -> rack toggle
    else if (btn_press_count == 2 && (now - btn_first_press_ms <= DOUBLE_PRESS_MS)) {
      if (!rack_is_out) startRackExtend("button:double");
      else              startRackRetract("button:double");
      btn_press_count = 0;
    }
  }

  // Release edge
  if (level == HIGH) pressed_latched = false;

  // If no second press arrives -> single press triggers window toggle
  if (btn_press_count == 1 && (now - btn_first_press_ms > DOUBLE_PRESS_MS)) {
    toggleWindowNow(now, "button:single");
    btn_press_count = 0;
  }
}

static bool isRainingNow() {
  bool lvl = digitalRead(RAIN_DO_PIN);
  return RAIN_ACTIVE_LOW ? (lvl == LOW) : (lvl == HIGH);
}

static void updateRain(uint32_t now) {
  bool raining = isRainingNow();

  if (raining) {
    dry_since_ms = 0;
    if (rain_since_ms == 0) rain_since_ms = now;

    // Confirm rain
    if (!rain_latched && (now - rain_since_ms >= RAIN_CONFIRM_MS)) {
      rain_latched = true;
      mqttPublishEvent("rain", "detected");
      mqttPublishState("rain_detected");

      // Auto-protection
      if (rain_auto) {
        if (window_is_open) {
          startCloseMove("rain:auto_close");
          last_toggle_ms = now;
        }
        if (rack_is_out) {
          startRackRetract("rain:auto_retract");
        }
      }
    }
  } else {
    rain_since_ms = 0;
    if (dry_since_ms == 0) dry_since_ms = now;

    // Clear rain only after stable dry period
    if (rain_latched && (now - dry_since_ms >= RAIN_CLEAR_MS)) {
      rain_latched = false;
      mqttPublishEvent("rain", "cleared");
      mqttPublishState("rain_cleared");

      rain_voice_ignored_latched = false;
      temp_open_latched = false;
    }
  }
}

static void updateDHT(uint32_t now) {
  if (now - last_dht_ms < DHT_INTERVAL_MS) return;
  last_dht_ms = now;

  float t = dht.readTemperature();
  if (isnan(t)) {
    mqttPublishEvent("dht", "read_failed");
    return;
  }

  // When raining and auto-protection ON: do not auto-open by temperature
  if (rain_latched && rain_auto) return;
  if (!temp_auto) return;

  // One-shot open when temperature reaches threshold
  if (!temp_open_latched && t >= TEMP_OPEN_AT) {
    temp_open_latched = true;
    if (!window_is_open) {
      startOpenMove("temp>=40C");
      last_toggle_ms = now;
    }
  }

  // Reset latch (hysteresis) to allow future open triggers
  if (temp_open_latched && t < (TEMP_OPEN_AT - 1.0f)) {
    temp_open_latched = false;
  }
}

// ============================================================================
// 8) SETUP + LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nAI Window + Drying Rack (ESP32-S3) starting...");

  // Inputs
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(RAIN_DO_PIN, INPUT_PULLUP);

  // Sensors
  dht.begin();

  // Window servo (continuous)
  windowServo.setPeriodHertz(50);
  windowServo.attach(SERVO_PIN, 500, 2500);
  servoStop();

  // Rack servo (standard)
  rackServo.setPeriodHertz(50);
  rackServo.attach(RACK_SERVO_PIN, 500, 2500);
  rackServo.write(RACK_IN_ANGLE);
  rack_is_out = false;

  // Network
  wifiConnect();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqttEnsureConnected();

  // Edge Impulse
  run_classifier_init();
  if (!microphone_inference_start(EI_CLASSIFIER_SLICE_SIZE)) {
    Serial.println("ERROR: microphone inference start failed");
    while (1) delay(1000);
  }

  last_toggle_ms = millis();
  last_high_ms = 0;

  mqttPublishEvent("boot", "ready");
  mqttPublishState("ready");

  Serial.println("Ready: voice / button single=window / double=rack / rain protects / temp>=40 opens");
}

void loop() {
  uint32_t now = millis();

  // Keep MQTT alive
  mqttEnsureConnected();
  mqtt.loop();

  // Periodic state heartbeat for dashboard
  if (now - last_state_publish_ms >= STATE_PUBLISH_INTERVAL_MS) {
    last_state_publish_ms = now;
    mqttPublishState("periodic");
  }

  // Update actuators + inputs
  updateServo();
  updateButton(now);
  updateRain(now);
  updateDHT(now);

  // ===== Edge Impulse inference step =====
  if (!microphone_inference_record()) return;

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
  signal.get_data = &microphone_audio_signal_get_data;

  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, debug_nn);
  if (r != EI_IMPULSE_OK) return;

  // Voice lockout when raining + rain_auto ON (safety)
  if (rain_latched && rain_auto) {
    if (!rain_voice_ignored_latched) {
      Serial.println("VOICE ignored (raining + rain_auto=ON)");
      rain_voice_ignored_latched = true;
    }
    return;
  } else {
    rain_voice_ignored_latched = false;
  }

  // Voice automation disabled
  if (!voice_auto) {
    if (!voice_ignored_latched) {
      Serial.println("VOICE ignored (voice_auto=OFF)");
      voice_ignored_latched = true;
    }
    return;
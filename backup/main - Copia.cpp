/*=====================================================*/
/* EchoMap - No Principal (prototipo de bancada)         */
/* ESP32 + FreeRTOS                                      */
/* HC-SR04 + IR + potenciometro + botao + RGB + anel 4x  */
/*=====================================================*/

#include <Arduino.h>
#include <math.h>
#include "config.h"

#if ENABLE_MQTT
#include <WiFi.h>
#include <PubSubClient.h>
#endif

/*=====================================================*/
/* Tipos                                                 */
/*=====================================================*/
enum Mode { MODE_RADAR = 0, MODE_SIM360 = 1, MODE_SELFTEST = 2 };
enum EventType { EVT_NEAR_MISS, EVT_CRASH };

struct SensorState {
  float rawCm = -1;
  float filteredCm = -1;
  bool irDetected = false;
  float potNormalized = 0;
};

struct AppState {
  Mode mode = MODE_RADAR;
  float lastThr = DIST_MIN_CM;
  float lastAngle = 0;
  float ringIntensity[RING_SIZE] = {0};
};

struct AppEvent {
  EventType type;
  float value;
  uint32_t t;
};

/*=====================================================*/
/* Globais                                               */
/*=====================================================*/
SensorState sensorState;
AppState appState;
SemaphoreHandle_t stateMutex;
QueueHandle_t eventQueue;
SemaphoreHandle_t buttonSemaphore;
volatile uint32_t crashFlashUntil = 0;

#if ENABLE_MQTT
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
#endif

/*=====================================================*/
/* Botao - ISR                                           */
/*=====================================================*/
void IRAM_ATTR buttonISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(buttonSemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*=====================================================*/
/* HC-SR04                                               */
/*=====================================================*/
float readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long duration = pulseIn(PIN_ECHO, HIGH, 30000UL); /* timeout ~5 m */
  if (duration == 0) return -1.0f;

  float cm = duration * 0.0343f / 2.0f;
  if (cm < 2.0f || cm > 400.0f) return -1.0f;
  return cm;
}

/*=====================================================*/
/* Saida: RGB e anel                                     */
/*=====================================================*/
void setRGB(float r, float g, float b) {
  if (RGB_COMMON_ANODE) {
    r = 1.0f - r;
    g = 1.0f - g;
    b = 1.0f - b;
  }
  analogWrite(PIN_RGB_R, constrain((int)(r * 255), 0, 255));
  analogWrite(PIN_RGB_G, constrain((int)(g * 255), 0, 255));
  analogWrite(PIN_RGB_B, constrain((int)(b * 255), 0, 255));
}

void setRing(const float intensity[RING_SIZE]) {
  for (int i = 0; i < RING_SIZE; i++) {
    analogWrite(RING_PINS[i], constrain((int)(intensity[i] * 255), 0, 255));
  }
}

void bootAnimation() {
  float off[RING_SIZE] = {0};
  setRing(off);
  for (int i = 0; i < RING_SIZE; i++) {
    float frame[RING_SIZE] = {0};
    frame[i] = 1.0f;
    setRing(frame);
    delay(150);
  }
  setRing(off);
}

/*=====================================================*/
/* Logica de cada modo - preenche anel + RGB             */
/*=====================================================*/
void computeRadar(const SensorState &s, float thrCm, uint32_t now,
                   float intensity[RING_SIZE], float &r, float &g, float &b) {
  float closeness = 0.0f;
  if (s.filteredCm >= 0 && s.filteredCm <= thrCm) {
    closeness = constrain((thrCm - s.filteredCm) / thrCm, 0.0f, 1.0f);
  }

  /* verde -> amarelo -> vermelho conforme se aproxima do limiar */
  if (closeness < 0.5f) {
    float t = closeness / 0.5f;
    r = t; g = 1.0f; b = 0.0f;
  } else {
    float t = (closeness - 0.5f) / 0.5f;
    r = 1.0f; g = 1.0f - t; b = 0.0f;
  }

  /* LED 0 = frente, pulsa mais rapido quanto mais perto (solido < ~10% do thr) */
  float freqHz = 0.5f + closeness * 4.5f;
  float phase = fmodf((now / 1000.0f) * freqHz, 1.0f);
  float pulse = (closeness >= 0.9f) ? 1.0f : closeness * (0.5f + 0.5f * sinf(phase * 2.0f * PI));

  for (int i = 0; i < RING_SIZE; i++) intensity[i] = 0.0f;
  intensity[0] = pulse;

  /* vizinhos (direita/esquerda) pulsam mais devagar e mais fraco */
  float neighborPhase = fmodf((now / 1000.0f) * (freqHz * 0.5f), 1.0f);
  float neighborPulse = closeness * 0.35f * (0.5f + 0.5f * sinf(neighborPhase * 2.0f * PI));
  intensity[1] = neighborPulse;
  intensity[RING_SIZE - 1] = neighborPulse;
  /* LED oposto (tras) permanece apagado */
}

void computeSim360(const SensorState &s, uint32_t now, float &angleOut,
                    float intensity[RING_SIZE], float &r, float &g, float &b) {
  float angle = s.potNormalized * 360.0f;
  angleOut = angle;

  float magnitude = 0.0f;
  if (s.filteredCm >= 0) {
    float ref = DIST_MAX_CM * 0.5f;
    magnitude = constrain((ref - s.filteredCm) / ref, 0.0f, 1.0f);
  }

  if (magnitude > 0.5f) {
    r = 1.0f; g = 0.0f; b = 1.0f; /* magenta: obstaculo perto */
  } else {
    r = 0.0f; g = 0.0f; b = 1.0f; /* azul: varredura normal */
  }

  for (int i = 0; i < RING_SIZE; i++) {
    float ledAngle = i * (360.0f / RING_SIZE);
    float diff = fabsf(angle - ledAngle);
    if (diff > 180.0f) diff = 360.0f - diff;
    float falloff = cosf(diff * PI / 180.0f);
    if (falloff < 0) falloff = 0;
    intensity[i] = magnitude * falloff * falloff;
  }
}

void computeSelftest(const SensorState &s, uint32_t now,
                      float intensity[RING_SIZE], float &r, float &g, float &b) {
  if (s.irDetected) {
    for (int i = 0; i < RING_SIZE; i++) intensity[i] = 1.0f;
  } else {
    int step = (now / 400) % RING_SIZE;
    for (int i = 0; i < RING_SIZE; i++) intensity[i] = 0.0f;
    intensity[step] = 1.0f;
  }

  int phase = (now / 600) % 4;
  switch (phase) {
    case 0: r = 1; g = 0; b = 0; break;
    case 1: r = 0; g = 1; b = 0; break;
    case 2: r = 0; g = 0; b = 1; break;
    default: r = 1; g = 1; b = 1; break;
  }
}

void computeIrAlert(uint32_t now, float intensity[RING_SIZE], float &r, float &g, float &b) {
  bool on = ((now / 100) % 2) == 0;
  for (int i = 0; i < RING_SIZE; i++) intensity[i] = on ? 1.0f : 0.0f;
  r = 1.0f; g = 0.0f; b = 0.0f;
}

void computeCrashFlash(float intensity[RING_SIZE], float &r, float &g, float &b) {
  for (int i = 0; i < RING_SIZE; i++) intensity[i] = 1.0f;
  r = 1.0f; g = 1.0f; b = 1.0f;
}

/*=====================================================*/
/* Task 1 - Sensores (HC-SR04, IR, potenciometro)        */
/*=====================================================*/
void TaskSensors(void *pvParameters) {
  bool wasNear = false;

  for (;;) {
    float raw = readDistanceCm();
    int irRaw = digitalRead(PIN_IR);
    bool ir = IR_ACTIVE_LOW ? (irRaw == LOW) : (irRaw == HIGH);
    int potRaw = analogRead(PIN_POT);

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    if (raw < 0) {
      sensorState.rawCm = -1;
      sensorState.filteredCm = -1;
    } else {
      sensorState.rawCm = raw;
      sensorState.filteredCm = (sensorState.filteredCm < 0)
                                    ? raw
                                    : (sensorState.filteredCm * 0.7f + raw * 0.3f);
    }
    sensorState.irDetected = ir;
    sensorState.potNormalized = potRaw / 4095.0f;
    float filtered = sensorState.filteredCm;
    xSemaphoreGive(stateMutex);

    bool isNear = (filtered >= 0 && filtered < NEAR_MISS_CM);
    if (isNear && !wasNear) {
      AppEvent evt{EVT_NEAR_MISS, filtered, millis()};
      xQueueSend(eventQueue, &evt, 0);
    }
    wasNear = isNear;

    vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS));
  }
}

/*=====================================================*/
/* Task 2 - Botao (clique curto = modo, longo = CRASH)   */
/*=====================================================*/
void TaskButton(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(buttonSemaphore, portMAX_DELAY) == pdTRUE) {
      vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
      if (digitalRead(PIN_BUTTON) != LOW) continue; /* ruido */

      uint32_t pressStart = millis();
      while (digitalRead(PIN_BUTTON) == LOW) {
        vTaskDelay(pdMS_TO_TICKS(10));
      }
      uint32_t heldMs = millis() - pressStart;

      if (heldMs >= BUTTON_LONGPRESS_MS) {
        AppEvent evt{EVT_CRASH, 0, millis()};
        xQueueSend(eventQueue, &evt, 0);
        crashFlashUntil = millis() + CRASH_FLASH_MS;
      } else {
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        appState.mode = (Mode)((appState.mode + 1) % 3);
        xSemaphoreGive(stateMutex);
      }
    }
  }
}

/*=====================================================*/
/* Task 3 - Haptica (RGB + anel de LEDs)                 */
/*=====================================================*/
void TaskHaptic(void *pvParameters) {
  bootAnimation();

  for (;;) {
    uint32_t now = millis();

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    SensorState s = sensorState;
    Mode mode = appState.mode;
    xSemaphoreGive(stateMutex);

    float thr = DIST_MIN_CM + s.potNormalized * (DIST_MAX_CM - DIST_MIN_CM);
    float angle = 0.0f;
    float intensity[RING_SIZE];
    float r = 0, g = 0, b = 0;

    if (now < crashFlashUntil) {
      computeCrashFlash(intensity, r, g, b);
    } else if (s.irDetected && mode != MODE_SELFTEST) {
      computeIrAlert(now, intensity, r, g, b);
    } else if (mode == MODE_RADAR) {
      computeRadar(s, thr, now, intensity, r, g, b);
    } else if (mode == MODE_SIM360) {
      computeSim360(s, now, angle, intensity, r, g, b);
    } else {
      computeSelftest(s, now, intensity, r, g, b);
    }

    setRing(intensity);
    setRGB(r, g, b);

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    appState.lastThr = thr;
    appState.lastAngle = angle;
    for (int i = 0; i < RING_SIZE; i++) appState.ringIntensity[i] = intensity[i];
    xSemaphoreGive(stateMutex);

    vTaskDelay(pdMS_TO_TICKS(HAPTIC_PERIOD_MS));
  }
}

/*=====================================================*/
/* Task 4 - Comunicacao (serial JSON + MQTT opcional)    */
/*=====================================================*/
#if ENABLE_MQTT
void conectarWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.println("[WiFi] Iniciando conexao...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  Serial.print("[WiFi] Conectado. IP local: ");
  Serial.println(WiFi.localIP());
}

void conectarMQTT() {
  if (mqttClient.connected()) return;

  mqttClient.setServer(MQTT_BROKER_IP, MQTT_BROKER_PORT);
  String clientId = "echomap-node1-" + String((uint32_t)ESP.getEfuseMac(), HEX);

  if (mqttClient.connect(clientId.c_str())) {
    Serial.println("[MQTT] Conectado ao broker.");
  } else {
    Serial.print("[MQTT] Falha na conexao. Estado: ");
    Serial.println(mqttClient.state());
  }
}
#endif

const char *modeToString(Mode m) {
  switch (m) {
    case MODE_RADAR: return "RADAR";
    case MODE_SIM360: return "SIM360";
    default: return "SELFTEST";
  }
}

void publishTelemetry() {
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  SensorState s = sensorState;
  Mode mode = appState.mode;
  float thr = appState.lastThr;
  float angle = appState.lastAngle;
  float ring[RING_SIZE];
  for (int i = 0; i < RING_SIZE; i++) ring[i] = appState.ringIntensity[i];
  xSemaphoreGive(stateMutex);

  char buf[200];
  int len = snprintf(buf, sizeof(buf),
                      "{\"t\":%lu,\"mode\":\"%s\",\"dist\":%.1f,\"raw\":%.1f,\"thr\":%.0f,"
                      "\"ir\":%d,\"angle\":%.0f,\"pot\":[",
                      (unsigned long)millis(), modeToString(mode), s.filteredCm, s.rawCm, thr,
                      s.irDetected ? 1 : 0, angle);

  for (int i = 0; i < RING_SIZE && len < (int)sizeof(buf) - 16; i++) {
    len += snprintf(buf + len, sizeof(buf) - len, "%s%.2f", (i == 0 ? "" : ","), ring[i]);
  }
  len += snprintf(buf + len, sizeof(buf) - len, "]}");

  Serial.println(buf);

#if ENABLE_MQTT
  if (mqttClient.connected()) {
    mqttClient.publish(MQTT_TOPIC_TELEMETRY, buf);
  }
#endif
}

void emitEvent(const AppEvent &evt) {
  char buf[96];
  if (evt.type == EVT_NEAR_MISS) {
    snprintf(buf, sizeof(buf), "{\"event\":\"NEAR_MISS\",\"value\":%.1f,\"t\":%lu}", evt.value,
              (unsigned long)evt.t);
  } else {
    snprintf(buf, sizeof(buf), "{\"event\":\"CRASH\",\"t\":%lu}", (unsigned long)evt.t);
  }

  Serial.println(buf);

#if ENABLE_MQTT
  if (mqttClient.connected()) {
    mqttClient.publish(MQTT_TOPIC_EVENTS, buf);
  }
#endif
}

void TaskComms(void *pvParameters) {
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
#if ENABLE_MQTT
    conectarWiFi();
    conectarMQTT();
    mqttClient.loop();
#endif

    AppEvent evt;
    while (xQueueReceive(eventQueue, &evt, 0) == pdTRUE) {
      emitEvent(evt);
    }

    publishTelemetry();

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
  }
}

/*=====================================================*/
/* Setup                                                 */
/*=====================================================*/
void setup() {
  Serial.begin(115200);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_IR, INPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_RGB_R, OUTPUT);
  pinMode(PIN_RGB_G, OUTPUT);
  pinMode(PIN_RGB_B, OUTPUT);
  for (int i = 0; i < RING_SIZE; i++) pinMode(RING_PINS[i], OUTPUT);

  stateMutex = xSemaphoreCreateMutex();
  eventQueue = xQueueCreate(10, sizeof(AppEvent));
  buttonSemaphore = xSemaphoreCreateBinary();

  if (stateMutex == NULL || eventQueue == NULL || buttonSemaphore == NULL) {
    Serial.println("[ERRO] Falha ao criar primitivos do FreeRTOS.");
    while (true) delay(1000);
  }

  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), buttonISR, FALLING);

  /* Core 1: leitura de sensores e atuacao (timing critico, longe do Wi-Fi) */
  xTaskCreatePinnedToCore(TaskSensors, "TaskSensors", 3072, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskHaptic, "TaskHaptic", 3072, NULL, 1, NULL, 1);

  /* Core 0: botao e comunicacao (serial/MQTT) */
  xTaskCreatePinnedToCore(TaskButton, "TaskButton", 2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskComms, "TaskComms", 4096, NULL, 1, NULL, 0);

  Serial.println("[APP] EchoMap no principal iniciado (anel com 4 LEDs).");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

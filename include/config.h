#pragma once

/*=====================================================*/
/* EchoMap - No Principal (bancada)                    */
/* Configuracao de pinos, rede e calibracao             */
/*=====================================================*/

/* ----- Rede / MQTT ----- */
#define ENABLE_MQTT 0   /* 1 = conecta Wi-Fi e publica no broker Mosquitto */

#define WIFI_SSID "SUA_REDE_WIFI"
#define WIFI_PASSWORD "SUA_SENHA_WIFI"

#define MQTT_BROKER_IP "192.168.1.100"
#define MQTT_BROKER_PORT 1883
#define MQTT_TOPIC_TELEMETRY "echomap/node1/telemetry"
#define MQTT_TOPIC_EVENTS "echomap/node1/events"

/* ----- HC-SR04 esquerdo (sensor original) ----- */
#define PIN_TRIG_L 18
#define PIN_ECHO_L 19 /* via divisor de tensao: ECHO -> 1k -> GPIO19 -> 2k -> GND */

/* ----- HC-SR04 direito (novo sensor) ----- */
#define PIN_TRIG_R 25
#define PIN_ECHO_R 33 /* via divisor de tensao: ECHO -> 1k -> GPIO33 -> 2k -> GND */

/* ----- Potenciometro (calibracao) ----- */
#define PIN_POT 35

/* ----- Botao (troca de modo / CRASH manual) ----- */
#define PIN_BUTTON 4
#define BUTTON_DEBOUNCE_MS 50
#define BUTTON_LONGPRESS_MS 1500

/* ----- LED RGB (status) ----- */
#define PIN_RGB_R 21
#define PIN_RGB_G 22
#define PIN_RGB_B 23
#define RGB_COMMON_ANODE 0 /* 1 = comum no 3V3 (anodo comum) */

/* ----- Buzzers passivos (1 por sensor ultrassonico) -----
 * Canais LEDC dedicados (nao usar tone()/noTone(): nesse core elas
 * compartilham um unico canal global e um buzzer rouba o som do outro). */
#define PIN_BUZZER_L 13
#define PIN_BUZZER_R 14
#define BUZZER_CHANNEL_L 0
#define BUZZER_CHANNEL_R 2
#define BUZZER_FREQ_HZ 2000 /* tom fixo; o que varia com a distancia e o volume */

/* Distancias pensadas para pessoas andando na rua: alguem passando a ~1,2 m ja
 * entra no alcance (volume baixo) e o volume sobe ate o maximo a 30 cm. */
#define BUZZER_START_CM 120.0f  /* comeca a apitar, no volume minimo */
#define BUZZER_FULL_CM 30.0f    /* daqui para perto: volume maximo */
#define BUZZER_MIN_VOLUME 0.15f /* volume (0..1) logo ao entrar no alcance */
#define BUZZER_HYST_CM 10.0f    /* so desliga ao passar de START + HYST (evita liga/desliga na borda) */
#define BUZZER_CONFIRM_SAMPLES 2 /* leituras seguidas para ligar/desligar (filtra eco espurio) */

/* ----- Calibracao / limiares ----- */
#define DIST_MIN_CM 20.0f
#define DIST_MAX_CM 200.0f
#define NEAR_MISS_CM 50.0f /* alguem passou a menos de meio metro */

/* ----- Periodos das tasks (ms) ----- */
#define SENSOR_PERIOD_MS 60
#define SENSOR_GAP_MS 30 /* intervalo entre os pings L/R para o eco de um nao chegar no outro */
#define STATUS_PERIOD_MS 20
#define TELEMETRY_PERIOD_MS 200
#define CRASH_FLASH_MS 300

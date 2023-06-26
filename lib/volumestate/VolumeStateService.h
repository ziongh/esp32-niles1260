#ifndef VolumeStateService_h
#define VolumeStateService_h

#include <VolumeMqttSettingsService.h>

#include <HttpEndpoint.h>
#include <MqttPubSub.h>
#include <WebSocketTxRx.h>

#ifdef ESP32
#define SERVO1_PIN  5
#define SERVO2_PIN 21
#define SERVO3_PIN 22
#define SPEED_IN_PIN A0 // 36/VP
#define MODE_ANALOG_INPUT_PIN A3 // 39
#elif defined(__AVR__) // Default as for ATmega328 like on Uno, Nano etc.
#define SERVO1_PIN 9 // For ATmega328 pins 9 + 10 are connected to timer 2 and can therefore be used also by the Lightweight Servo library
#define SERVO2_PIN 10
#define SERVO3_PIN 11
#define SPEED_IN_PIN A0
#define MODE_ANALOG_INPUT_PIN A1
#elif defined(ESP8266)
#define SERVO1_PIN  14 // D5
#define SERVO2_PIN  12 // D6
#define SERVO3_PIN  13 // D7
#define SPEED_IN_PIN A0
#endif

#define DEFAULT_VOLUME_STATE 0

// Note that the built-in VOLUME is on when the pin is low on most NodeMCU boards.
// This is because the anode is tied to VCC and the cathode to the GPIO 4 (Arduino pin 2).
#ifdef ESP32
#define VOLUME_MAX_ROTATION 270.0
#define VOLUME_MIN 0
#elif defined(ESP8266)
#define VOLUME_MAX_ROTATION 270
#define VOLUME_MIN 0
#endif

#define Volume_SETTINGS_ENDPOINT_PATH "/rest/volumeState"
#define Volume_SETTINGS_SOCKET_PATH "/ws/volumeState"

class VolumeState {
 public:
  int volumeSala;
  int volumeVaranda;
  int volumeCozinha;
  int volumeCinema;

  static void read(VolumeState& settings, JsonObject& root) {
    root["volumeSala"] = settings.volumeSala;
    root["volumeVaranda"] = settings.volumeVaranda;
    root["volumeCozinha"] = settings.volumeCozinha;
    root["volumeCinema"] = settings.volumeCinema;
  }

  static StateUpdateResult update(JsonObject& root, VolumeState& VolumeState) {
    bool updated = false;

    int newStateSala = root["volumeSala"] | DEFAULT_VOLUME_STATE;
    if (VolumeState.volumeSala != newStateSala) {
      VolumeState.volumeSala = newStateSala;
      updated = true;
    }

    int newStateVaranda = root["volumeVaranda"] | DEFAULT_VOLUME_STATE;
    if (VolumeState.volumeVaranda != newStateVaranda) {
      VolumeState.volumeVaranda = newStateVaranda;
      updated = true;
    }

    int newStateCozinha = root["volumeCozinha"] | DEFAULT_VOLUME_STATE;
    if (VolumeState.volumeCozinha != newStateCozinha) {
      VolumeState.volumeCozinha = newStateCozinha;
      updated = true;
    }

    int newStateCinema = root["volumeCinema"] | DEFAULT_VOLUME_STATE;
    if (VolumeState.volumeCinema != newStateCinema) {
      VolumeState.volumeCinema = newStateCinema;
      updated = true;
    }

    if(updated){
      return StateUpdateResult::CHANGED;
    }

    return StateUpdateResult::UNCHANGED;
  }

  static void haRead(VolumeState& settings, JsonObject& root) {
    root["volumeSala"] = settings.volumeSala;
    root["volumeVaranda"] = settings.volumeVaranda;
    root["volumeCozinha"] = settings.volumeCozinha;
    root["volumeCinema"] = settings.volumeCinema;
  }

  static StateUpdateResult haUpdate(JsonObject& root, VolumeState& VolumeState) {
    Serial.print(F("HA Changed State (volumeSala, volumeVaranda, volumeCozinha, volumeCinema): "));

    int volumeSala = root["volumeSala"];
    int volumeVaranda = root["volumeVaranda"];
    int volumeCozinha = root["volumeCozinha"];
    int volumeCinema = root["volumeCinema"];

    Serial.print(volumeSala);
    Serial.print(volumeVaranda);
    Serial.print(volumeCozinha);
    Serial.print(volumeCinema);
    Serial.println();

    Serial.println(root);

    bool updated = false;

    if (VolumeState.volumeSala != volumeSala) {
      VolumeState.volumeSala = volumeSala;
      updated = true;
    }

    if (VolumeState.volumeVaranda != volumeVaranda) {
      VolumeState.volumeVaranda = volumeVaranda;
      updated = true;
    }

    if (VolumeState.volumeCozinha != volumeCozinha) {
      VolumeState.volumeCozinha = volumeCozinha;
      updated = true;
    }

    if (VolumeState.volumeCinema != volumeCinema) {
      VolumeState.volumeCinema = volumeCinema;
      updated = true;
    }

    if(updated){
      return StateUpdateResult::CHANGED;
    }
    return StateUpdateResult::UNCHANGED;
  }
};

class VolumeStateService : public StatefulService<VolumeState> {
 public:
  VolumeStateService(AsyncWebServer* server,
                    SecurityManager* securityManager,
                    AsyncMqttClient* mqttClient,
                    VolumeMqttSettingsService* VolumeMqttSettingsService);
  void begin();

 private:
  HttpEndpoint<VolumeState> _httpEndpoint;
  MqttPubSub<VolumeState> _mqttPubSub;
  WebSocketTxRx<VolumeState> _webSocket;
  AsyncMqttClient* _mqttClient;
  VolumeMqttSettingsService* _VolumeMqttSettingsService;

  void registerConfig();
  void onConfigUpdated();
};

#endif

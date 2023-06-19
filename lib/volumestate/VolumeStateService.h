#ifndef VolumeStateService_h
#define VolumeStateService_h

#include <VolumeMqttSettingsService.h>

#include <HttpEndpoint.h>
#include <MqttPubSub.h>
#include <WebSocketTxRx.h>

#ifdef ESP32
#define SERVO1_PIN  5
#define SERVO2_PIN 18
#define SERVO3_PIN 19
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
#define OFF_STATE "OFF"
#define ON_STATE "ON"

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
  int volume;

  static void read(VolumeState& settings, JsonObject& root) {
    root["volume"] = settings.volume;
  }

  static StateUpdateResult update(JsonObject& root, VolumeState& VolumeState) {
    boolean newState = root["volume"] | DEFAULT_VOLUME_STATE;
    if (VolumeState.volume != newState) {
      VolumeState.volume = newState;
      return StateUpdateResult::CHANGED;
    }
    return StateUpdateResult::UNCHANGED;
  }

  static void haRead(VolumeState& settings, JsonObject& root) {
    root["state"] = settings.volume ? ON_STATE : OFF_STATE;
  }

  static StateUpdateResult haUpdate(JsonObject& root, VolumeState& VolumeState) {
    String state = root["state"];
    // parse new led state 
    boolean newState = false;
    if (state.equals(ON_STATE)) {
      newState = true;
    } else if (!state.equals(OFF_STATE)) {
      return StateUpdateResult::ERROR;
    }
    // change the new state, if required
    if (VolumeState.volume != newState) {
      VolumeState.volume = newState;
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

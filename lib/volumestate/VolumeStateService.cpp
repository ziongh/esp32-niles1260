#include <VolumeStateService.h>

// Must specify this before the include of "ServoEasing.hpp"
#define USE_PCA9685_SERVO_EXPANDER    // Activating this enables the use of the PCA9685 I2C expander chip/board.
//#define USE_SOFT_I2C_MASTER           // Saves 1756 bytes program memory and 218 bytes RAM compared with Arduino Wire
//#define USE_SERVO_LIB                 // If USE_PCA9685_SERVO_EXPANDER is defined, Activating this enables force additional using of regular servo library.
//#define USE_LEIGHTWEIGHT_SERVO_LIB    // Makes the servo pulse generating immune to other libraries blocking interrupts for a longer time like SoftwareSerial, Adafruit_NeoPixel and DmxSimple.
//#define PROVIDE_ONLY_LINEAR_MOVEMENT  // Activating this disables all but LINEAR movement. Saves up to 1540 bytes program memory.
#define DISABLE_COMPLEX_FUNCTIONS     // Activating this disables the SINE, CIRCULAR, BACK, ELASTIC, BOUNCE and PRECISION easings. Saves up to 1850 bytes program memory.
#define MAX_EASING_SERVOS 1
//#define DISABLE_MICROS_AS_DEGREE_PARAMETER // Activating this disables microsecond values as (target angle) parameter. Saves 128 bytes program memory.
//#define DISABLE_MIN_AND_MAX_CONSTRAINTS    // Activating this disables constraints. Saves 4 bytes RAM per servo but strangely enough no program memory.
//#define DISABLE_PAUSE_RESUME               // Activating this disables pause and resume functions. Saves 5 bytes RAM per servo.
//#define DEBUG                              // Activating this enables generate lots of lovely debug output for this library.

//#define PRINT_FOR_SERIAL_PLOTTER           // Activating this enables generate the Arduino plotter output from ServoEasing.hpp.
/*
 * Specify which easings types should be available.
 * If no easing is defined, all easings are active.
 * This must be done before the #include "ServoEasing.hpp"
 */
//#define ENABLE_EASE_QUADRATIC
#define ENABLE_EASE_CUBIC
//#define ENABLE_EASE_QUARTIC
//#define ENABLE_EASE_SINE
//#define ENABLE_EASE_CIRCULAR
//#define ENABLE_EASE_BACK
//#define ENABLE_EASE_ELASTIC
//#define ENABLE_EASE_BOUNCE
//#define ENABLE_EASE_PRECISION
//#define ENABLE_EASE_USER
#include "ServoEasing.hpp"

#if defined(USE_PCA9685_SERVO_EXPANDER)
ServoEasing Servo1(PCA9685_DEFAULT_ADDRESS); // If you use more than one PCA9685 you probably must modify MAX_EASING_SERVOS
#else
ServoEasing Servo1;
#endif
#define START_DEGREE_VALUE  0 // The degree value written to the servo at time of attach.

VolumeStateService::VolumeStateService(AsyncWebServer* server,
                                     SecurityManager* securityManager,
                                     AsyncMqttClient* mqttClient,
                                     VolumeMqttSettingsService* VolumeMqttSettingsService) :
    _httpEndpoint(VolumeState::read,
                  VolumeState::update,
                  this,
                  server,
                  Volume_SETTINGS_ENDPOINT_PATH,
                  securityManager,
                  AuthenticationPredicates::IS_AUTHENTICATED),
    _mqttPubSub(VolumeState::haRead, VolumeState::haUpdate, this, mqttClient),
    _webSocket(VolumeState::read,
               VolumeState::update,
               this,
               server,
               Volume_SETTINGS_SOCKET_PATH,
               securityManager,
               AuthenticationPredicates::IS_AUTHENTICATED),
    _mqttClient(mqttClient),
    _VolumeMqttSettingsService(VolumeMqttSettingsService) {
  // configure led to be output
  pinMode(SERVO1_PIN, OUTPUT);

  /********************************************************
   * Attach servo to pin and set servos to start position.
   * This is the position where the movement starts.
   *******************************************************/
  #if defined(USE_PCA9685_SERVO_EXPANDER)
    if (Servo1.InitializeAndCheckI2CConnection(&Serial)) {
        while (true) {
          delay(100);
        }
    }
  #endif

  #if !defined(PRINT_FOR_SERIAL_PLOTTER)
    #if defined(USE_PCA9685_SERVO_EXPANDER)
      #undef SERVO1_PIN
      #define SERVO1_PIN  0 // we use first port of expander
      Serial.println(F("Attach servo to port 0 of PCA9685 expander"));
    #else
      Serial.println(F("Attach servo at pin " STR(SERVO1_PIN)));
    #endif
  #endif
  if (Servo1.attach(SERVO1_PIN, START_DEGREE_VALUE) == INVALID_SERVO) {
      Serial.println(F("Error attaching servo"));
      while (true) {
          delay(100);
      }
  }

  // Wait for servo to reach start position.
  delay(500);
  #if defined(PRINT_FOR_SERIAL_PLOTTER)
    // Legend for Arduino Serial plotter
    Serial.println(); // end of line of attach values
    Serial.println("OneServo[us]_Linear->Cubic->Linear");
  #endif

  // configure MQTT callback
  _mqttClient->onConnect(std::bind(&VolumeStateService::registerConfig, this));

  // configure update handler for when the Volume settings change
  _VolumeMqttSettingsService->addUpdateHandler([&](const String& originId) { registerConfig(); }, false);

  // configure settings service update handler to update LED state
  addUpdateHandler([&](const String& originId) { onConfigUpdated(); }, false);
}

void VolumeStateService::begin() {
  _state.volume = DEFAULT_VOLUME_STATE;
  onConfigUpdated();
}

void VolumeStateService::onConfigUpdated() {
  Servo1.startEaseToD((float) (_state.volume * (VOLUME_MAX_ROTATION / 360.0)), (uint_fast16_t) 300, DO_NOT_START_UPDATE_BY_INTERRUPT); // rotate to position using speed
   do {
      // First do the delay, then check for update, since we are probably called directly after start and there is nothing to move yet
      delay(REFRESH_INTERVAL_MILLIS); // 20 ms
  #if defined(PRINT_FOR_SERIAL_PLOTTER)
    } while (!updateAllServos()); // this outputs a value plus a newline, whilst Servo1.update() would not output the newline
  #else
    } while (!Servo1.update());
  #endif
}

void VolumeStateService::registerConfig() {
  if (!_mqttClient->connected()) {
    return;
  }
  String configTopic;
  String subTopic;
  String pubTopic;

  DynamicJsonDocument doc(256);
  _VolumeMqttSettingsService->read([&](VolumeMqttSettings& settings) {
    configTopic = settings.mqttPath + "/config";
    subTopic = settings.mqttPath + "/set";
    pubTopic = settings.mqttPath + "/state";
    doc["~"] = settings.mqttPath;
    doc["name"] = settings.name;
    doc["unique_id"] = settings.uniqueId;
  });
  doc["cmd_t"] = "~/set";
  doc["stat_t"] = "~/state";
  doc["schema"] = "json";
  doc["brightness"] = false;

  String payload;
  serializeJson(doc, payload);
  _mqttClient->publish(configTopic.c_str(), 0, false, payload.c_str());

  _mqttPubSub.configureTopics(pubTopic, subTopic);
}

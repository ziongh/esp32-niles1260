#include <VolumeStateService.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


// our servo # counter
uint8_t servonum = 0;

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

  // Wire.begin(0x40, 22, 21, ); // SDA, SCL

  // configure MQTT callback
  _mqttClient->onConnect(std::bind(&VolumeStateService::registerConfig, this));

  // configure update handler for when the Volume settings change
  _VolumeMqttSettingsService->addUpdateHandler([&](const String& originId) { registerConfig(); }, false);

  // configure settings service update handler to update LED state
  addUpdateHandler([&](const String& originId) { onConfigUpdated(); }, false);
}

void VolumeStateService::begin() {
  Serial.print(F("starting PWM..."));
  pwm.begin();
  
  Serial.print(F("PWM Started."));

   /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(1600); // This is the maximum PWM frequency
  // pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates


  _state.volumeCinema = DEFAULT_VOLUME_STATE;
  _state.volumeCozinha = DEFAULT_VOLUME_STATE;
  _state.volumeSala = DEFAULT_VOLUME_STATE;
  _state.volumeVaranda = DEFAULT_VOLUME_STATE;
  onConfigUpdated();
}

void VolumeStateService::onConfigUpdated() {
  // Servo1AtPCA9685.startEaseToD((float) (_state.volume * (VOLUME_MAX_ROTATION / 360.0)), (uint_fast16_t) 300, DO_NOT_START_UPDATE_BY_INTERRUPT); // rotate to position using speed
  //  do {
  //     // First do the delay, then check for update, since we are probably called directly after start and there is nothing to move yet
  //     delay(REFRESH_INTERVAL_MILLIS); // 20 ms
  // #if defined(PRINT_FOR_SERIAL_PLOTTER)
  //   } while (!updateAllServos()); // this outputs a value plus a newline, whilst Servo1.update() would not output the newline
  // #else
  //   } while (!Servo1AtPCA9685.update());
  // #endif

  Serial.print(F("Updated State Volume Sala: "));
  Serial.print(_state.volumeSala);
  pwm.setPWM(0, 0, (_state.volumeSala * (SERVOMAX / 100.0)));

  
  Serial.print(F("Updated State Volume Cinema: "));
  Serial.print(_state.volumeCinema);
  pwm.setPWM(1, 0, (_state.volumeCinema * (SERVOMAX / 100.0)));

  
  Serial.print(F("Updated State Volume Varanda: "));
  Serial.print(_state.volumeVaranda);
  pwm.setPWM(2, 0, (_state.volumeVaranda * (SERVOMAX / 100.0)));

  
  Serial.print(F("Updated State Volume Cozinha: "));
  Serial.print(_state.volumeCozinha);
  pwm.setPWM(3, 0, (_state.volumeCozinha * (SERVOMAX / 100.0)));
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

#include <Arduino.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ESP32)
#include <WiFi.h>  // Using Espressif's WiFi.h
#else
#include <SPI.h>
#include <Ethernet.h>
#endif

#include <ArduinoHA.h>

// Must specify this before the include of "ServoEasing.hpp"
#define USE_PCA9685_SERVO_EXPANDER  // Activating this enables the use of the PCA9685 I2C expander chip/board.
// #define PCA9685_ACTUAL_CLOCK_FREQUENCY 26000000L // Change it, if your PCA9685 has another than the default 25 MHz
// internal clock. #define USE_SOFT_I2C_MASTER           // Saves 1756 bytes program memory and 218 bytes RAM compared
// with Arduino Wire #define USE_SERVO_LIB                 // If USE_PCA9685_SERVO_EXPANDER is defined, Activating this
// enables force additional using of regular servo library. #define USE_LEIGHTWEIGHT_SERVO_LIB    // Makes the servo
// pulse generating immune to other libraries blocking interrupts for a longer time like SoftwareSerial,
// Adafruit_NeoPixel and DmxSimple.
//  #define PROVIDE_ONLY_LINEAR_MOVEMENT  // Activating this disables all but LINEAR movement. Saves up to 1540 bytes
//  program memory.
#define DISABLE_COMPLEX_FUNCTIONS  // Activating this disables the SINE, CIRCULAR, BACK, ELASTIC, BOUNCE and PRECISION
                                   // easings. Saves up to 1850 bytes program memory.
#define MAX_EASING_SERVOS 12
// #define DISABLE_MICROS_AS_DEGREE_PARAMETER // Activating this disables microsecond values as (target angle)
// parameter. Saves 128 bytes program memory. #define DISABLE_MIN_AND_MAX_CONSTRAINTS    // Activating this disables
// constraints. Saves 4 bytes RAM per servo but strangely enough no program memory. #define DEBUG // Activating this
// enables generate lots of lovely debug output for this library.

/*
 * Specify which easings types should be available.
 * If no easing is defined, all easings are active.
 * This must be done before the #include "ServoEasing.hpp"
 */
#define ENABLE_EASE_QUADRATIC
// #define ENABLE_EASE_CUBIC
// #define ENABLE_EASE_QUARTIC
// #define ENABLE_EASE_SINE
// #define ENABLE_EASE_CIRCULAR
// #define ENABLE_EASE_BACK
// #define ENABLE_EASE_ELASTIC
// #define ENABLE_EASE_BOUNCE
// #define ENABLE_EASE_PRECISION
// #define ENABLE_EASE_USER

// #define PRINT_FOR_SERIAL_PLOTTER      // Activating this enables generate the Arduino plotter output from
// ServoEasing.hpp.
#include "ServoEasing.hpp"

#define SERVO_SALA_RIGHT_PIN 0
#define SERVO_SALA_LEFT_PIN 1

#define SERVO_CINEMA_RIGHT_PIN 2
#define SERVO_CINEMA_LEFT_PIN 3

#define SERVO_VARANDA_RIGHT_PIN 4
#define SERVO_VARANDA_LEFT_PIN 5

#define SERVO_COZINHA_RIGHT_PIN 6

WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device);
HANumber volumeSala("volumeSala");
HANumber volumeCozinha("volumeCozinha");
HANumber volumeCinema("volumeCinema");
HANumber volumeVaranda("volumeVaranda");
static int sleepCounter = 0;
static bool servoSleeping = false;
static bool servoSleeped = false;

ServoEasing ServoSalaRight(
    PCA9685_DEFAULT_ADDRESS);  // If you use more than one PCA9685 you probably must modify MAX_EASING_SERVOS
ServoEasing ServoSalaLeft(
    PCA9685_DEFAULT_ADDRESS);  // If you use more than one PCA9685 you probably must modify MAX_EASING_SERVOS
ServoEasing ServoCinemaRight(
    PCA9685_DEFAULT_ADDRESS);  // If you use more than one PCA9685 you probably must modify MAX_EASING_SERVOS
ServoEasing ServoCinemaLeft(
    PCA9685_DEFAULT_ADDRESS);  // If you use more than one PCA9685 you probably must modify MAX_EASING_SERVOS
ServoEasing ServoVarandaRight(
    PCA9685_DEFAULT_ADDRESS);  // If you use more than one PCA9685 you probably must modify MAX_EASING_SERVOS
ServoEasing ServoVarandaLeft(
    PCA9685_DEFAULT_ADDRESS);  // If you use more than one PCA9685 you probably must modify MAX_EASING_SERVOS
ServoEasing ServoCozinhaRight(
    PCA9685_DEFAULT_ADDRESS);  // If you use more than one PCA9685 you probably must modify MAX_EASING_SERVOS

void AttachServos() {
  // ------------------------------------------------- Sala -------------------------------------------------

  while (ServoSalaRight.attach(SERVO_SALA_RIGHT_PIN, 500, 2500, 0, 270) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo Sala Right"));
  }
  ServoSalaRight.setSpeed(60);                          // 60°/s
  ServoSalaRight.setEasingType(EASE_QUADRATIC_IN_OUT);  // EASE_LINEAR is default
  Serial.println(F("attached servo Sala Right"));


  while (ServoSalaLeft.attach(SERVO_SALA_LEFT_PIN, 500, 2500, 0, 270) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo Sala Left"));
  }
  ServoSalaLeft.setSpeed(60);                          // 60°/s
  ServoSalaLeft.setEasingType(EASE_QUADRATIC_IN_OUT);  // EASE_LINEAR is default

  // ------------------------------------------------- Cinema -------------------------------------------------

  while (ServoCinemaRight.attach(SERVO_CINEMA_RIGHT_PIN, 500, 2500, 0, 270) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo Cinema Right"));
  }
  ServoCinemaRight.setSpeed(60);                          // 60°/s
  ServoCinemaRight.setEasingType(EASE_QUADRATIC_IN_OUT);  // EASE_LINEAR is default

  while (ServoCinemaLeft.attach(SERVO_CINEMA_LEFT_PIN, 500, 2500, 0, 270) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo Cinema Left"));
  }
  ServoCinemaLeft.setSpeed(60);                          // 60°/s
  ServoCinemaLeft.setEasingType(EASE_QUADRATIC_IN_OUT);  // EASE_LINEAR is default

  // ------------------------------------------------- Varanda -------------------------------------------------

  while (ServoVarandaRight.attach(SERVO_VARANDA_RIGHT_PIN, 500, 2500, 0, 270) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo Varanda Right"));
  }
  ServoVarandaRight.setSpeed(60);                          // 60°/s
  ServoVarandaRight.setEasingType(EASE_QUADRATIC_IN_OUT);  // EASE_LINEAR is default

  while (ServoVarandaLeft.attach(SERVO_VARANDA_LEFT_PIN, 500, 2500, 0, 270) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo Varanda Left"));
  }
  ServoVarandaLeft.setSpeed(60);                          // 60°/s
  ServoVarandaLeft.setEasingType(EASE_QUADRATIC_IN_OUT);  // EASE_LINEAR is default

  // ------------------------------------------------- Cozinha -------------------------------------------------

  while (ServoCozinhaRight.attach(SERVO_COZINHA_RIGHT_PIN, 500, 2500, 0, 270) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo Cozinha Mono"));
  }
  ServoCozinhaRight.setSpeed(60);                          // 60°/s
  ServoCozinhaRight.setEasingType(EASE_QUADRATIC_IN_OUT);  // EASE_LINEAR is default
  servoSleeped = false;
  servoSleeping = false;
  sleepCounter = 0;
}

void EaseTo(float degree, ServoEasing* servo) {
  if (!servoSleeping) {
    int counter = 0;
    while(counter < 30000){
      counter++;
    }
  }

  if (servoSleeped) {
    AttachServos();
  }
  // you can do whatever you want with the number as follows:
  float numberInt16 = degree * 2.2f;

  if (numberInt16 > 220) {
    numberInt16 = 220;
  }
  if (numberInt16 < 0) {
    numberInt16 = 0;
  }

  servo->startEaseTo(numberInt16, 60, START_UPDATE_BY_INTERRUPT);
}

void onVolumeSalaCommand(HANumeric number, HANumber* sender) {
  if (!number.isSet()) {
    // the reset command was send by Home Assistant
  } else {
    float numberf = number.toFloat();
    EaseTo(numberf, &ServoSalaRight);
    EaseTo(numberf, &ServoSalaLeft);
  }
  sender->setState(number);  // report the selected option back to the HA panel
}


void onVolumeCinemaCommand(HANumeric number, HANumber* sender) {
  if (!number.isSet()) {
    // the reset command was send by Home Assistant
  } else {
    float numberf = number.toFloat();
    EaseTo(numberf, &ServoCinemaRight);
    EaseTo(numberf, &ServoCinemaLeft);
  }

  sender->setState(number);  // report the selected option back to the HA panel
}

void onVolumeVarandaCommand(HANumeric number, HANumber* sender) {
  if (!number.isSet()) {
    // the reset command was send by Home Assistant
  } else {
    float numberf = number.toFloat();
    EaseTo(numberf, &ServoVarandaRight);
    EaseTo(numberf, &ServoVarandaLeft);
  }

  sender->setState(number);  // report the selected option back to the HA panel
}

void onVolumeCozinhaCommand(HANumeric number, HANumber* sender) {
  if (!number.isSet()) {
    // the reset command was send by Home Assistant
  } else {
    float numberf = number.toFloat();
    EaseTo(numberf, &ServoCozinhaRight);
  }

  sender->setState(number);  // report the selected option back to the HA panel
}

void setup() {
  byte mac[6];
  WiFi.macAddress(mac);
  device.setUniqueId(mac, sizeof(mac));

  // your setup logic goes here,
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("Starting...");

  // Check connection PCA9865
  if (ServoSalaRight.InitializeAndCheckI2CConnection(&Serial)) {
    while (true) {
      Serial.println("Error I2c Connection Sala Right...");
    }
  }

  AttachServos();

  // you can skip this part if you're already maintaining the connection logic
  WiFi.begin("MarriedMe", "TudoIgual!");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);  // waiting for the connection
    Serial.println("Connecting WIFI...");
  }

  Serial.println("WIFI Connected!");

  // set device's details (optional)
  device.setName("NilesController");
  device.setSoftwareVersion("1.0.0");
  device.enableSharedAvailability();
  device.enableLastWill();

  // ----------------------------------------- SALA -----------------------------------------
  // handle command from the HA panel
  volumeSala.onCommand(onVolumeSalaCommand);
  // Optional configuration
  volumeSala.setIcon("mdi:volume-high");
  volumeSala.setName("Volume Sala");
  volumeSala.setMin(0);    // can be float if precision is set via the constructor
  volumeSala.setMax(100);  // can be float if precision is set via the constructor
  volumeSala.setStep(1);   // minimum step: 0.001f
  // number.setMode(HANumber::ModeBox);
  volumeSala.setMode(HANumber::ModeSlider);
  // You can set retain flag for the HA commands
  volumeSala.setRetain(true);

  // ----------------------------------------- CINEMA -----------------------------------------
  // handle command from the HA panel
  volumeCinema.onCommand(onVolumeCinemaCommand);
  // Optional configuration
  volumeCinema.setIcon("mdi:volume-high");
  volumeCinema.setName("Volume Cinema");
  volumeCinema.setMin(0);    // can be float if precision is set via the constructor
  volumeCinema.setMax(100);  // can be float if precision is set via the constructor
  volumeCinema.setStep(1);   // minimum step: 0.001f
  // number.setMode(HANumber::ModeBox);
  volumeCinema.setMode(HANumber::ModeSlider);
  // You can set retain flag for the HA commands
  volumeCinema.setRetain(true);

  // ----------------------------------------- COZINHA -----------------------------------------
  // handle command from the HA panel
  volumeCozinha.onCommand(onVolumeCozinhaCommand);
  // Optional configuration
  volumeCozinha.setIcon("mdi:volume-high");
  volumeCozinha.setName("Volume Cozinha");
  volumeCozinha.setMin(0);    // can be float if precision is set via the constructor
  volumeCozinha.setMax(100);  // can be float if precision is set via the constructor
  volumeCozinha.setStep(1);   // minimum step: 0.001f
  // number.setMode(HANumber::ModeBox);
  volumeCozinha.setMode(HANumber::ModeSlider);
  // You can set retain flag for the HA commands
  volumeCozinha.setRetain(true);

  // ----------------------------------------- VARANDA -----------------------------------------
  // handle command from the HA panel
  volumeVaranda.onCommand(onVolumeVarandaCommand);
  // Optional configuration
  volumeVaranda.setIcon("mdi:volume-high");
  volumeVaranda.setName("Volume Varanda");
  volumeVaranda.setMin(0);    // can be float if precision is set via the constructor
  volumeVaranda.setMax(100);  // can be float if precision is set via the constructor
  volumeVaranda.setStep(1);   // minimum step: 0.001f
  // number.setMode(HANumber::ModeBox);
  volumeVaranda.setMode(HANumber::ModeSlider);
  // You can set retain flag for the HA commands
  volumeVaranda.setRetain(true);

  // MQTT broker connection (use your data here)
  mqtt.begin("192.168.15.248", "ziongh", "Ziongh123!");
  Serial.println("MQTT Connected!");
}

void loop() {
  mqtt.loop();

  // if (sleepCounter >= 30000 && !servoSleeping && !servoSleeped) {
  //   Serial.println(F("Sleeping servos..."));
  //   servoSleeping = true;
  //   ServoSalaRight.detach();
  //   ServoSalaLeft.detach();
  //   ServoCinemaRight.detach();
  //   ServoCinemaLeft.detach();
  //   ServoVarandaRight.detach();
  //   ServoVarandaLeft.detach();
  //   ServoCozinhaRight.detach();
  //   servoSleeped = true;
  //   servoSleeping = false;
  //   sleepCounter = 0;
  // } 
  // else if(sleepCounter <= 30000 && !servoSleeped && !servoSleeping) {
  //   sleepCounter++;
  // }
}

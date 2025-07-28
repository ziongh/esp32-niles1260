#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <ArduinoHA.h>
#include <Preferences.h>

// Must be defined before including ServoEasing.hpp
#define USE_PCA9685_SERVO_EXPANDER
#define DISABLE_COMPLEX_FUNCTIONS
#define MAX_EASING_SERVOS 16 // Increased to handle all entities
#define ENABLE_EASE_QUADRATIC
#include "ServoEasing.hpp"

// =====================================================================================================================
// --- Configuration Constants ---
// =====================================================================================================================

// Network
const char* WIFI_SSID = FACTORY_WIFI_SSID;
const char* WIFI_PASSWORD = FACTORY_WIFI_PASSWORD;
const char* MQTT_HOST = FACTORY_MQTT_HOST;
const uint16_t MQTT_PORT = FACTORY_MQTT_PORT;
const char* MQTT_USER = FACTORY_MQTT_USERNAME;
const char* MQTT_PASSWORD = FACTORY_MQTT_PASSWORD;

// Timing
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 20000;
const unsigned long INITIAL_SYNC_PERIOD_MS = 3000;
const unsigned long SERVO_SLEEP_TIMEOUT_MS = 60000;

// Servo Configuration
const int SERVO_SPEED_DPS = 80;
const float HA_VOLUME_MIN = 0.0f;
const float HA_VOLUME_MAX = 100.0f;
const float HA_BALANCE_MIN = -100.0f;
const float HA_BALANCE_MAX = 100.0f;
const float HA_CALIBRATION_ANGLE_MIN = 0.0f;
const float HA_CALIBRATION_ANGLE_MAX = 270.0f;

// Servo PCA9685 Channels
const uint8_t SERVO_SALA_RIGHT_CH = 0;
const uint8_t SERVO_SALA_LEFT_CH = 1;
const uint8_t SERVO_CINEMA_RIGHT_CH = 2;
const uint8_t SERVO_CINEMA_LEFT_CH = 3;
const uint8_t SERVO_VARANDA_RIGHT_CH = 4;
const uint8_t SERVO_VARANDA_LEFT_CH = 5;
const uint8_t SERVO_COZINHA_RIGHT_CH = 6;

// =====================================================================================================================
// --- Global Objects and State Variables ---
// =====================================================================================================================

// System Objects
WiFiClient wifiClient;
HADevice device;
HAMqtt mqtt(wifiClient, device, 16); // Increased entity limit
Preferences preferences;

// Servo objects
ServoEasing ServoSalaRight(PCA9685_DEFAULT_ADDRESS), ServoSalaLeft(PCA9685_DEFAULT_ADDRESS);
ServoEasing ServoCinemaRight(PCA9685_DEFAULT_ADDRESS), ServoCinemaLeft(PCA9685_DEFAULT_ADDRESS);
ServoEasing ServoVarandaRight(PCA9685_DEFAULT_ADDRESS), ServoVarandaLeft(PCA9685_DEFAULT_ADDRESS);
ServoEasing ServoCozinhaRight(PCA9685_DEFAULT_ADDRESS);

// Struct to hold all data for a single stereo zone
struct StereoZone {
    const char* id;
    const char* name;
    ServoEasing& leftServo;
    ServoEasing& rightServo;
    HANumber& volumeEntity;
    HANumber& balanceEntity;
    HANumber& minAngleLeftEntity;
    HANumber& maxAngleLeftEntity;
    HANumber& minAngleRightEntity;
    HANumber& maxAngleRightEntity;
    float currentVolume;
    float currentBalance;
    float minAngleLeft;
    float maxAngleLeft;
    float minAngleRight;
    float maxAngleRight;
    char balanceName[32];
    char minAngleLeftName[40];
    char maxAngleLeftName[40];
    char minAngleRightName[40];
    char maxAngleRightName[40];
};

// Struct for a mono zone
struct MonoZone {
    const char* id;
    const char* name;
    ServoEasing& servo;
    HANumber& volumeEntity;
    HANumber& minAngleEntity;
    HANumber& maxAngleEntity;
    float currentVolume;
    float minAngle;
    float maxAngle;
    char minAngleName[32];
    char maxAngleName[32];
};

// Home Assistant Entities
HANumber volumeSala("volumeSala"), balanceSala("balanceSala");
HANumber minAngleSalaLeft("minAngleSalaLeft"), maxAngleSalaLeft("maxAngleSalaLeft");
HANumber minAngleSalaRight("minAngleSalaRight"), maxAngleSalaRight("maxAngleSalaRight");

HANumber volumeCinema("volumeCinema"), balanceCinema("balanceCinema");
HANumber minAngleCinemaLeft("minAngleCinemaLeft"), maxAngleCinemaLeft("maxAngleCinemaLeft");
HANumber minAngleCinemaRight("minAngleCinemaRight"), maxAngleCinemaRight("maxAngleCinemaRight");

HANumber volumeVaranda("volumeVaranda"), balanceVaranda("balanceVaranda");
HANumber minAngleVarandaLeft("minAngleVarandaLeft"), maxAngleVarandaLeft("maxAngleVarandaLeft");
HANumber minAngleVarandaRight("minAngleVarandaRight"), maxAngleVarandaRight("maxAngleVarandaRight");

HANumber volumeCozinha("volumeCozinha"), minAngleCozinha("minAngleCozinha"), maxAngleCozinha("maxAngleCozinha");
HANumber masterVolume("masterVolume");

// Array of stereo zones
StereoZone stereoZones[] = {
    { "sala", "Sala", ServoSalaLeft, ServoSalaRight, volumeSala, balanceSala, minAngleSalaLeft, maxAngleSalaLeft, minAngleSalaRight, maxAngleSalaRight, 0.0, 0.0, 0.0, 250.0, 0.0, 250.0 },
    { "cinema", "Cinema", ServoCinemaLeft, ServoCinemaRight, volumeCinema, balanceCinema, minAngleCinemaLeft, maxAngleCinemaLeft, minAngleCinemaRight, maxAngleCinemaRight, 0.0, 0.0, 0.0, 250.0, 0.0, 250.0 },
    { "varanda", "Varanda", ServoVarandaLeft, ServoVarandaRight, volumeVaranda, balanceVaranda, minAngleVarandaLeft, maxAngleVarandaLeft, minAngleVarandaRight, maxAngleVarandaRight, 0.0, 0.0, 0.0, 250.0, 0.0, 250.0 }
};
const int numStereoZones = sizeof(stereoZones) / sizeof(stereoZones);

// Array of mono zones
MonoZone monoZones[] = {
    { "cozinha", "Cozinha", ServoCozinhaRight, volumeCozinha, minAngleCozinha, maxAngleCozinha, 0.0, 0.0, 220.0 }
};
const int numMonoZones = sizeof(monoZones) / sizeof(monoZones);

// State for master volume
float currentMasterVolume = 100.0;

// System State Variables
bool initialSyncComplete = false;
bool reannouncementTriggered = false;
bool statesPublished = false;
bool servosAreSleeping = true;
volatile bool servosBusy = false;
unsigned long lastActivityTime = 0;
unsigned long mqttConnectedTime = 0;

// Mutex for critical sections
portMUX_TYPE commandMutex = portMUX_INITIALIZER_UNLOCKED;

// =====================================================================================================================
// --- Function Prototypes ---
// =====================================================================================================================
void connectWiFi();
void checkConnections();
void wakeUpServos();
void sleepServos();
void setupHaNumber(HANumber& number, const char* name, const char* icon, float minVal, float maxVal, float step);
void moveServoToAngle(ServoEasing& servo, float targetAngle, const char* name);
void updateStereoPairServos(StereoZone& zone);
void updateMonoServo(MonoZone& zone);

// Callbacks
void unifiedStereoVolumeCallback(HANumeric number, HANumber* sender);
void unifiedStereoBalanceCallback(HANumeric number, HANumber* sender);
void unifiedStereoMinAngleCallback(HANumeric number, HANumber* sender);
void unifiedStereoMaxAngleCallback(HANumeric number, HANumber* sender);
void unifiedMonoVolumeCallback(HANumeric number, HANumber* sender);
void unifiedMonoMinAngleCallback(HANumeric number, HANumber* sender);
void unifiedMonoMaxAngleCallback(HANumeric number, HANumber* sender);
void onMasterVolumeCommand(HANumeric number, HANumber* sender);

// =====================================================================================================================
// --- Main Setup and Loop ---
// =====================================================================================================================

void setup() {
    Serial.begin(115200);
    Serial.println(F("\n\nStarting Niles Volume Controller v7.1.0..."));

    byte mac[6];
    WiFi.macAddress(mac);
    device.setUniqueId(mac, sizeof(mac));

    connectWiFi();

    if (ServoSalaRight.InitializeAndCheckI2CConnection(&Serial)) {
        Serial.println(F("FATAL ERROR: PCA9685 not found. Halting."));
        while (true) { delay(1000); }
    }
    Serial.println(F("PCA9685 board found."));

    device.setName("Niles Controller");
    device.setSoftwareVersion("7.1.0");
    device.enableSharedAvailability();
    device.enableLastWill();

    preferences.begin("niles-ctrl", false);

    // Load preferences for stereo zones
    for (int i = 0; i < numStereoZones; i++) {
        char minLeftKey[24], maxLeftKey[24], minRightKey[24], maxRightKey[24];
        snprintf(minLeftKey, sizeof(minLeftKey), "%s_min_angle_l", stereoZones[i].id);
        snprintf(maxLeftKey, sizeof(maxLeftKey), "%s_max_angle_l", stereoZones[i].id);
        snprintf(minRightKey, sizeof(minRightKey), "%s_min_angle_r", stereoZones[i].id);
        snprintf(maxRightKey, sizeof(maxRightKey), "%s_max_angle_r", stereoZones[i].id);
        
        stereoZones[i].minAngleLeft = preferences.getFloat(minLeftKey, stereoZones[i].minAngleLeft);
        stereoZones[i].maxAngleLeft = preferences.getFloat(maxLeftKey, stereoZones[i].maxAngleLeft);
        stereoZones[i].minAngleRight = preferences.getFloat(minRightKey, stereoZones[i].minAngleRight);
        stereoZones[i].maxAngleRight = preferences.getFloat(maxRightKey, stereoZones[i].maxAngleRight);
    }

    // Load preferences for mono zones
    for (int i = 0; i < numMonoZones; i++) {
        char minKey[20], maxKey[20];
        snprintf(minKey, sizeof(minKey), "%s_min_angle", monoZones[i].id);
        snprintf(maxKey, sizeof(maxKey), "%s_max_angle", monoZones[i].id);
        monoZones[i].minAngle = preferences.getFloat(minKey, monoZones[i].minAngle);
        monoZones[i].maxAngle = preferences.getFloat(maxKey, monoZones[i].maxAngle);
    }
    preferences.end();
    
    // Setup HA entities for stereo zones
    for (int i = 0; i < numStereoZones; i++) {
        snprintf(stereoZones[i].balanceName, sizeof(stereoZones[i].balanceName), "Balance %s", stereoZones[i].name);
        snprintf(stereoZones[i].minAngleLeftName, sizeof(stereoZones[i].minAngleLeftName), "Min Angle %s Left", stereoZones[i].name);
        snprintf(stereoZones[i].maxAngleLeftName, sizeof(stereoZones[i].maxAngleLeftName), "Max Angle %s Left", stereoZones[i].name);
        snprintf(stereoZones[i].minAngleRightName, sizeof(stereoZones[i].minAngleRightName), "Min Angle %s Right", stereoZones[i].name);
        snprintf(stereoZones[i].maxAngleRightName, sizeof(stereoZones[i].maxAngleRightName), "Max Angle %s Right", stereoZones[i].name);

        setupHaNumber(stereoZones[i].volumeEntity, stereoZones[i].name, "mdi:volume-high", HA_VOLUME_MIN, HA_VOLUME_MAX, 1);
        stereoZones[i].volumeEntity.onCommand(unifiedStereoVolumeCallback);

        setupHaNumber(stereoZones[i].balanceEntity, stereoZones[i].balanceName, "mdi:speaker-multiple", HA_BALANCE_MIN, HA_BALANCE_MAX, 1);
        stereoZones[i].balanceEntity.onCommand(unifiedStereoBalanceCallback);
        
        setupHaNumber(stereoZones[i].minAngleLeftEntity, stereoZones[i].minAngleLeftName, "mdi:page-layout-header-footer", HA_CALIBRATION_ANGLE_MIN, HA_CALIBRATION_ANGLE_MAX, 1.0);
        stereoZones[i].minAngleLeftEntity.onCommand(unifiedStereoMinAngleCallback);

        setupHaNumber(stereoZones[i].maxAngleLeftEntity, stereoZones[i].maxAngleLeftName, "mdi:page-layout-header-footer", HA_CALIBRATION_ANGLE_MIN, HA_CALIBRATION_ANGLE_MAX, 1.0);
        stereoZones[i].maxAngleLeftEntity.onCommand(unifiedStereoMaxAngleCallback);

        setupHaNumber(stereoZones[i].minAngleRightEntity, stereoZones[i].minAngleRightName, "mdi:page-layout-header-footer", HA_CALIBRATION_ANGLE_MIN, HA_CALIBRATION_ANGLE_MAX, 1.0);
        stereoZones[i].minAngleRightEntity.onCommand(unifiedStereoMinAngleCallback);

        setupHaNumber(stereoZones[i].maxAngleRightEntity, stereoZones[i].maxAngleRightName, "mdi:page-layout-header-footer", HA_CALIBRATION_ANGLE_MIN, HA_CALIBRATION_ANGLE_MAX, 1.0);
        stereoZones[i].maxAngleRightEntity.onCommand(unifiedStereoMaxAngleCallback);
    }

    // Setup HA entities for mono zones
    for (int i = 0; i < numMonoZones; i++) {
        snprintf(monoZones[i].minAngleName, sizeof(monoZones[i].minAngleName), "Min Angle %s", monoZones[i].name);
        snprintf(monoZones[i].maxAngleName, sizeof(monoZones[i].maxAngleName), "Max Angle %s", monoZones[i].name);

        setupHaNumber(monoZones[i].volumeEntity, monoZones[i].name, "mdi:silverware-fork-knife", HA_VOLUME_MIN, HA_VOLUME_MAX, 1);
        monoZones[i].volumeEntity.onCommand(unifiedMonoVolumeCallback);

        setupHaNumber(monoZones[i].minAngleEntity, monoZones[i].minAngleName, "mdi:page-layout-header-footer", HA_CALIBRATION_ANGLE_MIN, HA_CALIBRATION_ANGLE_MAX, 1.0);
        monoZones[i].minAngleEntity.onCommand(unifiedMonoMinAngleCallback);

        setupHaNumber(monoZones[i].maxAngleEntity, monoZones[i].maxAngleName, "mdi:page-layout-header-footer", HA_CALIBRATION_ANGLE_MIN, HA_CALIBRATION_ANGLE_MAX, 1.0);
        monoZones[i].maxAngleEntity.onCommand(unifiedMonoMaxAngleCallback);
    }

    setupHaNumber(masterVolume, "Master Volume", "mdi:volume-vibrate", HA_VOLUME_MIN, HA_VOLUME_MAX, 1);
    masterVolume.onCommand(onMasterVolumeCommand);

    Serial.println(F("Connecting to MQTT broker..."));
    mqtt.begin(MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASSWORD);
}

void loop() {
    mqtt.loop();
    
    if (servosBusy &&!ServoEasing::areInterruptsActive()) {
        Serial.println(F("Servo movement complete. Releasing lock."));
        taskENTER_CRITICAL(&commandMutex);
        servosBusy = false;
        taskEXIT_CRITICAL(&commandMutex);
    }

    if (mqtt.isConnected() &&!reannouncementTriggered) {
        Serial.println("Connection stable. Forcing full discovery re-announcement...");
        mqtt.publish("homeassistant/status", "online", true);
        Serial.println("Re-announcement message sent to homeassistant/status.");
        reannouncementTriggered = true; 
    }

    if (reannouncementTriggered &&!statesPublished) {
        Serial.println("Publishing initial states...");
        
        for (int i = 0; i < numStereoZones; i++) {
            stereoZones[i].minAngleLeftEntity.setState(stereoZones[i].minAngleLeft);
            stereoZones[i].maxAngleLeftEntity.setState(stereoZones[i].maxAngleLeft);
            stereoZones[i].minAngleRightEntity.setState(stereoZones[i].minAngleRight);
            stereoZones[i].maxAngleRightEntity.setState(stereoZones[i].maxAngleRight);
            stereoZones[i].volumeEntity.setState(stereoZones[i].currentVolume);
            stereoZones[i].balanceEntity.setState(stereoZones[i].currentBalance);
        }
        for (int i = 0; i < numMonoZones; i++) {
            monoZones[i].minAngleEntity.setState(monoZones[i].minAngle);
            monoZones[i].maxAngleEntity.setState(monoZones[i].maxAngle);
            monoZones[i].volumeEntity.setState(monoZones[i].currentVolume);
        }
        masterVolume.setState(currentMasterVolume);

        statesPublished = true;
        Serial.println("Initial states published.");
    }

    if (mqtt.isConnected() &&!initialSyncComplete) {
        if (mqttConnectedTime == 0) {
            mqttConnectedTime = millis();
        }
        if (millis() - mqttConnectedTime > INITIAL_SYNC_PERIOD_MS) {
            initialSyncComplete = true;
            Serial.println(F("Initial sync grace period complete. Ready for commands."));
            taskENTER_CRITICAL(&commandMutex);
            lastActivityTime = millis();
            taskEXIT_CRITICAL(&commandMutex);
        }
    }

    // Check if servos should go to sleep
    taskENTER_CRITICAL(&commandMutex);
    unsigned long currentTime = millis();
    taskEXIT_CRITICAL(&commandMutex);
    bool shouldSleep = false;
    taskENTER_CRITICAL(&commandMutex);
    if (initialSyncComplete && !servosAreSleeping && !servosBusy && (currentTime - lastActivityTime > SERVO_SLEEP_TIMEOUT_MS)) {
        shouldSleep = true;
    }
    taskEXIT_CRITICAL(&commandMutex);

    if (shouldSleep) {
        sleepServos();
    }
}

// =====================================================================================================================
// --- Home Assistant Callback Functions ---
// =====================================================================================================================

void unifiedStereoVolumeCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet()) return;
    
    taskENTER_CRITICAL(&commandMutex);
    if (servosBusy) {
        taskEXIT_CRITICAL(&commandMutex);
        Serial.println(F("Volume command ignored: Servos are busy."));
        return;
    }
    servosBusy = true;
    taskEXIT_CRITICAL(&commandMutex);

    for (int i = 0; i < numStereoZones; i++) {
        if (sender == &stereoZones[i].volumeEntity) {
            stereoZones[i].currentVolume = number.toFloat();
            sender->setState(number);
            if (initialSyncComplete) {
                taskENTER_CRITICAL(&commandMutex);
                updateStereoPairServos(stereoZones[i]);
                lastActivityTime = millis();
                taskEXIT_CRITICAL(&commandMutex);
            }
            break;
        }
    }
}

void unifiedStereoBalanceCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet()) return;

    taskENTER_CRITICAL(&commandMutex);
    if (servosBusy) {
        taskEXIT_CRITICAL(&commandMutex);
        Serial.println(F("Balance command ignored: Servos are busy."));
        return;
    }
    servosBusy = true;
    taskEXIT_CRITICAL(&commandMutex);

    for (int i = 0; i < numStereoZones; i++) {
        if (sender == &stereoZones[i].balanceEntity) {
            stereoZones[i].currentBalance = number.toFloat();
            sender->setState(number); 
            if (initialSyncComplete) {
                taskENTER_CRITICAL(&commandMutex);
                updateStereoPairServos(stereoZones[i]);
                lastActivityTime = millis();
                taskEXIT_CRITICAL(&commandMutex);
            }
            break;
        }
    }
}

void unifiedStereoMinAngleCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet()) return;

    taskENTER_CRITICAL(&commandMutex);
    if (servosBusy) {
        taskEXIT_CRITICAL(&commandMutex);
        Serial.println(F("Min angle command ignored: Servos are busy."));
        return;
    }
    servosBusy = true;
    taskEXIT_CRITICAL(&commandMutex);

    for (int i = 0; i < numStereoZones; i++) {
        if (sender == &stereoZones[i].minAngleLeftEntity || sender == &stereoZones[i].minAngleRightEntity) {
            taskENTER_CRITICAL(&commandMutex);
            float newMin = number.toFloat();
            sender->setState(number);
            
            preferences.begin("niles-ctrl", false);
            if (sender == &stereoZones[i].minAngleLeftEntity) {
                stereoZones[i].minAngleLeft = newMin;
                char minKey[1];
                snprintf(minKey, sizeof(minKey), "%s_min_angle_l", stereoZones[i].id);
                preferences.putFloat(minKey, newMin);
                Serial.printf("Saved new min angle for %s Left: %.1f\n", stereoZones[i].name, newMin);
                moveServoToAngle(stereoZones[i].leftServo, newMin, "Left Calib");
            } else { // Right
                stereoZones[i].minAngleRight = newMin;
                char minKey[1];
                snprintf(minKey, sizeof(minKey), "%s_min_angle_r", stereoZones[i].id);
                preferences.putFloat(minKey, newMin);
                Serial.printf("Saved new min angle for %s Right: %.1f\n", stereoZones[i].name, newMin);
                moveServoToAngle(stereoZones[i].rightServo, newMin, "Right Calib");
            }
            preferences.end();
            lastActivityTime = millis();
            taskEXIT_CRITICAL(&commandMutex);
            break;
        }
    }
}

void unifiedStereoMaxAngleCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet()) return;

    taskENTER_CRITICAL(&commandMutex);
    if (servosBusy) {
        taskEXIT_CRITICAL(&commandMutex);
        Serial.println(F("Max angle command ignored: Servos are busy."));
        return;
    }
    servosBusy = true;
    taskEXIT_CRITICAL(&commandMutex);

    for (int i = 0; i < numStereoZones; i++) {
        if (sender == &stereoZones[i].maxAngleLeftEntity || sender == &stereoZones[i].maxAngleRightEntity) {
            taskENTER_CRITICAL(&commandMutex);
            float newMax = number.toFloat();
            sender->setState(number);

            preferences.begin("niles-ctrl", false);
            if (sender == &stereoZones[i].maxAngleLeftEntity) {
                stereoZones[i].maxAngleLeft = newMax;
                char maxKey[1];
                snprintf(maxKey, sizeof(maxKey), "%s_max_angle_l", stereoZones[i].id);
                preferences.putFloat(maxKey, newMax);
                Serial.printf("Saved new max angle for %s Left: %.1f\n", stereoZones[i].name, newMax);
                moveServoToAngle(stereoZones[i].leftServo, newMax, "Left Calib");
            } else { // Right
                stereoZones[i].maxAngleRight = newMax;
                char maxKey[1];
                snprintf(maxKey, sizeof(maxKey), "%s_max_angle_r", stereoZones[i].id);
                preferences.putFloat(maxKey, newMax);
                Serial.printf("Saved new max angle for %s Right: %.1f\n", stereoZones[i].name, newMax);
                moveServoToAngle(stereoZones[i].rightServo, newMax, "Right Calib");
            }
            preferences.end();
            lastActivityTime = millis();
            taskEXIT_CRITICAL(&commandMutex);
            break;
        }
    }
}

void unifiedMonoVolumeCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet()) return;

    taskENTER_CRITICAL(&commandMutex);
    if (servosBusy) {
        taskEXIT_CRITICAL(&commandMutex);
        Serial.println(F("Mono volume command ignored: Servos are busy."));
        return;
    }
    servosBusy = true;
    taskEXIT_CRITICAL(&commandMutex);

    for (int i = 0; i < numMonoZones; i++) {
        if (sender == &monoZones[i].volumeEntity) {
            monoZones[i].currentVolume = number.toFloat();
            sender->setState(number);
            if (initialSyncComplete) {
                taskENTER_CRITICAL(&commandMutex);
                updateMonoServo(monoZones[i]);
                lastActivityTime = millis();
                taskEXIT_CRITICAL(&commandMutex);
            }
            break;
        }
    }
}

void unifiedMonoMinAngleCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet()) return;

    taskENTER_CRITICAL(&commandMutex);
    if (servosBusy) {
        taskEXIT_CRITICAL(&commandMutex);
        Serial.println(F("Mono min angle command ignored: Servos are busy."));
        return;
    }
    servosBusy = true;
    taskEXIT_CRITICAL(&commandMutex);

    for (int i = 0; i < numMonoZones; i++) {
        if (sender == &monoZones[i].minAngleEntity) {
            taskENTER_CRITICAL(&commandMutex);
            float newMin = number.toFloat();
            monoZones[i].minAngle = newMin;
            sender->setState(number);

            preferences.begin("niles-ctrl", false);
            char minKey[4];
            snprintf(minKey, sizeof(minKey), "%s_min_angle", monoZones[i].id);
            preferences.putFloat(minKey, newMin);
            preferences.end();
            Serial.printf("Saved new min angle for %s: %.1f\n", monoZones[i].name, newMin);

            moveServoToAngle(monoZones[i].servo, newMin, "Calib");
            lastActivityTime = millis();
            taskEXIT_CRITICAL(&commandMutex);
            break;
        }
    }
}

void unifiedMonoMaxAngleCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet()) return;

    taskENTER_CRITICAL(&commandMutex);
    if (servosBusy) {
        taskEXIT_CRITICAL(&commandMutex);
        Serial.println(F("Mono max angle command ignored: Servos are busy."));
        return;
    }
    servosBusy = true;
    taskEXIT_CRITICAL(&commandMutex);

    for (int i = 0; i < numMonoZones; i++) {
        if (sender == &monoZones[i].maxAngleEntity) {
            taskENTER_CRITICAL(&commandMutex);
            float newMax = number.toFloat();
            monoZones[i].maxAngle = newMax;
            sender->setState(number);

            preferences.begin("niles-ctrl", false);
            char maxKey[4];
            snprintf(maxKey, sizeof(maxKey), "%s_max_angle", monoZones[i].id);
            preferences.putFloat(maxKey, newMax);
            preferences.end();
            Serial.printf("Saved new max angle for %s: %.1f\n", monoZones[i].name, newMax);

            moveServoToAngle(monoZones[i].servo, newMax, "Calib");
            lastActivityTime = millis();
            taskEXIT_CRITICAL(&commandMutex);
            break;
        }
    }
}

void onMasterVolumeCommand(HANumeric number, HANumber* sender) {
    if (!number.isSet() ||!initialSyncComplete) return;

    taskENTER_CRITICAL(&commandMutex);
    if (servosBusy) {
        taskEXIT_CRITICAL(&commandMutex);
        Serial.println(F("Master volume command ignored: Servos are busy."));
        return;
    }
    servosBusy = true;
    taskEXIT_CRITICAL(&commandMutex);

    currentMasterVolume = number.toFloat();
    sender->setState(number);
    Serial.printf("Master Volume set to %.1f. Updating all servos.\n", currentMasterVolume);

    
    taskENTER_CRITICAL(&commandMutex);
    for (int i = 0; i < numStereoZones; i++) {
        updateStereoPairServos(stereoZones[i]);
    }
    for (int i = 0; i < numMonoZones; i++) {
        updateMonoServo(monoZones[i]);
    }
    lastActivityTime = millis();
    taskEXIT_CRITICAL(&commandMutex);
}

// =====================================================================================================================
// --- Helper Functions ---
// =====================================================================================================================

void updateStereoPairServos(StereoZone& zone) {
    Serial.printf("Updating pair '%s': Vol=%.1f, Bal=%.1f\n", zone.name, zone.currentVolume, zone.currentBalance);

    float masterMultiplier = currentMasterVolume / HA_VOLUME_MAX;
    float effectiveVolume = zone.currentVolume * masterMultiplier;

    float baseAngleLeft = map(effectiveVolume, HA_VOLUME_MIN, HA_VOLUME_MAX, zone.minAngleLeft, zone.maxAngleLeft);
    float baseAngleRight = map(effectiveVolume, HA_VOLUME_MIN, HA_VOLUME_MAX, zone.minAngleRight, zone.maxAngleRight);

    float leftMultiplier = 1.0, rightMultiplier = 1.0;
    if (zone.currentBalance > 0) { // Balance to the right
        leftMultiplier = map(zone.currentBalance, 0, HA_BALANCE_MAX, 1.0, 0.0);
    } else if (zone.currentBalance < 0) { // Balance to the left
        rightMultiplier = map(zone.currentBalance, 0, HA_BALANCE_MIN, 1.0, 0.0);
    }

    moveServoToAngle(zone.leftServo, baseAngleLeft * leftMultiplier, "Left");
    moveServoToAngle(zone.rightServo, baseAngleRight * rightMultiplier, "Right");
}

void updateMonoServo(MonoZone& zone) {
    Serial.printf("Updating mono '%s': Vol=%.1f, MinAng=%.1f, MaxAng=%.1f\n",
                  zone.name, zone.currentVolume, zone.minAngle, zone.maxAngle);

    float masterMultiplier = currentMasterVolume / HA_VOLUME_MAX;
    float effectiveVolume = zone.currentVolume * masterMultiplier;

    float targetAngle = map(effectiveVolume, HA_VOLUME_MIN, HA_VOLUME_MAX, zone.minAngle, zone.maxAngle);
    moveServoToAngle(zone.servo, targetAngle, zone.name);
}

void moveServoToAngle(ServoEasing& servo, float targetAngle, const char* name) {
    // Assumes lock is already held by calling function
    if (servosAreSleeping) {
        wakeUpServos();
    }
    lastActivityTime = millis(); // Update activity time on any move command

    targetAngle = constrain(targetAngle, HA_CALIBRATION_ANGLE_MIN, HA_CALIBRATION_ANGLE_MAX);

    Serial.printf("  > Moving %s servo to angle %.1f\n", name, targetAngle);
    servo.startEaseTo(targetAngle, SERVO_SPEED_DPS, START_UPDATE_BY_INTERRUPT);
}

void setupHaNumber(HANumber& number, const char* name, const char* icon, float minVal, float maxVal, float step) {
    number.setName(name);
    number.setIcon(icon);
    number.setMin(minVal);
    number.setMax(maxVal);
    number.setStep(step);
    number.setMode(HANumber::ModeSlider);
    number.setRetain(false); 
}

void connectWiFi() {
    Serial.print(F("Connecting to WiFi: "));
    Serial.print(WIFI_SSID);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.setAutoReconnect(true);

    unsigned long startTime = millis();
    while (WiFi.status()!= WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if (millis() - startTime > WIFI_CONNECT_TIMEOUT_MS) {
            Serial.println(F("\nWiFi connection failed! Rebooting..."));
            delay(1000);
            ESP.restart();
        }
    }
    Serial.print(F("\nWiFi Connected! IP: "));
    Serial.println(WiFi.localIP());
}

void checkConnections() {
    if (WiFi.status()!= WL_CONNECTED) {
        Serial.println(F("WiFi connection lost! Rebooting..."));
        delay(1000);
        ESP.restart();
    }
}

void wakeUpServos() {
    if (!servosAreSleeping) return;
    Serial.println(F("Waking up servos..."));

    auto attachAndConfigure = [](ServoEasing& servo, uint8_t channel, const char* name){
        if (servo.attach(channel, 500, 2500, 0, 270) == INVALID_SERVO) {
             Serial.printf("ERROR attaching servo %s\n", name);
        }
        servo.setSpeed(SERVO_SPEED_DPS);
        servo.setEasingType(EASE_QUADRATIC_IN_OUT);
    };

    attachAndConfigure(ServoSalaRight, SERVO_SALA_RIGHT_CH, "Sala Right");
    attachAndConfigure(ServoSalaLeft, SERVO_SALA_LEFT_CH, "Sala Left");
    attachAndConfigure(ServoCinemaRight, SERVO_CINEMA_RIGHT_CH, "Cinema Right");
    attachAndConfigure(ServoCinemaLeft, SERVO_CINEMA_LEFT_CH, "Cinema Left");
    attachAndConfigure(ServoVarandaRight, SERVO_VARANDA_RIGHT_CH, "Varanda Right");
    attachAndConfigure(ServoVarandaLeft, SERVO_VARANDA_LEFT_CH, "Varanda Left");
    attachAndConfigure(ServoCozinhaRight, SERVO_COZINHA_RIGHT_CH, "Cozinha Right");
    delay(50); // Small delay to allow servos to initialize
    
    servosAreSleeping = false;
    Serial.println(F("Servos are awake."));
}

void sleepServos() {
    taskENTER_CRITICAL(&commandMutex);
    if (servosAreSleeping) {
        taskEXIT_CRITICAL(&commandMutex);
        return;
    }
    Serial.println(F("Inactivity detected. Sleeping servos..."));
    ServoSalaRight.detach();
    ServoSalaLeft.detach();
    ServoCinemaRight.detach();
    ServoCinemaLeft.detach();
    ServoVarandaRight.detach();
    ServoVarandaLeft.detach();
    ServoCozinhaRight.detach();
    
    servosAreSleeping = true;
    taskEXIT_CRITICAL(&commandMutex);
}
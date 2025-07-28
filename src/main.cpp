#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <ArduinoHA.h>
#include <Preferences.h>

// Must be defined before including ServoEasing.hpp
#define USE_PCA9685_SERVO_EXPANDER
#define DISABLE_COMPLEX_FUNCTIONS
#define MAX_EASING_SERVOS 16
#define ENABLE_EASE_QUADRATIC
#include "ServoEasing.hpp"

// =====================================================================================================================
// --- Configuration Constants ---
// =====================================================================================================================

// Network (from platformio.ini)
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
HAMqtt mqtt(wifiClient, device);
Preferences preferences;

// Servo objects
ServoEasing ServoSalaRight(PCA9685_DEFAULT_ADDRESS), ServoSalaLeft(PCA9685_DEFAULT_ADDRESS);
ServoEasing ServoCinemaRight(PCA9685_DEFAULT_ADDRESS), ServoCinemaLeft(PCA9685_DEFAULT_ADDRESS);
ServoEasing ServoVarandaRight(PCA9685_DEFAULT_ADDRESS), ServoVarandaLeft(PCA9685_DEFAULT_ADDRESS);
ServoEasing ServoCozinhaRight(PCA9685_DEFAULT_ADDRESS);

// Locking for thread-safe servo operations
portMUX_TYPE servoActivityLock = portMUX_INITIALIZER_UNLOCKED;

// Struct to hold all data for a single stereo zone with per-servo calibration
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

// Struct to hold all data for a single mono zone
struct SingleZone {
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
HANumber minAngleSalaLeft("minAngleSalaLeft"), maxAngleSalaLeft("maxAngleSalaLeft"), minAngleSalaRight("minAngleSalaRight"), maxAngleSalaRight("maxAngleSalaRight");

HANumber volumeCinema("volumeCinema"), balanceCinema("balanceCinema");
HANumber minAngleCinemaLeft("minAngleCinemaLeft"), maxAngleCinemaLeft("maxAngleCinemaLeft"), minAngleCinemaRight("minAngleCinemaRight"), maxAngleCinemaRight("maxAngleCinemaRight");

HANumber volumeVaranda("volumeVaranda"), balanceVaranda("balanceVaranda");
HANumber minAngleVarandaLeft("minAngleVarandaLeft"), maxAngleVarandaLeft("maxAngleVarandaLeft"), minAngleVarandaRight("minAngleVarandaRight"), maxAngleVarandaRight("maxAngleVarandaRight");

HANumber volumeCozinha("volumeCozinha"), minAngleCozinha("minAngleCozinha"), maxAngleCozinha("maxAngleCozinha");
HANumber masterVolume("masterVolume");

// Array of stereo zones
StereoZone zones[] = {
    { "sala", "Sala", ServoSalaLeft, ServoSalaRight, volumeSala, balanceSala,
      minAngleSalaLeft, maxAngleSalaLeft, minAngleSalaRight, maxAngleSalaRight,
      0.0, 0.0, 0.0, 250.0, 0.0, 250.0 },
    { "cinema", "Cinema", ServoCinemaLeft, ServoCinemaRight, volumeCinema, balanceCinema,
      minAngleCinemaLeft, maxAngleCinemaLeft, minAngleCinemaRight, maxAngleCinemaRight,
      0.0, 0.0, 0.0, 250.0, 0.0, 250.0 },
    { "varanda", "Varanda", ServoVarandaLeft, ServoVarandaRight, volumeVaranda, balanceVaranda,
      minAngleVarandaLeft, maxAngleVarandaLeft, minAngleVarandaRight, maxAngleVarandaRight,
      0.0, 0.0, 0.0, 250.0, 0.0, 250.0 }
};
const int numStereoZones = sizeof(zones) / sizeof(zones[0]);

// Single zone definition
SingleZone cozinhaZone = { "cozinha", "Cozinha", ServoCozinhaRight, volumeCozinha, minAngleCozinha, maxAngleCozinha, 0.0, 0.0, 250.0 };

// System State Variables
float lastMasterVolume = 100.0;
bool initialSyncComplete = false;
bool reannouncementTriggered = false;
bool statesPublished = false;
bool servosAreSleeping = true;
unsigned long lastActivityTime = 0;
unsigned long mqttConnectedTime = 0;

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
void updateSingleZoneServos(SingleZone& zone);
void updateAllServoPositions();

// Callbacks
void unifiedVolumeCallback(HANumeric number, HANumber* sender);
void unifiedBalanceCallback(HANumeric number, HANumber* sender);
void unifiedMinAngleCallback(HANumeric number, HANumber* sender);
void unifiedMaxAngleCallback(HANumeric number, HANumber* sender);
void onVolumeCozinhaCommand(HANumeric number, HANumber* sender);
void onMinAngleCozinhaCommand(HANumeric number, HANumber* sender);
void onMaxAngleCozinhaCommand(HANumeric number, HANumber* sender);
void onMasterVolumeCommand(HANumeric number, HANumber* sender);


// =====================================================================================================================
// --- Main Setup and Loop ---
// =====================================================================================================================

void setup() {
    Serial.begin(115200);
    Serial.println(F("\n\nStarting Niles Volume Controller..."));

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
    device.setSoftwareVersion("6.1.0");
    device.enableSharedAvailability();
    device.enableLastWill();

    preferences.begin("niles-ctrl", false);

    // Load saved settings for stereo zones
    for (int i = 0; i < numStereoZones; i++) {
        char minKeyL[24], maxKeyL[24], minKeyR[24], maxKeyR[24];
        snprintf(minKeyL, sizeof(minKeyL), "%s_min_angle_l", zones[i].id);
        snprintf(maxKeyL, sizeof(maxKeyL), "%s_max_angle_l", zones[i].id);
        snprintf(minKeyR, sizeof(minKeyR), "%s_min_angle_r", zones[i].id);
        snprintf(maxKeyR, sizeof(maxKeyR), "%s_max_angle_r", zones[i].id);
        zones[i].minAngleLeft = preferences.getFloat(minKeyL, zones[i].minAngleLeft);
        zones[i].maxAngleLeft = preferences.getFloat(maxKeyL, zones[i].maxAngleLeft);
        zones[i].minAngleRight = preferences.getFloat(minKeyR, zones[i].minAngleRight);
        zones[i].maxAngleRight = preferences.getFloat(maxKeyR, zones[i].maxAngleRight);
    }

    // Load saved settings for single zones (Cozinha)
    char cozMinKey[20], cozMaxKey[20];
    snprintf(cozMinKey, sizeof(cozMinKey), "%s_min_angle", cozinhaZone.id);
    snprintf(cozMaxKey, sizeof(cozMaxKey), "%s_max_angle", cozinhaZone.id);
    cozinhaZone.minAngle = preferences.getFloat(cozMinKey, cozinhaZone.minAngle);
    cozinhaZone.maxAngle = preferences.getFloat(cozMaxKey, cozinhaZone.maxAngle);

    // Setup HA entities for stereo zones
    for (int i = 0; i < numStereoZones; i++) {
        snprintf(zones[i].balanceName, sizeof(zones[i].balanceName), "Balance %s", zones[i].name);
        snprintf(zones[i].minAngleLeftName, sizeof(zones[i].minAngleLeftName), "Min Angle %s Left", zones[i].name);
        snprintf(zones[i].maxAngleLeftName, sizeof(zones[i].maxAngleLeftName), "Max Angle %s Left", zones[i].name);
        snprintf(zones[i].minAngleRightName, sizeof(zones[i].minAngleRightName), "Min Angle %s Right", zones[i].name);
        snprintf(zones[i].maxAngleRightName, sizeof(zones[i].maxAngleRightName), "Max Angle %s Right", zones[i].name);

        setupHaNumber(zones[i].volumeEntity, zones[i].name, "mdi:volume-high", HA_VOLUME_MIN, HA_VOLUME_MAX, 1);
        zones[i].volumeEntity.onCommand(unifiedVolumeCallback);

        setupHaNumber(zones[i].balanceEntity, zones[i].balanceName, "mdi:speaker-multiple", HA_BALANCE_MIN, HA_BALANCE_MAX, 1);
        zones[i].balanceEntity.onCommand(unifiedBalanceCallback);

        // Setup individual angle controls
        const char* icon = "mdi:page-layout-header-footer";
        setupHaNumber(zones[i].minAngleLeftEntity, zones[i].minAngleLeftName, icon, HA_CALIBRATION_ANGLE_MIN, HA_CALIBRATION_ANGLE_MAX, 1.0);
        zones[i].minAngleLeftEntity.onCommand(unifiedMinAngleCallback);
        setupHaNumber(zones[i].maxAngleLeftEntity, zones[i].maxAngleLeftName, icon, HA_CALIBRATION_ANGLE_MIN, HA_CALIBRATION_ANGLE_MAX, 1.0);
        zones[i].maxAngleLeftEntity.onCommand(unifiedMaxAngleCallback);
        setupHaNumber(zones[i].minAngleRightEntity, zones[i].minAngleRightName, icon, HA_CALIBRATION_ANGLE_MIN, HA_CALIBRATION_ANGLE_MAX, 1.0);
        zones[i].minAngleRightEntity.onCommand(unifiedMinAngleCallback);
        setupHaNumber(zones[i].maxAngleRightEntity, zones[i].maxAngleRightName, icon, HA_CALIBRATION_ANGLE_MIN, HA_CALIBRATION_ANGLE_MAX, 1.0);
        zones[i].maxAngleRightEntity.onCommand(unifiedMaxAngleCallback);
    }

    // Setup HA entities for Cozinha
    snprintf(cozinhaZone.minAngleName, sizeof(cozinhaZone.minAngleName), "Min Angle %s", cozinhaZone.name);
    snprintf(cozinhaZone.maxAngleName, sizeof(cozinhaZone.maxAngleName), "Max Angle %s", cozinhaZone.name);
    setupHaNumber(volumeCozinha, "Volume Cozinha", "mdi:silverware-fork-knife", HA_VOLUME_MIN, HA_VOLUME_MAX, 1);
    volumeCozinha.onCommand(onVolumeCozinhaCommand);
    setupHaNumber(minAngleCozinha, cozinhaZone.minAngleName, "mdi:page-layout-header-footer", HA_CALIBRATION_ANGLE_MIN, HA_CALIBRATION_ANGLE_MAX, 1.0);
    minAngleCozinha.onCommand(onMinAngleCozinhaCommand);
    setupHaNumber(maxAngleCozinha, cozinhaZone.maxAngleName, "mdi:page-layout-header-footer", HA_CALIBRATION_ANGLE_MIN, HA_CALIBRATION_ANGLE_MAX, 1.0);
    maxAngleCozinha.onCommand(onMaxAngleCozinhaCommand);

    // Setup Master Volume
    setupHaNumber(masterVolume, "Master Volume", "mdi:volume-vibrate", HA_VOLUME_MIN, HA_VOLUME_MAX, 1);
    masterVolume.onCommand(onMasterVolumeCommand);

    Serial.println(F("Connecting to MQTT broker..."));
    mqtt.begin(MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASSWORD);
}

void loop() {
    mqtt.loop();

    if (mqtt.isConnected() && !reannouncementTriggered) {
        Serial.println("Connection stable. Forcing full discovery re-announcement...");
        mqtt.publish("homeassistant/status", "online", true);
        Serial.println("Re-announcement message sent to homeassistant/status.");
        reannouncementTriggered = true;
    }

    if (reannouncementTriggered && !statesPublished) {
        Serial.println("Publishing initial states...");
        for (int i = 0; i < numStereoZones; i++) {
            zones[i].minAngleLeftEntity.setState(zones[i].minAngleLeft);
            zones[i].maxAngleLeftEntity.setState(zones[i].maxAngleLeft);
            zones[i].minAngleRightEntity.setState(zones[i].minAngleRight);
            zones[i].maxAngleRightEntity.setState(zones[i].maxAngleRight);
            zones[i].volumeEntity.setState(zones[i].currentVolume);
            zones[i].balanceEntity.setState(zones[i].currentBalance);
        }
        volumeCozinha.setState(cozinhaZone.currentVolume);
        minAngleCozinha.setState(cozinhaZone.minAngle);
        maxAngleCozinha.setState(cozinhaZone.maxAngle);
        masterVolume.setState(lastMasterVolume);

        statesPublished = true;
        Serial.println("Initial states published.");
    }

    if (mqtt.isConnected() && !initialSyncComplete) {
        if (mqttConnectedTime == 0) {
            mqttConnectedTime = millis();
        }
        if (millis() - mqttConnectedTime > INITIAL_SYNC_PERIOD_MS) {
            initialSyncComplete = true;
            Serial.println(F("Initial sync grace period complete. Ready for commands."));
            taskENTER_CRITICAL(&servoActivityLock);
            lastActivityTime = millis();
            taskEXIT_CRITICAL(&servoActivityLock);
        }
    }

    // Check if servos should go to sleep
    unsigned long currentTime = millis();
    bool shouldSleep = false;
    taskENTER_CRITICAL(&servoActivityLock);
    if (!servosAreSleeping && (currentTime - lastActivityTime > SERVO_SLEEP_TIMEOUT_MS)) {
        shouldSleep = true;
    }
    taskEXIT_CRITICAL(&servoActivityLock);

    if (shouldSleep) {
        sleepServos();
    }
}


// =====================================================================================================================
// --- Home Assistant Callback Functions ---
// =====================================================================================================================

void unifiedVolumeCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet() || !initialSyncComplete) return;
    for (int i = 0; i < numStereoZones; i++) {
        if (sender == &zones[i].volumeEntity) {
            taskENTER_CRITICAL(&servoActivityLock);
            zones[i].currentVolume = number.toFloat();
            sender->setState(number); // Confirm state back to HA
            updateStereoPairServos(zones[i]);
            taskEXIT_CRITICAL(&servoActivityLock);
            break;
        }
    }
}

void unifiedBalanceCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet() || !initialSyncComplete) return;
    for (int i = 0; i < numStereoZones; i++) {
        if (sender == &zones[i].balanceEntity) {
            taskENTER_CRITICAL(&servoActivityLock);
            zones[i].currentBalance = number.toFloat();
            sender->setState(number);
            updateStereoPairServos(zones[i]);
            taskEXIT_CRITICAL(&servoActivityLock);
            break;
        }
    }
}

void unifiedMinAngleCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet() || !initialSyncComplete) return;
    for (int i = 0; i < numStereoZones; i++) {
        if (sender == &zones[i].minAngleLeftEntity) {
            taskENTER_CRITICAL(&servoActivityLock);
            float newMin = number.toFloat();
            zones[i].minAngleLeft = newMin;
            sender->setState(number);

            char minKey[24];
            snprintf(minKey, sizeof(minKey), "%s_min_angle_l", zones[i].id);
            preferences.putFloat(minKey, newMin);
            Serial.printf("Saved new min angle for %s Left: %.1f\n", zones[i].name, newMin);

            moveServoToAngle(zones[i].leftServo, newMin, "Left Calib");
            taskEXIT_CRITICAL(&servoActivityLock);
            break;
        }
        if (sender == &zones[i].minAngleRightEntity) {
            taskENTER_CRITICAL(&servoActivityLock);
            float newMin = number.toFloat();
            zones[i].minAngleRight = newMin;
            sender->setState(number);

            char minKey[24];
            snprintf(minKey, sizeof(minKey), "%s_min_angle_r", zones[i].id);
            preferences.putFloat(minKey, newMin);
            Serial.printf("Saved new min angle for %s Right: %.1f\n", zones[i].name, newMin);

            moveServoToAngle(zones[i].rightServo, newMin, "Right Calib");
            taskEXIT_CRITICAL(&servoActivityLock);
            break;
        }
    }
}

void unifiedMaxAngleCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet() || !initialSyncComplete) return;
    for (int i = 0; i < numStereoZones; i++) {
        if (sender == &zones[i].maxAngleLeftEntity) {
            taskENTER_CRITICAL(&servoActivityLock);
            float newMax = number.toFloat();
            zones[i].maxAngleLeft = newMax;
            sender->setState(number);

            char maxKey[24];
            snprintf(maxKey, sizeof(maxKey), "%s_max_angle_l", zones[i].id);
            preferences.putFloat(maxKey, newMax);
            Serial.printf("Saved new max angle for %s Left: %.1f\n", zones[i].name, newMax);

            moveServoToAngle(zones[i].leftServo, newMax, "Left Calib");
            taskEXIT_CRITICAL(&servoActivityLock);
            break;
        }
        if (sender == &zones[i].maxAngleRightEntity) {
            taskENTER_CRITICAL(&servoActivityLock);
            float newMax = number.toFloat();
            zones[i].maxAngleRight = newMax;
            sender->setState(number);

            char maxKey[24];
            snprintf(maxKey, sizeof(maxKey), "%s_max_angle_r", zones[i].id);
            preferences.putFloat(maxKey, newMax);
            Serial.printf("Saved new max angle for %s Right: %.1f\n", zones[i].name, newMax);

            moveServoToAngle(zones[i].rightServo, newMax, "Right Calib");
            taskEXIT_CRITICAL(&servoActivityLock);
            break;
        }
    }
}

void onVolumeCozinhaCommand(HANumeric number, HANumber* sender) {
    if (!number.isSet() || !initialSyncComplete) return;
    taskENTER_CRITICAL(&servoActivityLock);
    cozinhaZone.currentVolume = number.toFloat();
    sender->setState(number);
    updateSingleZoneServos(cozinhaZone);
    taskEXIT_CRITICAL(&servoActivityLock);
}

void onMinAngleCozinhaCommand(HANumeric number, HANumber* sender) {
    if (!number.isSet() || !initialSyncComplete) return;
    taskENTER_CRITICAL(&servoActivityLock);
    float newMin = number.toFloat();
    cozinhaZone.minAngle = newMin;
    sender->setState(number);

    char minKey[20];
    snprintf(minKey, sizeof(minKey), "%s_min_angle", cozinhaZone.id);
    preferences.putFloat(minKey, newMin);
    Serial.printf("Saved new min angle for %s: %.1f\n", cozinhaZone.name, newMin);

    moveServoToAngle(cozinhaZone.servo, newMin, "Cozinha Calib");
    taskEXIT_CRITICAL(&servoActivityLock);
}

void onMaxAngleCozinhaCommand(HANumeric number, HANumber* sender) {
    if (!number.isSet() || !initialSyncComplete) return;
    taskENTER_CRITICAL(&servoActivityLock);
    float newMax = number.toFloat();
    cozinhaZone.maxAngle = newMax;
    sender->setState(number);

    char maxKey[20];
    snprintf(maxKey, sizeof(maxKey), "%s_max_angle", cozinhaZone.id);
    preferences.putFloat(maxKey, newMax);
    Serial.printf("Saved new max angle for %s: %.1f\n", cozinhaZone.name, newMax);

    moveServoToAngle(cozinhaZone.servo, newMax, "Cozinha Calib");
    taskEXIT_CRITICAL(&servoActivityLock);
}

void onMasterVolumeCommand(HANumeric number, HANumber* sender) {
    if (!number.isSet() || !initialSyncComplete) return;

    taskENTER_CRITICAL(&servoActivityLock);
    lastMasterVolume = number.toFloat();
    sender->setState(number); // Update master slider itself

    Serial.printf("Master Volume set to %.1f. Recalculating all zones.\n", lastMasterVolume);
    updateAllServoPositions(); // This will apply the new master volume to all zones
    taskEXIT_CRITICAL(&servoActivityLock);
}


// =====================================================================================================================
// --- Helper Functions ---
// =====================================================================================================================

// Updates all servos based on their current state and the master volume
void updateAllServoPositions() {
    // Assumes lock is already held
    for (int i = 0; i < numStereoZones; i++) {
        updateStereoPairServos(zones[i]);
    }
    updateSingleZoneServos(cozinhaZone);
}

void updateStereoPairServos(StereoZone& zone) {
    // Assumes lock is already held.
    // This function calculates final servo positions based on a hierarchy:
    // 1. Apply Balance to the zone's base volume to get per-channel volume.
    // 2. Apply the global Master Volume to the per-channel volume.
    // 3. Map the final per-channel volume to its unique angle range.

    float masterMultiplier = lastMasterVolume / 100.0f;

    float leftVol = zone.currentVolume;
    float rightVol = zone.currentVolume;

    // 1. Apply Balance
    if (zone.currentBalance > 0) { // Balance to the right, attenuate left
        leftVol *= map(zone.currentBalance, 0, HA_BALANCE_MAX, 1.0, 0.0);
    } else if (zone.currentBalance < 0) { // Balance to the left, attenuate right
        rightVol *= map(zone.currentBalance, 0, HA_BALANCE_MIN, 1.0, 0.0);
    }

    // 2. Apply Master Volume
    leftVol *= masterMultiplier;
    rightVol *= masterMultiplier;

    // 3. Constrain final values
    leftVol = constrain(leftVol, HA_VOLUME_MIN, HA_VOLUME_MAX);
    rightVol = constrain(rightVol, HA_VOLUME_MIN, HA_VOLUME_MAX);

    Serial.printf("Updating pair '%s': StoredVol=%.1f, Bal=%.1f, Master=%.1f -> EffVol L:%.1f R:%.1f\n",
                  zone.name, zone.currentVolume, zone.currentBalance, lastMasterVolume, leftVol, rightVol);

    // 4. Map final volumes to their respective angle ranges
    float targetAngleLeft = map(leftVol, HA_VOLUME_MIN, HA_VOLUME_MAX, zone.minAngleLeft, zone.maxAngleLeft);
    float targetAngleRight = map(rightVol, HA_VOLUME_MIN, HA_VOLUME_MAX, zone.minAngleRight, zone.maxAngleRight);

    // 5. Move servos
    moveServoToAngle(zone.leftServo, targetAngleLeft, "Left");
    moveServoToAngle(zone.rightServo, targetAngleRight, "Right");
}

void updateSingleZoneServos(SingleZone& zone) {
    // Assumes lock is already held
    float effectiveVolume = zone.currentVolume * (lastMasterVolume / 100.0f);
    effectiveVolume = constrain(effectiveVolume, HA_VOLUME_MIN, HA_VOLUME_MAX);

    Serial.printf("Updating single '%s': StoredVol=%.1f, Master=%.1f -> EffVol=%.1f\n",
                  zone.name, zone.currentVolume, lastMasterVolume, effectiveVolume);

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
    number.setRetain(true); // Retain state so HA remembers it across reboots
}

void connectWiFi() {
    Serial.print(F("Connecting to WiFi: "));
    Serial.print(WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.setAutoReconnect(true);

    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED) {
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
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("WiFi connection lost! Rebooting..."));
        delay(1000);
        ESP.restart();
    }
}

void wakeUpServos() {
    // Assumes lock is already held
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
    taskENTER_CRITICAL(&servoActivityLock);
    if (servosAreSleeping) {
        taskEXIT_CRITICAL(&servoActivityLock);
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
    taskEXIT_CRITICAL(&servoActivityLock);
}
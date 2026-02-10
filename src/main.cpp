#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <ArduinoHA.h>
#include <Preferences.h>
#include <functional>
#include <math.h>

// Must be defined before including ServoEasing.hpp
#define USE_PCA9685_SERVO_EXPANDER
#define DISABLE_COMPLEX_FUNCTIONS
#define MAX_EASING_SERVOS 16
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
const unsigned long SERVO_SLEEP_TIMEOUT_MS = 15000;
const unsigned long REBOOT_SAFETY_CLEAR_MS = 60000;
const int REBOOT_SAFE_MODE_THRESHOLD = 3;

// Servo Configuration
const int SERVO_SPEED_DPS = 80;
const float HA_VOLUME_MIN = 0.0f;
const float HA_VOLUME_MAX = 100.0f;
const float HA_BALANCE_MIN = -100.0f;
const float HA_BALANCE_MAX = 100.0f;
const float HA_CALIBRATION_ANGLE_MIN = 0.0f;
const float HA_CALIBRATION_ANGLE_MAX = 270.0f;
const float ANGLE_DEDUP_THRESHOLD = 0.5f;

// Servo PCA9685 Channels
const uint8_t SERVO_SALA_RIGHT_CH = 0;
const uint8_t SERVO_SALA_LEFT_CH = 1;
const uint8_t SERVO_CINEMA_RIGHT_CH = 2;
const uint8_t SERVO_CINEMA_LEFT_CH = 3;
const uint8_t SERVO_VARANDA_RIGHT_CH = 4;
const uint8_t SERVO_VARANDA_LEFT_CH = 5;
const uint8_t SERVO_COZINHA_RIGHT_CH = 6;

// Startup phase state machine
enum StartupPhase {
    PHASE_WAITING_MQTT,
    PHASE_SYNCING,
    PHASE_STARTUP_MOVE,
    PHASE_RUNNING
};

// Reboot counter (survives software resets, cleared on power cycle)
RTC_DATA_ATTR int rebootCount = 0;

// =====================================================================================================================
// --- Global Objects and State Variables ---
// =====================================================================================================================

// System Objects
WiFiClient wifiClient;
HADevice device;
HAMqtt mqtt(wifiClient, device, 24);
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
    float lastAngleLeft;
    float lastAngleRight;
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
    float lastAngle;
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
StereoZone stereoZones[] = {
    { "sala", "Sala", ServoSalaLeft, ServoSalaRight, volumeSala, balanceSala, minAngleSalaLeft, maxAngleSalaLeft, minAngleSalaRight, maxAngleSalaRight, 0.0, 0.0, 0.0, 250.0, 0.0, 250.0, 0.0f, 0.0f },
    { "cinema", "Cinema", ServoCinemaLeft, ServoCinemaRight, volumeCinema, balanceCinema, minAngleCinemaLeft, maxAngleCinemaLeft, minAngleCinemaRight, maxAngleCinemaRight, 0.0, 0.0, 0.0, 250.0, 0.0, 250.0, 0.0f, 0.0f },
    { "varanda", "Varanda", ServoVarandaLeft, ServoVarandaRight, volumeVaranda, balanceVaranda, minAngleVarandaLeft, maxAngleVarandaLeft, minAngleVarandaRight, maxAngleVarandaRight, 0.0, 0.0, 0.0, 250.0, 0.0, 250.0, 0.0f, 0.0f }
};
const int numStereoZones = sizeof(stereoZones) / sizeof(stereoZones[0]);

// Array of mono zones
MonoZone monoZones[] = {
    { "cozinha", "Cozinha", ServoCozinhaRight, volumeCozinha, minAngleCozinha, maxAngleCozinha, 0.0, 0.0, 220.0, 0.0f }
};
const int numMonoZones = sizeof(monoZones) / sizeof(monoZones[0]);

// State for master volume
float currentMasterVolume = 100.0;

// System State Variables
StartupPhase startupPhase = PHASE_WAITING_MQTT;
bool servosAreSleeping = true;
volatile bool servosBusy = false;
unsigned long lastActivityTime = 0;
unsigned long mqttConnectedTime = 0;
bool wasMqttConnected = false;
bool wifiWasConnected = true;
bool safeMode = false;
bool rebootCounterCleared = false;
unsigned long wifiDisconnectedSince = 0;
const unsigned long WIFI_FORCE_RECONNECT_MS = 30000;

// Mutex for critical sections
portMUX_TYPE commandMutex = portMUX_INITIALIZER_UNLOCKED;

// =====================================================================================================================
// --- Function Prototypes ---
// =====================================================================================================================
float mapf(float x, float in_min, float in_max, float out_min, float out_max);
void computeStereoAngles(const StereoZone& zone, float& outLeftAngle, float& outRightAngle);
float computeMonoAngle(const MonoZone& zone);
void connectWiFi();
void wakeUpServos();
void sleepServos();
void saveVolumesToNVS();
void setupHaNumber(HANumber& number, const char* name, const char* icon, float minVal, float maxVal, float step);
void moveServoToAngle(ServoEasing& servo, float targetAngle, const char* name, float& lastAngle);
void moveServoToAngle(ServoEasing& servo, float targetAngle, const char* name);
void updateStereoPairServos(StereoZone& zone);
void updateMonoServo(MonoZone& zone);
void updateAllServoTargets();
void processCommand(std::function<void()> action);

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
    Serial.println(F("\n\nStarting Niles Volume Controller v7.3.0..."));

    rebootCount++;
    Serial.printf("Boot count (RTC): %d\n", rebootCount);
    if (rebootCount > REBOOT_SAFE_MODE_THRESHOLD) {
        safeMode = true;
        Serial.println(F("WARNING: Repeated reboots detected. Entering safe mode (servos disabled at startup)."));
    }

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
    device.setSoftwareVersion("7.3.0");
    device.enableSharedAvailability();
    device.enableLastWill();

    preferences.begin("niles-ctrl", false);
    for (int i = 0; i < numStereoZones; i++) {
        char minLeftKey[32], maxLeftKey[32], minRightKey[32], maxRightKey[32];
        snprintf(minLeftKey, sizeof(minLeftKey), "%s_min_angle_l", stereoZones[i].id);
        snprintf(maxLeftKey, sizeof(maxLeftKey), "%s_max_angle_l", stereoZones[i].id);
        snprintf(minRightKey, sizeof(minRightKey), "%s_min_angle_r", stereoZones[i].id);
        snprintf(maxRightKey, sizeof(maxRightKey), "%s_max_angle_r", stereoZones[i].id);

        stereoZones[i].minAngleLeft = preferences.getFloat(minLeftKey, stereoZones[i].minAngleLeft);
        stereoZones[i].maxAngleLeft = preferences.getFloat(maxLeftKey, stereoZones[i].maxAngleLeft);
        stereoZones[i].minAngleRight = preferences.getFloat(minRightKey, stereoZones[i].minAngleRight);
        stereoZones[i].maxAngleRight = preferences.getFloat(maxRightKey, stereoZones[i].maxAngleRight);

        char volKey[32], balKey[32];
        snprintf(volKey, sizeof(volKey), "%s_vol", stereoZones[i].id);
        snprintf(balKey, sizeof(balKey), "%s_bal", stereoZones[i].id);
        stereoZones[i].currentVolume = preferences.getFloat(volKey, 0.0f);
        stereoZones[i].currentBalance = preferences.getFloat(balKey, 0.0f);
    }
    for (int i = 0; i < numMonoZones; i++) {
        char minKey[32], maxKey[32], volKey[32];
        snprintf(minKey, sizeof(minKey), "%s_min_angle", monoZones[i].id);
        snprintf(maxKey, sizeof(maxKey), "%s_max_angle", monoZones[i].id);
        snprintf(volKey, sizeof(volKey), "%s_vol", monoZones[i].id);
        monoZones[i].minAngle = preferences.getFloat(minKey, monoZones[i].minAngle);
        monoZones[i].maxAngle = preferences.getFloat(maxKey, monoZones[i].maxAngle);
        monoZones[i].currentVolume = preferences.getFloat(volKey, 0.0f);
    }
    currentMasterVolume = preferences.getFloat("master_vol", 100.0f);
    preferences.end();

    // Pre-compute initial servo angles from NVS state (used for attach-at-position)
    for (int i = 0; i < numStereoZones; i++) {
        computeStereoAngles(stereoZones[i], stereoZones[i].lastAngleLeft, stereoZones[i].lastAngleRight);
    }
    for (int i = 0; i < numMonoZones; i++) {
        monoZones[i].lastAngle = computeMonoAngle(monoZones[i]);
    }

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

        const char* icon = "mdi:page-layout-header-footer";
        setupHaNumber(stereoZones[i].minAngleLeftEntity, stereoZones[i].minAngleLeftName, icon, HA_CALIBRATION_ANGLE_MIN, HA_CALIBRATION_ANGLE_MAX, 1.0);
        stereoZones[i].minAngleLeftEntity.onCommand(unifiedStereoMinAngleCallback);
        setupHaNumber(stereoZones[i].maxAngleLeftEntity, stereoZones[i].maxAngleLeftName, icon, HA_CALIBRATION_ANGLE_MIN, HA_CALIBRATION_ANGLE_MAX, 1.0);
        stereoZones[i].maxAngleLeftEntity.onCommand(unifiedStereoMaxAngleCallback);
        setupHaNumber(stereoZones[i].minAngleRightEntity, stereoZones[i].minAngleRightName, icon, HA_CALIBRATION_ANGLE_MIN, HA_CALIBRATION_ANGLE_MAX, 1.0);
        stereoZones[i].minAngleRightEntity.onCommand(unifiedStereoMinAngleCallback);
        setupHaNumber(stereoZones[i].maxAngleRightEntity, stereoZones[i].maxAngleRightName, icon, HA_CALIBRATION_ANGLE_MIN, HA_CALIBRATION_ANGLE_MAX, 1.0);
        stereoZones[i].maxAngleRightEntity.onCommand(unifiedStereoMaxAngleCallback);
    }

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

    wifiClient.setNoDelay(true);
    Serial.println(F("Connecting to MQTT broker..."));
    mqtt.begin(MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASSWORD);
}

void loop() {
    mqtt.loop();

    // WiFi disconnect monitoring with forced reconnection
    bool wifiConnected = (WiFi.status() == WL_CONNECTED);
    if (!wifiConnected && wifiWasConnected) {
        Serial.println(F("WiFi disconnected! Waiting for auto-reconnect..."));
        wifiDisconnectedSince = millis();
    } else if (wifiConnected && !wifiWasConnected) {
        Serial.print(F("WiFi reconnected. IP: "));
        Serial.println(WiFi.localIP());
        esp_wifi_set_ps(WIFI_PS_NONE);
        wifiDisconnectedSince = 0;
    } else if (!wifiConnected && !wifiWasConnected && wifiDisconnectedSince > 0 &&
               (millis() - wifiDisconnectedSince > WIFI_FORCE_RECONNECT_MS)) {
        Serial.println(F("WiFi auto-reconnect failed. Forcing reconnect cycle..."));
        WiFi.disconnect();
        delay(100);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        wifiDisconnectedSince = millis();
    }
    wifiWasConnected = wifiConnected;

    // Detect MQTT connection state changes
    bool mqttConnected = mqtt.isConnected();
    if (!mqttConnected && wasMqttConnected) {
        Serial.println(F("MQTT disconnected."));
        startupPhase = PHASE_WAITING_MQTT;
    }
    if (mqttConnected && !wasMqttConnected) {
        Serial.println(F("MQTT connected. Entering sync phase..."));
        startupPhase = PHASE_SYNCING;
        mqttConnectedTime = millis();
    }
    wasMqttConnected = mqttConnected;

    // Servo movement completion detection
    if (servosBusy && !ServoEasing::areInterruptsActive()) {
        taskENTER_CRITICAL(&commandMutex);
        servosBusy = false;
        lastActivityTime = millis();
        taskEXIT_CRITICAL(&commandMutex);
        Serial.println(F("Servo movement complete."));
    }

    // Phase-based state machine
    switch (startupPhase) {
        case PHASE_WAITING_MQTT:
            break;

        case PHASE_SYNCING:
            if (millis() - mqttConnectedTime > INITIAL_SYNC_PERIOD_MS) {
                Serial.println(F("Sync period complete."));
                startupPhase = PHASE_STARTUP_MOVE;
            }
            break;

        case PHASE_STARTUP_MOVE: {
            // Publish post-sync state to HA (merged NVS defaults + MQTT retained updates)
            // Force re-publish all states to HA (force=true bypasses ArduinoHA's
            // deduplication, ensuring states reach the broker even if values haven't
            // changed since last publish — critical after MQTT reconnection when the
            // broker may have lost retained messages)
            Serial.println(F("Force-publishing post-sync states to HA..."));
            for (int i = 0; i < numStereoZones; i++) {
                stereoZones[i].volumeEntity.setState(stereoZones[i].currentVolume, true);
                stereoZones[i].balanceEntity.setState(stereoZones[i].currentBalance, true);
                stereoZones[i].minAngleLeftEntity.setState(stereoZones[i].minAngleLeft, true);
                stereoZones[i].maxAngleLeftEntity.setState(stereoZones[i].maxAngleLeft, true);
                stereoZones[i].minAngleRightEntity.setState(stereoZones[i].minAngleRight, true);
                stereoZones[i].maxAngleRightEntity.setState(stereoZones[i].maxAngleRight, true);
            }
            for (int i = 0; i < numMonoZones; i++) {
                monoZones[i].volumeEntity.setState(monoZones[i].currentVolume, true);
                monoZones[i].minAngleEntity.setState(monoZones[i].minAngle, true);
                monoZones[i].maxAngleEntity.setState(monoZones[i].maxAngle, true);
            }
            masterVolume.setState(currentMasterVolume, true);

            // Transition to RUNNING so moveServoToAngle will execute
            startupPhase = PHASE_RUNNING;
            servosBusy = true;

            if (safeMode) {
                Serial.println(F("Safe mode active: skipping startup servo move."));
            } else if (!servosAreSleeping) {
                // MQTT reconnect while servos were already awake: ease to new targets
                Serial.println(F("Servos already awake. Easing to post-sync targets..."));
                updateAllServoTargets();
            } else {
                // Normal boot: wake servos at NVS-computed angles, then ease to post-sync targets
                Serial.println(F("Performing startup servo positioning..."));
                wakeUpServos();
                updateAllServoTargets();
            }

            taskENTER_CRITICAL(&commandMutex);
            lastActivityTime = millis();
            taskEXIT_CRITICAL(&commandMutex);

            Serial.println(F("Startup complete. Ready for commands."));
            break;
        }

        case PHASE_RUNNING: {
            // Servo sleep timeout
            bool shouldSleep = false;
            taskENTER_CRITICAL(&commandMutex);
            if (!servosAreSleeping && !servosBusy && (millis() - lastActivityTime > SERVO_SLEEP_TIMEOUT_MS)) {
                shouldSleep = true;
            }
            taskEXIT_CRITICAL(&commandMutex);
            if (shouldSleep) {
                sleepServos();
            }

            // Clear reboot counter after stable operation
            if (!rebootCounterCleared && millis() > REBOOT_SAFETY_CLEAR_MS) {
                rebootCount = 0;
                rebootCounterCleared = true;
                if (safeMode) {
                    safeMode = false;
                    Serial.println(F("Safe mode cleared after stable operation. Normal operation resumed."));
                }
            }
            break;
        }
    }

    delay(1); // Yield to FreeRTOS WiFi/TCP background tasks
}

// =====================================================================================================================
// --- Home Assistant Callback Functions ---
// =====================================================================================================================

void processCommand(std::function<void()> action) {
    if (startupPhase == PHASE_SYNCING) {
        // During sync: run action for state updates only.
        // moveServoToAngle() will no-op because startupPhase != PHASE_RUNNING.
        action();
        return;
    }
    if (startupPhase != PHASE_RUNNING) return;

    taskENTER_CRITICAL(&commandMutex);
    if (servosBusy) {
        taskEXIT_CRITICAL(&commandMutex);
        Serial.println(F("Command ignored: Servos are busy."));
        return;
    }
    servosBusy = true;
    taskEXIT_CRITICAL(&commandMutex);

    action();
}

void unifiedStereoVolumeCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet()) return;
    auto action = [&]() {
        for (int i = 0; i < numStereoZones; i++) {
            if (sender == &stereoZones[i].volumeEntity) {
                stereoZones[i].currentVolume = number.toFloat();
                sender->setState(number);
                updateStereoPairServos(stereoZones[i]);
                break;
            }
        }
    };
    processCommand(action);
}

void unifiedStereoBalanceCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet()) return;
    auto action = [&]() {
        for (int i = 0; i < numStereoZones; i++) {
            if (sender == &stereoZones[i].balanceEntity) {
                stereoZones[i].currentBalance = number.toFloat();
                sender->setState(number);
                updateStereoPairServos(stereoZones[i]);
                break;
            }
        }
    };
    processCommand(action);
}

void unifiedStereoMinAngleCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet()) return;
    auto action = [&]() {
        for (int i = 0; i < numStereoZones; i++) {
            if (sender == &stereoZones[i].minAngleLeftEntity || sender == &stereoZones[i].minAngleRightEntity) {
                float newMin = number.toFloat();
                bool isLeft = (sender == &stereoZones[i].minAngleLeftEntity);
                float currentMax = isLeft ? stereoZones[i].maxAngleLeft : stereoZones[i].maxAngleRight;
                if (newMin >= currentMax) {
                    Serial.printf("Rejected min angle %.1f for %s %s: must be less than max (%.1f)\n",
                                  newMin, stereoZones[i].name, isLeft ? "Left" : "Right", currentMax);
                    sender->setState(isLeft ? stereoZones[i].minAngleLeft : stereoZones[i].minAngleRight);
                    break;
                }
                sender->setState(number);
                preferences.begin("niles-ctrl", false);
                if (isLeft) {
                    stereoZones[i].minAngleLeft = newMin;
                    char key[32];
                    snprintf(key, sizeof(key), "%s_min_angle_l", stereoZones[i].id);
                    preferences.putFloat(key, newMin);
                    Serial.printf("Saved new min angle for %s Left: %.1f\n", stereoZones[i].name, newMin);
                    moveServoToAngle(stereoZones[i].leftServo, newMin, "Left Calib");
                } else {
                    stereoZones[i].minAngleRight = newMin;
                    char key[32];
                    snprintf(key, sizeof(key), "%s_min_angle_r", stereoZones[i].id);
                    preferences.putFloat(key, newMin);
                    Serial.printf("Saved new min angle for %s Right: %.1f\n", stereoZones[i].name, newMin);
                    moveServoToAngle(stereoZones[i].rightServo, newMin, "Right Calib");
                }
                preferences.end();
                break;
            }
        }
    };
    processCommand(action);
}

void unifiedStereoMaxAngleCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet()) return;
    auto action = [&]() {
        for (int i = 0; i < numStereoZones; i++) {
            if (sender == &stereoZones[i].maxAngleLeftEntity || sender == &stereoZones[i].maxAngleRightEntity) {
                float newMax = number.toFloat();
                bool isLeft = (sender == &stereoZones[i].maxAngleLeftEntity);
                float currentMin = isLeft ? stereoZones[i].minAngleLeft : stereoZones[i].minAngleRight;
                if (newMax <= currentMin) {
                    Serial.printf("Rejected max angle %.1f for %s %s: must be greater than min (%.1f)\n",
                                  newMax, stereoZones[i].name, isLeft ? "Left" : "Right", currentMin);
                    sender->setState(isLeft ? stereoZones[i].maxAngleLeft : stereoZones[i].maxAngleRight);
                    break;
                }
                sender->setState(number);
                preferences.begin("niles-ctrl", false);
                if (isLeft) {
                    stereoZones[i].maxAngleLeft = newMax;
                    char key[32];
                    snprintf(key, sizeof(key), "%s_max_angle_l", stereoZones[i].id);
                    preferences.putFloat(key, newMax);
                    Serial.printf("Saved new max angle for %s Left: %.1f\n", stereoZones[i].name, newMax);
                    moveServoToAngle(stereoZones[i].leftServo, newMax, "Left Calib");
                } else {
                    stereoZones[i].maxAngleRight = newMax;
                    char key[32];
                    snprintf(key, sizeof(key), "%s_max_angle_r", stereoZones[i].id);
                    preferences.putFloat(key, newMax);
                    Serial.printf("Saved new max angle for %s Right: %.1f\n", stereoZones[i].name, newMax);
                    moveServoToAngle(stereoZones[i].rightServo, newMax, "Right Calib");
                }
                preferences.end();
                break;
            }
        }
    };
    processCommand(action);
}

void unifiedMonoVolumeCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet()) return;
    auto action = [&]() {
        for (int i = 0; i < numMonoZones; i++) {
            if (sender == &monoZones[i].volumeEntity) {
                monoZones[i].currentVolume = number.toFloat();
                sender->setState(number);
                updateMonoServo(monoZones[i]);
                break;
            }
        }
    };
    processCommand(action);
}

void unifiedMonoMinAngleCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet()) return;
    auto action = [&]() {
        for (int i = 0; i < numMonoZones; i++) {
            if (sender == &monoZones[i].minAngleEntity) {
                float newMin = number.toFloat();
                if (newMin >= monoZones[i].maxAngle) {
                    Serial.printf("Rejected min angle %.1f for %s: must be less than max (%.1f)\n",
                                  newMin, monoZones[i].name, monoZones[i].maxAngle);
                    sender->setState(monoZones[i].minAngle);
                    break;
                }
                monoZones[i].minAngle = newMin;
                sender->setState(number);
                preferences.begin("niles-ctrl", false);
                char key[32];
                snprintf(key, sizeof(key), "%s_min_angle", monoZones[i].id);
                preferences.putFloat(key, newMin);
                preferences.end();
                Serial.printf("Saved new min angle for %s: %.1f\n", monoZones[i].name, newMin);
                moveServoToAngle(monoZones[i].servo, newMin, "Calib");
                break;
            }
        }
    };
    processCommand(action);
}

void unifiedMonoMaxAngleCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet()) return;
    auto action = [&]() {
        for (int i = 0; i < numMonoZones; i++) {
            if (sender == &monoZones[i].maxAngleEntity) {
                float newMax = number.toFloat();
                if (newMax <= monoZones[i].minAngle) {
                    Serial.printf("Rejected max angle %.1f for %s: must be greater than min (%.1f)\n",
                                  newMax, monoZones[i].name, monoZones[i].minAngle);
                    sender->setState(monoZones[i].maxAngle);
                    break;
                }
                monoZones[i].maxAngle = newMax;
                sender->setState(number);
                preferences.begin("niles-ctrl", false);
                char key[32];
                snprintf(key, sizeof(key), "%s_max_angle", monoZones[i].id);
                preferences.putFloat(key, newMax);
                preferences.end();
                Serial.printf("Saved new max angle for %s: %.1f\n", monoZones[i].name, newMax);
                moveServoToAngle(monoZones[i].servo, newMax, "Calib");
                break;
            }
        }
    };
    processCommand(action);
}

void onMasterVolumeCommand(HANumeric number, HANumber* sender) {
    if (!number.isSet()) return;
    auto action = [&]() {
        currentMasterVolume = number.toFloat();
        sender->setState(number);
        preferences.begin("niles-ctrl", false);
        preferences.putFloat("master_vol", currentMasterVolume);
        preferences.end();
        Serial.printf("Master Volume set to %.1f. Updating all servos.\n", currentMasterVolume);
        updateAllServoTargets();
    };
    processCommand(action);
}

// =====================================================================================================================
// --- Helper Functions ---
// =====================================================================================================================

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void computeStereoAngles(const StereoZone& zone, float& outLeftAngle, float& outRightAngle) {
    float leftVol = zone.currentVolume;
    float rightVol = zone.currentVolume;
    if (zone.currentBalance > 0) { leftVol *= mapf(zone.currentBalance, 0, HA_BALANCE_MAX, 1.0f, 0.0f); }
    else if (zone.currentBalance < 0) { rightVol *= mapf(zone.currentBalance, 0, HA_BALANCE_MIN, 1.0f, 0.0f); }
    float masterMultiplier = currentMasterVolume / HA_VOLUME_MAX;
    leftVol *= masterMultiplier;
    rightVol *= masterMultiplier;
    leftVol = constrain(leftVol, HA_VOLUME_MIN, HA_VOLUME_MAX);
    rightVol = constrain(rightVol, HA_VOLUME_MIN, HA_VOLUME_MAX);
    outLeftAngle = mapf(leftVol, HA_VOLUME_MIN, HA_VOLUME_MAX, zone.minAngleLeft, zone.maxAngleLeft);
    outRightAngle = mapf(rightVol, HA_VOLUME_MIN, HA_VOLUME_MAX, zone.minAngleRight, zone.maxAngleRight);
}

float computeMonoAngle(const MonoZone& zone) {
    float masterMultiplier = currentMasterVolume / HA_VOLUME_MAX;
    float effectiveVolume = zone.currentVolume * masterMultiplier;
    effectiveVolume = constrain(effectiveVolume, HA_VOLUME_MIN, HA_VOLUME_MAX);
    return mapf(effectiveVolume, HA_VOLUME_MIN, HA_VOLUME_MAX, zone.minAngle, zone.maxAngle);
}

void updateAllServoTargets() {
    for (int i = 0; i < numStereoZones; i++) {
        updateStereoPairServos(stereoZones[i]);
    }
    for (int i = 0; i < numMonoZones; i++) {
        updateMonoServo(monoZones[i]);
    }
}

void updateStereoPairServos(StereoZone& zone) {
    float targetAngleLeft, targetAngleRight;
    computeStereoAngles(zone, targetAngleLeft, targetAngleRight);
    Serial.printf("Updating '%s': Vol=%.1f, Bal=%.1f, Master=%.1f -> Angle L:%.1f R:%.1f\n",
                  zone.name, zone.currentVolume, zone.currentBalance, currentMasterVolume, targetAngleLeft, targetAngleRight);
    moveServoToAngle(zone.leftServo, targetAngleLeft, "Left", zone.lastAngleLeft);
    moveServoToAngle(zone.rightServo, targetAngleRight, "Right", zone.lastAngleRight);
}

void updateMonoServo(MonoZone& zone) {
    float targetAngle = computeMonoAngle(zone);
    Serial.printf("Updating mono '%s': Vol=%.1f -> Angle=%.1f\n", zone.name, zone.currentVolume, targetAngle);
    moveServoToAngle(zone.servo, targetAngle, zone.name, zone.lastAngle);
}

void moveServoToAngle(ServoEasing& servo, float targetAngle, const char* name, float& lastAngle) {
    if (startupPhase != PHASE_RUNNING) return;

    targetAngle = constrain(targetAngle, HA_CALIBRATION_ANGLE_MIN, HA_CALIBRATION_ANGLE_MAX);

    // De-duplication: skip if target is effectively the same as last written angle
    if (lastAngle >= 0.0f && fabsf(targetAngle - lastAngle) < ANGLE_DEDUP_THRESHOLD) {
        Serial.printf("  > Skipping %s servo: angle %.1f unchanged\n", name, targetAngle);
        return;
    }

    if (servosAreSleeping) {
        wakeUpServos();
    }

    taskENTER_CRITICAL(&commandMutex);
    lastActivityTime = millis();
    taskEXIT_CRITICAL(&commandMutex);

    Serial.printf("  > Moving %s servo to angle %.1f\n", name, targetAngle);
    servo.startEaseTo(targetAngle, SERVO_SPEED_DPS, START_UPDATE_BY_INTERRUPT);
    lastAngle = targetAngle;
}

// Overload without de-duplication (used for calibration moves)
void moveServoToAngle(ServoEasing& servo, float targetAngle, const char* name) {
    float dummy = -1.0f;
    moveServoToAngle(servo, targetAngle, name, dummy);
}

void saveVolumesToNVS() {
    preferences.begin("niles-ctrl", false);
    for (int i = 0; i < numStereoZones; i++) {
        char volKey[32], balKey[32];
        snprintf(volKey, sizeof(volKey), "%s_vol", stereoZones[i].id);
        snprintf(balKey, sizeof(balKey), "%s_bal", stereoZones[i].id);
        preferences.putFloat(volKey, stereoZones[i].currentVolume);
        preferences.putFloat(balKey, stereoZones[i].currentBalance);
    }
    for (int i = 0; i < numMonoZones; i++) {
        char volKey[32];
        snprintf(volKey, sizeof(volKey), "%s_vol", monoZones[i].id);
        preferences.putFloat(volKey, monoZones[i].currentVolume);
    }
    preferences.end();
    Serial.println(F("Volume states saved to NVS."));
}

void setupHaNumber(HANumber& number, const char* name, const char* icon, float minVal, float maxVal, float step) {
    number.setName(name);
    number.setIcon(icon);
    number.setMin(minVal);
    number.setMax(maxVal);
    number.setStep(step);
    number.setMode(HANumber::ModeSlider);
    number.setRetain(true);
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

    esp_wifi_set_ps(WIFI_PS_NONE);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    Serial.println(F("WiFi power saving disabled, TX power set to max."));
}

void wakeUpServos() {
    taskENTER_CRITICAL(&commandMutex);
    if (!servosAreSleeping) {
        taskEXIT_CRITICAL(&commandMutex);
        return;
    }
    servosAreSleeping = false;
    taskEXIT_CRITICAL(&commandMutex);

    Serial.println(F("Waking up servos..."));

    // Attach each servo at its last-known angle to prevent snapping to 0
    auto attachAndConfigure = [](ServoEasing& servo, uint8_t channel, int initialAngle, const char* name) {
        initialAngle = constrain(initialAngle, (int)HA_CALIBRATION_ANGLE_MIN, (int)HA_CALIBRATION_ANGLE_MAX);
        if (servo.attach(channel, initialAngle, 500, 2500, 0, 270) == INVALID_SERVO) {
            Serial.printf("ERROR attaching servo %s\n", name);
        }
        servo.setSpeed(SERVO_SPEED_DPS);
        servo.setEasingType(EASE_QUADRATIC_IN_OUT);
    };

    attachAndConfigure(ServoSalaLeft, SERVO_SALA_LEFT_CH, (int)stereoZones[0].lastAngleLeft, "Sala Left");
    attachAndConfigure(ServoSalaRight, SERVO_SALA_RIGHT_CH, (int)stereoZones[0].lastAngleRight, "Sala Right");
    attachAndConfigure(ServoCinemaLeft, SERVO_CINEMA_LEFT_CH, (int)stereoZones[1].lastAngleLeft, "Cinema Left");
    attachAndConfigure(ServoCinemaRight, SERVO_CINEMA_RIGHT_CH, (int)stereoZones[1].lastAngleRight, "Cinema Right");
    attachAndConfigure(ServoVarandaLeft, SERVO_VARANDA_LEFT_CH, (int)stereoZones[2].lastAngleLeft, "Varanda Left");
    attachAndConfigure(ServoVarandaRight, SERVO_VARANDA_RIGHT_CH, (int)stereoZones[2].lastAngleRight, "Varanda Right");
    attachAndConfigure(ServoCozinhaRight, SERVO_COZINHA_RIGHT_CH, (int)monoZones[0].lastAngle, "Cozinha Right");

    Serial.println(F("Servos are awake."));
}

void sleepServos() {
    taskENTER_CRITICAL(&commandMutex);
    if (servosAreSleeping) {
        taskEXIT_CRITICAL(&commandMutex);
        return;
    }
    servosAreSleeping = true;
    taskEXIT_CRITICAL(&commandMutex);

    Serial.println(F("Inactivity detected. Sleeping servos..."));
    ServoSalaRight.detach();
    ServoSalaLeft.detach();
    ServoCinemaRight.detach();
    ServoCinemaLeft.detach();
    ServoVarandaRight.detach();
    ServoVarandaLeft.detach();
    ServoCozinhaRight.detach();
    saveVolumesToNVS();
    Serial.println(F("Servos are asleep."));
}

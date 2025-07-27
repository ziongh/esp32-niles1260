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

// Network
const char* WIFI_SSID = FACTORY_WIFI_SSID;
const char* WIFI_PASSWORD = FACTORY_WIFI_PASSWORD;
const char* MQTT_HOST = FACTORY_MQTT_HOST;
const uint16_t MQTT_PORT = FACTORY_MQTT_PORT;
const char* MQTT_USER = FACTORY_MQTT_USERNAME;
const char* MQTT_PASSWORD = FACTORY_MQTT_PASSWORD;

// Timing
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 20000;
const unsigned long INITIAL_SYNC_PERIOD_MS = 2000;
const unsigned long SERVO_SLEEP_TIMEOUT_MS = 60000;

// Servo Configuration
const int SERVO_SPEED_DPS = 60;
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

// Struct to hold all data for a single stereo zone, with calibration
struct StereoZone {
    const char* id;
    const char* name;
    ServoEasing& leftServo;
    ServoEasing& rightServo;
    HANumber& volumeEntity;
    HANumber& balanceEntity;
    HANumber& minAngleEntity;
    HANumber& maxAngleEntity;
    float currentVolume;
    float currentBalance;
    float minAngle;
    float maxAngle;
    char balanceName[32];
    char minAngleName[32];
    char maxAngleName[32];
};

// Home Assistant Entities
HANumber volumeSala("volumeSala"), balanceSala("balanceSala"), minAngleSala("minAngleSala"), maxAngleSala("maxAngleSala");
HANumber volumeCinema("volumeCinema"), balanceCinema("balanceCinema"), minAngleCinema("minAngleCinema"), maxAngleCinema("maxAngleCinema");
HANumber volumeVaranda("volumeVaranda"), balanceVaranda("balanceVaranda"), minAngleVaranda("minAngleVaranda"), maxAngleVaranda("maxAngleVaranda");
HANumber volumeCozinha("volumeCozinha");
HANumber masterVolume("masterVolume");

// Array of zones now includes calibration entities and default values
StereoZone zones[] = {
    { "sala", "Sala", ServoSalaLeft, ServoSalaRight, volumeSala, balanceSala, minAngleSala, maxAngleSala, 0.0, 0.0, 0.0, 220.0 },
    { "cinema", "Cinema", ServoCinemaLeft, ServoCinemaRight, volumeCinema, balanceCinema, minAngleCinema, maxAngleCinema, 0.0, 0.0, 0.0, 220.0 },
    { "varanda", "Varanda", ServoVarandaLeft, ServoVarandaRight, volumeVaranda, balanceVaranda, minAngleVaranda, maxAngleVaranda, 0.0, 0.0, 0.0, 220.0 }
};
const int numStereoZones = sizeof(zones) / sizeof(zones[0]);

// State for other entities
float currentCozinhaVolume = 0.0;
float lastMasterVolume = 100.0;

// System State Variables
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

// Callbacks
void unifiedVolumeCallback(HANumeric number, HANumber* sender);
void unifiedBalanceCallback(HANumeric number, HANumber* sender);
void unifiedMinAngleCallback(HANumeric number, HANumber* sender);
void unifiedMaxAngleCallback(HANumeric number, HANumber* sender);
void onVolumeCozinhaCommand(HANumeric number, HANumber* sender);
void onMasterVolumeCommand(HANumeric number, HANumber* sender);


// =====================================================================================================================
// --- Main Setup and Loop ---
// =====================================================================================================================

void setup() {
    Serial.begin(115200);
    Serial.println(F("\n\nStarting Niles Volume Controller v5.4.12 (Retain Fix)..."));

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
    device.setSoftwareVersion("5.4.12");
    device.enableSharedAvailability();
    device.enableLastWill();

    preferences.begin("niles-ctrl", false);

    for (int i = 0; i < numStereoZones; i++) {
        char minKey[16], maxKey[16];
        snprintf(minKey, sizeof(minKey), "%s_min_angle", zones[i].id);
        snprintf(maxKey, sizeof(maxKey), "%s_max_angle", zones[i].id);
        zones[i].minAngle = preferences.getFloat(minKey, zones[i].minAngle);
        zones[i].maxAngle = preferences.getFloat(maxKey, zones[i].maxAngle);
    }
    
    for (int i = 0; i < numStereoZones; i++) {
        snprintf(zones[i].balanceName, sizeof(zones[i].balanceName), "Balance %s", zones[i].name);
        snprintf(zones[i].minAngleName, sizeof(zones[i].minAngleName), "Min Angle %s", zones[i].name);
        snprintf(zones[i].maxAngleName, sizeof(zones[i].maxAngleName), "Max Angle %s", zones[i].name);

        setupHaNumber(zones[i].volumeEntity, zones[i].name, "mdi:volume-high", HA_VOLUME_MIN, HA_VOLUME_MAX, 1);
        zones[i].volumeEntity.onCommand(unifiedVolumeCallback);

        setupHaNumber(zones[i].balanceEntity, zones[i].balanceName, "mdi:speaker-multiple", HA_BALANCE_MIN, HA_BALANCE_MAX, 1);
        zones[i].balanceEntity.onCommand(unifiedBalanceCallback);
        
        setupHaNumber(zones[i].minAngleEntity, zones[i].minAngleName, "mdi:page-layout-header-footer", HA_CALIBRATION_ANGLE_MIN, HA_CALIBRATION_ANGLE_MAX, 1.0);
        zones[i].minAngleEntity.onCommand(unifiedMinAngleCallback);

        setupHaNumber(zones[i].maxAngleEntity, zones[i].maxAngleName, "mdi:page-layout-header-footer", HA_CALIBRATION_ANGLE_MIN, HA_CALIBRATION_ANGLE_MAX, 1.0);
        zones[i].maxAngleEntity.onCommand(unifiedMaxAngleCallback);
    }

    setupHaNumber(volumeCozinha, "Volume Cozinha", "mdi:silverware-fork-knife", HA_VOLUME_MIN, HA_VOLUME_MAX, 1);
    volumeCozinha.onCommand(onVolumeCozinhaCommand);

    setupHaNumber(masterVolume, "Master Volume", "mdi:volume-vibrate", HA_VOLUME_MIN, HA_VOLUME_MAX, 1);
    masterVolume.onCommand(onMasterVolumeCommand);

    Serial.println(F("Connecting to MQTT broker..."));
    mqtt.begin(MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASSWORD);
}

void loop() {
    mqtt.loop();
    
    if (mqtt.isConnected() && !reannouncementTriggered) {
        Serial.println("Connection stable. Forcing full discovery re-announcement...");
        mqtt.publish("homeassistant/status", "online", true); // Use retain=true here for HA's benefit
        Serial.println("Re-announcement message sent to homeassistant/status.");
        reannouncementTriggered = true; 
    }

    if (reannouncementTriggered && !statesPublished) {
        Serial.println("Publishing initial states...");
        
        for (int i = 0; i < numStereoZones; i++) {
            zones[i].minAngleEntity.setState(zones[i].minAngle);
            zones[i].maxAngleEntity.setState(zones[i].maxAngle);
            zones[i].volumeEntity.setState(zones[i].currentVolume);
            zones[i].balanceEntity.setState(zones[i].currentBalance);
        }
        masterVolume.setState(lastMasterVolume);
        volumeCozinha.setState(currentCozinhaVolume);

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
            lastActivityTime = millis();
        }
    }

    if (initialSyncComplete && !servosAreSleeping && (millis() - lastActivityTime > SERVO_SLEEP_TIMEOUT_MS)) {
        sleepServos();
    }
}


// =====================================================================================================================
// --- Home Assistant Callback Functions ---
// =====================================================================================================================

void unifiedVolumeCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet()) return;
    for (int i = 0; i < numStereoZones; i++) {
        if (sender == &zones[i].volumeEntity) {
            zones[i].currentVolume = number.toFloat();
            sender->setState(number);
            if (initialSyncComplete) {
                updateStereoPairServos(zones[i]);
                lastActivityTime = millis();
            }
            break;
        }
    }
}

void unifiedBalanceCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet()) return;
    for (int i = 0; i < numStereoZones; i++) {
        if (sender == &zones[i].balanceEntity) {
            zones[i].currentBalance = number.toFloat();
            sender->setState(number); 
            if (initialSyncComplete) {
                updateStereoPairServos(zones[i]);
                lastActivityTime = millis();
            }
            break;
        }
    }
}

void unifiedMinAngleCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet()) return;
    for (int i = 0; i < numStereoZones; i++) {
        if (sender == &zones[i].minAngleEntity) {
            float newMin = number.toFloat();
            zones[i].minAngle = newMin;
            sender->setState(number);
            
            char minKey[16];
            snprintf(minKey, sizeof(minKey), "%s_min_angle", zones[i].id);
            preferences.putFloat(minKey, newMin);
            Serial.printf("Saved new min angle for %s: %.1f\n", zones[i].name, newMin);

            moveServoToAngle(zones[i].leftServo, newMin, "Left Calib");
            moveServoToAngle(zones[i].rightServo, newMin, "Right Calib");
            lastActivityTime = millis();
            break;
        }
    }
}

void unifiedMaxAngleCallback(HANumeric number, HANumber* sender) {
    if (!number.isSet()) return;
    for (int i = 0; i < numStereoZones; i++) {
        if (sender == &zones[i].maxAngleEntity) {
            float newMax = number.toFloat();
            zones[i].maxAngle = newMax;
            sender->setState(number);

            char maxKey[16];
            snprintf(maxKey, sizeof(maxKey), "%s_max_angle", zones[i].id);
            preferences.putFloat(maxKey, newMax);
            Serial.printf("Saved new max angle for %s: %.1f\n", zones[i].name, newMax);
            
            moveServoToAngle(zones[i].leftServo, newMax, "Left Calib");
            moveServoToAngle(zones[i].rightServo, newMax, "Right Calib");
            lastActivityTime = millis();
            break;
        }
    }
}

void onMasterVolumeCommand(HANumeric number, HANumber* sender) {
    if (!number.isSet() || !initialSyncComplete) return;

    float newMaster = number.toFloat();
    if (newMaster < 0) newMaster = 0;

    float ratio = (lastMasterVolume > 0.01) ? (newMaster / lastMasterVolume) : 1.0;
    
    Serial.printf("Master Volume changed to %.1f (Ratio: %.2f)\n", newMaster, ratio);

    for (int i = 0; i < numStereoZones; i++) {
        zones[i].currentVolume *= ratio;
        zones[i].currentVolume = constrain(zones[i].currentVolume, HA_VOLUME_MIN, HA_VOLUME_MAX);
        zones[i].volumeEntity.setState(zones[i].currentVolume);
        updateStereoPairServos(zones[i]);
    }
    
    currentCozinhaVolume *= ratio;
    currentCozinhaVolume = constrain(currentCozinhaVolume, HA_VOLUME_MIN, HA_VOLUME_MAX);
    volumeCozinha.setState(currentCozinhaVolume);
    float cozAngle = map(currentCozinhaVolume, HA_VOLUME_MIN, HA_VOLUME_MAX, 0.0, 220.0);
    moveServoToAngle(ServoCozinhaRight, cozAngle, "Cozinha");

    lastMasterVolume = newMaster;
    sender->setState(newMaster);
    lastActivityTime = millis();
}

void onVolumeCozinhaCommand(HANumeric number, HANumber* sender) {
    if (!number.isSet()) { return; }
    currentCozinhaVolume = number.toFloat();
    sender->setState(number);
    if (!initialSyncComplete) { return; }
    
    float targetAngle = map(currentCozinhaVolume, HA_VOLUME_MIN, HA_VOLUME_MAX, 0.0, 220.0);
    moveServoToAngle(ServoCozinhaRight, targetAngle, "Cozinha");
    lastActivityTime = millis();
}


// =====================================================================================================================
// --- Helper Functions ---
// =====================================================================================================================

void updateStereoPairServos(StereoZone& zone) {
    Serial.printf("Updating pair '%s': Vol=%.1f, Bal=%.1f, MinAng=%.1f, MaxAng%.1f\n",
                  zone.name, zone.currentVolume, zone.currentBalance, zone.minAngle, zone.maxAngle);

    float baseAngle = map(zone.currentVolume, HA_VOLUME_MIN, HA_VOLUME_MAX, zone.minAngle, zone.maxAngle);

    float leftMultiplier = 1.0, rightMultiplier = 1.0;
    if (zone.currentBalance > 0) {
        leftMultiplier = map(zone.currentBalance, 0, HA_BALANCE_MAX, 1.0, 0.0);
    } else if (zone.currentBalance < 0) {
        rightMultiplier = map(zone.currentBalance, 0, HA_BALANCE_MIN, 1.0, 0.0);
    }

    moveServoToAngle(zone.leftServo, baseAngle * leftMultiplier, "Left");
    delay(50);
    moveServoToAngle(zone.rightServo, baseAngle * rightMultiplier, "Right");
}


void moveServoToAngle(ServoEasing& servo, float targetAngle, const char* name) {
    if (servosAreSleeping) {
        wakeUpServos();
        delay(50); 
    }

    targetAngle = constrain(targetAngle, HA_CALIBRATION_ANGLE_MIN, HA_CALIBRATION_ANGLE_MAX);

    Serial.printf("  > Moving %s servo to angle %.1f\n", name, targetAngle);
    servo.startEaseTo(targetAngle, SERVO_SPEED_DPS, START_UPDATE_BY_INTERRUPT);
}

// FIX: Change setRetain to false to prevent commands from being retained.
void setupHaNumber(HANumber& number, const char* name, const char* icon, float minVal, float maxVal, float step) {
    number.setName(name);
    number.setIcon(icon);
    number.setMin(minVal);
    number.setMax(maxVal);
    number.setStep(step);
    number.setMode(HANumber::ModeSlider);
    number.setRetain(false); // This is the key change
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
    
    servosAreSleeping = false;
    Serial.println(F("Servos are awake."));
}

void sleepServos() {
    if (servosAreSleeping) return;
    Serial.println(F("Inactivity detected. Sleeping servos..."));
    ServoSalaRight.detach();
    ServoSalaLeft.detach();
    ServoCinemaRight.detach();
    ServoCinemaLeft.detach();
    ServoVarandaRight.detach();
    ServoVarandaLeft.detach();
    ServoCozinhaRight.detach();
    
    servosAreSleeping = true;
}
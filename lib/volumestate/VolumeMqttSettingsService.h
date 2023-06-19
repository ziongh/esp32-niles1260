#ifndef VolumeMqttSettingsService_h
#define VolumeMqttSettingsService_h

#include <HttpEndpoint.h>
#include <FSPersistence.h>
#include <SettingValue.h>

#define Volume_BROKER_SETTINGS_FILE "/config/brokerSettings.json"
#define Volume_BROKER_SETTINGS_PATH "/rest/brokerSettings"

class VolumeMqttSettings {
 public:
  String mqttPath;
  String name;
  String uniqueId;

  static void read(VolumeMqttSettings& settings, JsonObject& root) {
    root["mqtt_path"] = settings.mqttPath;
    root["name"] = settings.name;
    root["unique_id"] = settings.uniqueId;
  }

  static StateUpdateResult update(JsonObject& root, VolumeMqttSettings& settings) {
    settings.mqttPath = root["mqtt_path"] | SettingValue::format("homeassistant/volume/#{unique_id}");
    settings.name = root["name"] | SettingValue::format("volume-#{unique_id}");
    settings.uniqueId = root["unique_id"] | SettingValue::format("volume-#{unique_id}");
    return StateUpdateResult::CHANGED;
  }
};

class VolumeMqttSettingsService : public StatefulService<VolumeMqttSettings> {
 public:
  VolumeMqttSettingsService(AsyncWebServer* server, FS* fs, SecurityManager* securityManager);
  void begin();

 private:
  HttpEndpoint<VolumeMqttSettings> _httpEndpoint;
  FSPersistence<VolumeMqttSettings> _fsPersistence;
};

#endif  // end VolumeMqttSettingsService_h

#include <VolumeMqttSettingsService.h>

VolumeMqttSettingsService::VolumeMqttSettingsService(AsyncWebServer* server, FS* fs, SecurityManager* securityManager) :
    _httpEndpoint(VolumeMqttSettings::read,
                  VolumeMqttSettings::update,
                  this,
                  server,
                  Volume_BROKER_SETTINGS_PATH,
                  securityManager,
                  AuthenticationPredicates::IS_AUTHENTICATED),
    _fsPersistence(VolumeMqttSettings::read, VolumeMqttSettings::update, this, fs, Volume_BROKER_SETTINGS_FILE) {
}

void VolumeMqttSettingsService::begin() {
  _fsPersistence.readFromFS();
}

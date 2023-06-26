#include <ESP8266React.h>


#include <VolumeMqttSettingsService.h>
#include <VolumeStateService.h>

#define SERIAL_BAUD_RATE 115200

AsyncWebServer server(80);
ESP8266React esp8266React(&server);
                                                        
VolumeMqttSettingsService volumeMqttSettingsService =
    VolumeMqttSettingsService(&server, esp8266React.getFS(), esp8266React.getSecurityManager());
VolumeStateService volumeStateService = VolumeStateService(&server,
                                                        esp8266React.getSecurityManager(),
                                                        esp8266React.getMqttClient(),
                                                        &volumeMqttSettingsService);

void setup() {
  // start serial and filesystem
  Serial.begin(SERIAL_BAUD_RATE);

  // start the framework and demo project
  esp8266React.begin();

  // load the initial volume settings
  volumeStateService.begin();

  // start the volume service
  volumeMqttSettingsService.begin();

  // start the server
  server.begin();
}

void loop() {
  // run the framework's loop function
  esp8266React.loop();
}

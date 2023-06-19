export interface LightState {
  led_on: boolean;
}

export interface LightMqttSettings {
  unique_id: string;
  name: string;
  mqtt_path: string;
}

export interface VolumeState {
  volume: number;
}

export interface VolumeMqttSettings {
  unique_id: string;
  name: string;
  mqtt_path: string;
}

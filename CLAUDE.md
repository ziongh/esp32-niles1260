# Niles Volume Controller

ESP32 firmware that controls a Niles audio amplifier via PCA9685 servo driver, integrated with Home Assistant through MQTT auto-discovery.

## Project Structure

```
src/main.cpp          - All firmware logic (single-file architecture)
platformio.ini        - PlatformIO config, board definitions, library deps
factory_settings.ini  - WiFi, MQTT, OTA credentials (compile-time defines)
features.ini          - Feature flags (MQTT, NTP, OTA, etc.)
.github/workflows/    - CI: builds node32s + denky32 environments
```

## Build

```bash
# Default target (denky32)
pio run

# Specific environment
pio run -e denky32
pio run -e node32s

# Upload
pio run -e denky32 -t upload
```

The default board is `denky32`. All environments use `espressif32` platform with Arduino framework. Upload port is `COM3`.

**Build flag**: `ARDUINOHA_MAX_ENTITIES=24` in platformio.ini — must be >= 22 (the actual entity count). The ArduinoHA library silently drops entities beyond this limit.

## Hardware

- **ESP32** (denky32 board) connected to a **PCA9685** I2C servo expander at the default address
- **7 servo channels** driving volume/balance knobs on a Niles amplifier:

| Channel | Servo Variable     | Zone    | Side  |
|---------|--------------------|---------|-------|
| 0       | ServoSalaRight     | Sala    | Right |
| 1       | ServoSalaLeft      | Sala    | Left  |
| 2       | ServoCinemaRight   | Cinema  | Right |
| 3       | ServoCinemaLeft    | Cinema  | Left  |
| 4       | ServoVarandaRight  | Varanda | Right |
| 5       | ServoVarandaLeft   | Varanda | Left  |
| 6       | ServoCozinhaRight  | Cozinha | Right |

Servo PWM range: 500-2500us, angle range: 0-270 degrees.

## Zone Name Mapping (IMPORTANT)

The firmware zone names do NOT match the physical speaker locations:

| Firmware Zone | HA Entity Name                  | Physical Speakers |
|---------------|---------------------------------|-------------------|
| `sala`        | `number.niles_controller_sala`    | Sala (living room) |
| `cinema`      | `number.niles_controller_cinema`  | **Varanda** (balcony) |
| `varanda`     | `number.niles_controller_varanda` | **Jantar** (dining room) |
| `cozinha`     | `number.niles_controller_cozinha` | Cozinha (kitchen) |

The `cinema` and `varanda` names are swapped relative to physical wiring. This is a legacy wiring decision — the firmware names cannot change without re-pairing all HA automations.

## Home Assistant Entities (22 total)

Each **stereo zone** (sala, cinema, varanda) exposes 6 entities:
- `volume{Zone}` — Volume slider (0-100)
- `balance{Zone}` — Balance slider (-100 to +100)
- `minAngle{Zone}Left/Right` — Calibration: minimum servo angle (0-270)
- `maxAngle{Zone}Right/Left` — Calibration: maximum servo angle (0-270)

The **mono zone** (cozinha) exposes 3 entities:
- `volumeCozinha` — Volume slider (0-100)
- `minAngleCozinha` — Calibration: minimum servo angle
- `maxAngleCozinha` — Calibration: maximum servo angle

Plus 1 global entity:
- `masterVolume` — Master volume multiplier (0-100), affects all zones

All entities use `setRetain(true)` so HA publishes retained MQTT commands.

## Architecture: Phase-Based Startup (v7.3.0)

The startup uses a state machine to safely recover servo positions after reboot:

```
PHASE_WAITING_MQTT  →  PHASE_SYNCING  →  PHASE_STARTUP_MOVE  →  PHASE_RUNNING
     (boot)           (MQTT connects)      (3s timer fires)      (normal ops)
```

### Why this exists

Servos have no position feedback. After reboot, the ESP32 doesn't know where the physical knobs are. The MQTT broker's retained values are the best source of truth for what the volume *should* be.

### Phase behavior

| Phase | MQTT Callbacks | Servo Movement | Purpose |
|-------|----------------|----------------|---------|
| WAITING_MQTT | Ignored | Sleeping | Waiting for MQTT connection |
| SYNCING | State updates only (no servo moves) | Sleeping | Absorbing retained MQTT messages for 3s |
| STARTUP_MOVE | — | Wake at NVS angle, ease to target | Single smooth positioning of all servos |
| RUNNING | Full processing with busy guard | Normal move/sleep cycle | Normal operation |

### Key design decisions

1. **`moveServoToAngle()` no-ops during SYNCING** — prevents servos from jumping to each retained value as it arrives
2. **6-param `servo.attach(channel, initialAngle, 500, 2500, 0, 270)`** — wakes servo at the NVS-stored angle instead of snapping to 0 degrees
3. **De-duplication**: skips servo movement if `|target - lastAngle| < 0.5` degrees — prevents feedback loops when HA echoes the same value back
4. **Reboot safety**: `RTC_DATA_ATTR int rebootCount` survives software resets. After 3 rapid reboots, enters safe mode (servos disabled). Counter clears after 60s of stable operation.

### MQTT reconnect behavior

On disconnect: phase resets to `PHASE_WAITING_MQTT`. On reconnect: full sync cycle runs again. If servos were already awake, they ease to new targets without re-attaching.

## NVS Persistence

Volumes and calibration angles are stored in NVS (namespace: `niles-ctrl`):

| Key Pattern | Example | Saved When |
|-------------|---------|------------|
| `{zone}_vol` | `sala_vol` | On servo sleep (inactivity timeout) |
| `{zone}_bal` | `sala_bal` | On servo sleep |
| `{zone}_min_angle_l` | `sala_min_angle_l` | Immediately on calibration change |
| `{zone}_max_angle_l` | `sala_max_angle_l` | Immediately on calibration change |
| `{zone}_min_angle_r` | `sala_min_angle_r` | Immediately on calibration change |
| `{zone}_max_angle_r` | `sala_max_angle_r` | Immediately on calibration change |
| `{zone}_min_angle` | `cozinha_min_angle` | Immediately (mono zones) |
| `{zone}_max_angle` | `cozinha_max_angle` | Immediately (mono zones) |
| `master_vol` | — | Immediately on change |

## Servo Sleep/Wake Cycle

- Servos sleep after **15 seconds** of inactivity (configurable: `SERVO_SLEEP_TIMEOUT_MS`)
- On sleep: all servos detach (PCA9685 stops driving PWM), volumes saved to NVS
- On wake: servos re-attach at their `lastAngle` position using the 6-param attach
- The `servosBusy` flag (protected by FreeRTOS critical section) prevents new commands while easing is in progress

## Known Pitfalls

- **Arduino `map()` returns `long`** — use the custom `mapf()` function for float-to-float mapping. Using `map()` with float ranges silently truncates to integers, breaking balance calculations.
- **Critical sections must not contain slow I/O** — `Serial.println()`, `servo.detach()`, and NVS writes must be done *outside* `taskENTER_CRITICAL` / `taskEXIT_CRITICAL` blocks.
- **ServoEasing `write()` is instant** — it bypasses easing and writes directly to PCA9685 via I2C. Use `startEaseTo()` for smooth movement.
- **After `detach()`, internal state is lost** — re-attaching requires providing the initial angle again.
- **PCA9685 resolution** — 4096 steps per 20ms frame, approximately 0.5 degree precision for a 270-degree range.

## Network Stability (v7.4.0)

The device runs 24/7. Several measures prevent MQTT/WiFi disconnections that cause HA entities to show NaN:

### Build flags

- **`MQTT_KEEPALIVE=60`** in `platformio.ini` — overrides PubSubClient's default 15s keepalive at compile time. The 15s default is too aggressive: any `loop()` hiccup longer than 15s causes the broker to consider the client dead. 60s gives ample headroom. This must be a compile-time `-D` flag because PubSubClient uses `#ifndef MQTT_KEEPALIVE`.

### WiFi hardening (`connectWiFi()` + `loop()`)

- **`esp_wifi_set_ps(WIFI_PS_NONE)`** — disables WiFi modem sleep. ESP32 modem sleep powers down the radio between AP beacons, causing packet loss over time on always-on devices. Re-applied on reconnect.
- **`WiFi.setTxPower(WIFI_POWER_19_5dBm)`** — max TX power for better signal margin.
- **Forced WiFi reconnection** — `WiFi.setAutoReconnect(true)` is unreliable for prolonged disconnections. After 30s without reconnection (`WIFI_FORCE_RECONNECT_MS`), the loop forces a `WiFi.disconnect()` + `WiFi.begin()` cycle. Does NOT call `ESP.restart()` to avoid triggering the safe mode reboot counter.

### TCP / main loop

- **`wifiClient.setNoDelay(true)`** — disables Nagle's algorithm so MQTT PINGREQ/PINGRESP packets go out immediately instead of being buffered up to 200ms.
- **`delay(1)` at end of `loop()`** — yields CPU to the FreeRTOS scheduler, allowing WiFi/TCP background tasks to process. Feeds the task watchdog. No impact on servo easing (runs via hardware timer interrupt).

## Security Note

`factory_settings.ini` contains WiFi, MQTT, and admin credentials as compile-time defines. This file should not be committed to public repositories.

---

## Home Assistant: Tuya Knob Automations

The system uses **Tuya TS004F** Zigbee rotary knobs (via ZHA integration) to control volume from each room. Each knob fires a `zha_event` with `command: step` when rotated.

### Knob-to-Zone Mapping

| Knob Name    | Device IEEE                  | Controls Entity                    | Physical Speakers |
|--------------|------------------------------|------------------------------------|-------------------|
| Knob_cozinha | `e0:79:8d:ff:fe:83:a6:1b`   | `number.niles_controller_cozinha`  | Cozinha           |
| Knob_estar   | `e0:79:8d:ff:fe:9c:09:bc`   | `number.niles_controller_sala`     | Sala              |
| Knob_jantar  | `e0:79:8d:ff:fe:b7:7e:2c`   | `number.niles_controller_varanda`  | Jantar            |
| Knob_varanda | `e0:79:8d:ff:fe:88:46:03`   | `number.niles_controller_cinema`   | Varanda           |

Note: `Knob_jantar` maps to entity `varanda` and `Knob_varanda` maps to entity `cinema` because of the firmware zone name mismatch described above.

### Automation Template

Each knob automation follows this pattern:

```yaml
alias: auto_knob_<room>
description: "Tuya knob ZHA event controls Niles volume"
mode: single
triggers:
  - trigger: event
    event_type: zha_event
    event_data:
      device_ieee: "<KNOB_IEEE_ADDRESS>"
      endpoint_id: 1
      command: step
actions:
  - action: number.set_value
    target:
      entity_id: number.niles_controller_<zone>
    data:
      value: >-
        {% set step_dir = 1 - 2 * trigger.event.data.params.step_mode %}
        {% set step_pct = (trigger.event.data.params.step_size * step_dir) / 8.0 %}
        {% set current = states('number.niles_controller_<zone>') | float(0) %}
        {{ [0, [100, (current + step_pct)] | min] | max | round(0) }}
```

### How the template works

The Tuya TS004F sends ZHA events with:
- `step_mode`: 0 = clockwise (volume up), 1 = counter-clockwise (volume down)
- `step_size`: typically 26 per detent

The formula:
1. `step_dir = 1 - 2 * step_mode` → converts to +1 (up) or -1 (down)
2. `step_pct = step_size * step_dir / 8.0` → ~3.25% per click
3. Clamps result to 0-100 range using `[0, [100, value] | min] | max`
4. Rounds to integer with `round(0)`

### Common mistakes to avoid

1. **Entity ID mismatch between read and write** — the `states('...')` call and the `target.entity_id` MUST reference the same entity. Using a wrong entity ID for `states()` causes `float(0)` to return 0, making volume always jump to just the step delta (~3).
2. **Using `mode: queued`** — causes event stacking. Use `mode: single` to drop events that arrive while processing.
3. **Using `|int` instead of `|round(0)`** — `int` truncates (always rounds down), `round(0)` rounds to nearest.

### Creating automations via the HA REST API

Automations can be pushed programmatically using the config endpoint:

```
POST http://<HA_HOST>:8123/api/config/automation/config/<AUTOMATION_ID>
Authorization: Bearer <LONG_LIVED_TOKEN>
Content-Type: application/json

{
  "id": "<AUTOMATION_ID>",
  "alias": "auto_knob_<room>",
  "description": "Tuya knob ZHA event controls Niles volume",
  "triggers": [{
    "trigger": "event",
    "event_type": "zha_event",
    "event_data": {
      "device_ieee": "<IEEE>",
      "endpoint_id": 1,
      "command": "step"
    }
  }],
  "conditions": [],
  "actions": [{
    "action": "number.set_value",
    "target": { "entity_id": "number.niles_controller_<zone>" },
    "data": { "value": "<JINJA_TEMPLATE>" }
  }],
  "mode": "single"
}
```

After pushing, reload automations:
```
POST http://<HA_HOST>:8123/api/services/automation/reload
```

### Finding knob IEEE addresses

Use the HA template API to look up device identifiers:

```
POST http://<HA_HOST>:8123/api/template
Content-Type: application/json

{
  "template": "{% for eid in ['sensor.knob_jantar_bateria', 'sensor.knob_estar_bateria', 'sensor.knob_cozinha_bateria', 'sensor.knob_varanda_bateria'] %}{% set did = device_id(eid) %}{{ eid }}: {{ device_attr(did, 'identifiers') }}\n{% endfor %}"
}
```

This returns the ZHA IEEE address for each knob device.

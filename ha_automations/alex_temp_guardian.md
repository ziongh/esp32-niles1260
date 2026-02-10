# Alex Room Temperature Guardian

**Automation ID:** `auto_alex_temp_guardian`
**HA Entity:** `automation.auto_alex_temp_guardian`

Safety-net automation that keeps Alex's bedroom near 22°C whenever the AC is running. Prevents the room from getting too cold or too hot when someone sets an aggressive temperature and forgets.

## Entities

| Entity | Role |
|--------|------|
| `climate.150633093086542_climate` | Alex AC (heat/cool/dry/fan/auto) |
| `sensor.temp_alex_temperatura` | Zigbee bed sensor (primary) |
| `sensor.150633093086542_indoor_temperature` | AC ceiling sensor (fallback, reads ~0.5°C warm) |
| `switch.150633093086542_prompt_tone` | AC beep — must stay off |
| `switch.150633093086542_screen_display` | AC display — must stay off |
| `switch.150633093086542_screen_display_alternate` | AC alt display — must stay off |
| `number.150633093086542_fan_speed` | Fan speed percent (target: 50) |

## Triggers

| # | Trigger | Threshold | Duration | ID |
|---|---------|-----------|----------|----|
| 1 | Bed sensor too cold | < 21°C | 5 min | `bed_cold` |
| 2 | Bed sensor too hot | > 23°C | 5 min | `bed_hot` |
| 3 | AC sensor too cold (fallback) | < 21.5°C | 10 min | `ac_cold` |
| 4 | AC sensor too hot (fallback) | > 23.5°C | 10 min | `ac_hot` |
| 5 | Heartbeat | every 10 min | — | `heartbeat` |

## Global Condition

AC must be ON (not `off`). The automation never turns on the AC — only adjusts an already-running one.

## Action Flow

1. **Compute variables**: determine if Zigbee sensor is fresh (updated within 30 min), pick effective temp, decide `heat`/`cool`/`none`
2. **Stop** if `needed_mode == "none"` (temp is in comfort range)
3. **Stop** if AC already at correct mode + 22°C target (avoid redundant commands)
4. **Pre-silence**: turn off `prompt_tone`, `screen_display`, `screen_display_alternate` → 2s delay
5. **Set HVAC mode** to `cool` or `heat` → 1s delay
6. **Set target temperature** to 22°C
7. **Set fan speed** to 50%  → 2s delay
8. **Post-silence**: turn off beep/display again (mode changes can re-enable them)
9. **Persistent notification** (replaces previous via `notification_id: alex_temp_guardian`)

## Design Decisions

- **`cool`/`heat` instead of `auto`**: AC is noisy in auto mode and doesn't allow fan speed control
- **Fan speed 50%**: preferred sleeping fan speed, set via `number.set_value` for precise control
- **Pre/post silencing**: mode changes can re-enable beep and display on this AC unit
- **Stale sensor detection**: if Zigbee sensor `last_updated` > 30 min ago, falls through to AC sensor
- **Both sensors unavailable**: `effective_temp` defaults to 22°C → `needed_mode = none` → no action
- **Comfort band**: ±1°C on bed sensor [21–23°C], ±1.5°C on AC fallback [21.5–23.5°C] (shifted +0.5°C for ceiling offset)

## JSON Payload

```json
{
  "id": "auto_alex_temp_guardian",
  "alias": "auto_alex_temp_guardian",
  "description": "Safety net: keeps Alex's bedroom near 22°C. Uses bed Zigbee sensor (primary) or AC indoor sensor (fallback). Sets cool/heat mode at 22°C with fan at 50%. Silences beep and display before/after commands.",
  "triggers": [
    {
      "trigger": "numeric_state",
      "entity_id": "sensor.temp_alex_temperatura",
      "below": 21.0,
      "for": "00:05:00",
      "id": "bed_cold"
    },
    {
      "trigger": "numeric_state",
      "entity_id": "sensor.temp_alex_temperatura",
      "above": 23.0,
      "for": "00:05:00",
      "id": "bed_hot"
    },
    {
      "trigger": "numeric_state",
      "entity_id": "sensor.150633093086542_indoor_temperature",
      "below": 21.5,
      "for": "00:10:00",
      "id": "ac_cold"
    },
    {
      "trigger": "numeric_state",
      "entity_id": "sensor.150633093086542_indoor_temperature",
      "above": 23.5,
      "for": "00:10:00",
      "id": "ac_hot"
    },
    {
      "trigger": "time_pattern",
      "minutes": "/10",
      "id": "heartbeat"
    }
  ],
  "conditions": [
    {
      "condition": "not",
      "conditions": [
        {
          "condition": "state",
          "entity_id": "climate.150633093086542_climate",
          "state": "off"
        }
      ]
    }
  ],
  "actions": [
    {
      "variables": {
        "zigbee_ok": "{{ states('sensor.temp_alex_temperatura') not in ['unavailable', 'unknown', 'none'] and (as_timestamp(now()) - as_timestamp(states.sensor.temp_alex_temperatura.last_updated)) < 1800 }}"
      }
    },
    {
      "variables": {
        "effective_temp": "{{ states('sensor.temp_alex_temperatura') | float(22) if zigbee_ok else (states('sensor.150633093086542_indoor_temperature') | float(22) if states('sensor.150633093086542_indoor_temperature') not in ['unavailable', 'unknown', 'none'] else 22) }}",
        "low_threshold": "{{ 21.0 if zigbee_ok else 21.5 }}",
        "high_threshold": "{{ 23.0 if zigbee_ok else 23.5 }}",
        "sensor_source": "{{ 'bed' if zigbee_ok else 'ac_fallback' }}"
      }
    },
    {
      "variables": {
        "needed_mode": "{% if effective_temp | float < low_threshold | float %}heat{% elif effective_temp | float > high_threshold | float %}cool{% else %}none{% endif %}"
      }
    },
    {
      "condition": "template",
      "value_template": "{{ needed_mode != 'none' }}"
    },
    {
      "condition": "template",
      "value_template": "{{ not (state_attr('climate.150633093086542_climate', 'temperature') == 22 and states('climate.150633093086542_climate') == needed_mode) }}"
    },
    {
      "alias": "Pre-command: silence beep and display",
      "action": "switch.turn_off",
      "target": {
        "entity_id": [
          "switch.150633093086542_prompt_tone",
          "switch.150633093086542_screen_display",
          "switch.150633093086542_screen_display_alternate"
        ]
      }
    },
    {
      "delay": "00:00:02"
    },
    {
      "alias": "Set HVAC mode (cool or heat based on direction)",
      "action": "climate.set_hvac_mode",
      "target": {
        "entity_id": "climate.150633093086542_climate"
      },
      "data": {
        "hvac_mode": "{{ needed_mode }}"
      }
    },
    {
      "delay": "00:00:01"
    },
    {
      "alias": "Set target temperature to 22°C",
      "action": "climate.set_temperature",
      "target": {
        "entity_id": "climate.150633093086542_climate"
      },
      "data": {
        "temperature": 22
      }
    },
    {
      "alias": "Set fan speed to 50%",
      "action": "number.set_value",
      "target": {
        "entity_id": "number.150633093086542_fan_speed"
      },
      "data": {
        "value": 50
      }
    },
    {
      "delay": "00:00:02"
    },
    {
      "alias": "Post-command: re-silence beep and display (mode change may re-enable)",
      "action": "switch.turn_off",
      "target": {
        "entity_id": [
          "switch.150633093086542_prompt_tone",
          "switch.150633093086542_screen_display",
          "switch.150633093086542_screen_display_alternate"
        ]
      }
    },
    {
      "alias": "Notify",
      "action": "persistent_notification.create",
      "data": {
        "title": "Alex Room Temp Guardian",
        "message": "{{ 'Bed sensor' if sensor_source == 'bed' else 'AC sensor (Zigbee unreliable)' }}: {{ effective_temp }}°C (outside {{ '21-23' if sensor_source == 'bed' else '21.5-23.5' }}°C range). AC set to {{ needed_mode }} at 22°C, fan 50%.{{ '' if sensor_source == 'bed' else ' Check Zigbee sensor.' }}",
        "notification_id": "alex_temp_guardian"
      }
    }
  ],
  "mode": "single"
}
```

## Re-deploying

```bash
# POST the JSON payload above to:
curl -X POST "http://192.168.53.10:8123/api/config/automation/config/auto_alex_temp_guardian" \
  -H "Authorization: Bearer <TOKEN>" \
  -H "Content-Type: application/json" \
  -d '<JSON>'

# Then reload:
curl -X POST "http://192.168.53.10:8123/api/services/automation/reload" \
  -H "Authorization: Bearer <TOKEN>"
```

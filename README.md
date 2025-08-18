# LD2415H Velocity Radar Sensor
## Overview
The ld2415h sensor platform allows you to use the Hi-Link HLK-LD2415H velocity radar sensor (datasheet and user manual) with ESPHome to track the velocity of objects.  The sensor utilizes millimeter wave radar to measure the velocity of objects within it's field of view.

## Specifications
 The HLK-LD2415H is a velocity radar sensor module has the following specifications:
 * K-band RF circuit operating in the 24.125GHz frequency range (customizable)
 * Accurately measures object velocities from 1-240KM/H
 * A precision of less than 1KM/H
 * Range up to 180 meters.
 * Antenna angle is 40° horizontal with a 16° pitch
 * Capable of 22 samples per second
 * RS-485 and TTL Serial interfaces
 * Site specific angle and sensitivity configuration


## Product Images
![Frontside of Sensor](ld2415h.front.png "Frontside of HLK-LD2415H Sensor")
![Backside of Sensor](ld2415h.back.png "Backside of HLK-LD2415H Sensor")

## Sensor Communication
The UART is required to be set up in your configuration for this sensor to work. Use of hardware UART pins is recommended, however the HLK-LD2415H sensor is limited to a 9600 baud rate.

### Serial Interface Specification
The sensor has both RS-485 and TTL serial interfaces with a pair of RX and TX pins for each interface.  Either interface can be used to read or configure the sensor.

### Serial Interface Command Syntax
Commands are comprised of 8, 11, or 13 bytes that are transmitted over either RX pin and responses can be received on either TX pin:

#### Command Format:
  - Length: 8, 11, or 13 bytes
  - 1st Byte: 0x43
  - 2nd Byte: 0x46
  - 3rd Byte: Command identifier
  - Remaining Bytes: Command specific parameters

Example:

    0x43 0x46 0x01 0x01 0x00 0x05 0x0d 0x0a

#### Supported Commands:
  - **0x01** : Set speed threshold, angular compensation, and sensitivity

    | Param | Default | Min  | Max  | Description |
    | ----- | --------| ---- | ---- | ----------- |
    | 1     | 0x01    | 0x01 | ???? | Represents the minimum speed reported by the sensor. |
    | 2     | 0x00    | 0x00 | ???? | Compensation angle between the center of the radar beam axis and the travel direction of objects. |
    | 3     | 0x05    | 0x01 | 0x0f | Radar sensitivity.  Smaller values have higher sensitivity and more interference.  Larger values are less sensitive and more resilient to interference. |

    Example:

        >>> 0x43 0x46 0x01 0x01 0x00 0x05 0x0d 0x0a

  - **0x02** : Set tracking mode, sample rate, and unit of measure

    | Param | Default | Min  | Max  | Description |
    | ----- | --------| ---- | ---- | ----------- |
    | 1     | 0x01    | 0x00 | 0x02 | Tracking Mode: <br>**0x00** : Approaching and retreating <br>**0x01** : Approaching <br>**0x02** : Retreating
    | 2     | 0x01    | 0x00 | ???? | The sample rate, higher values lower the rate. A value of 0 is ~22 samples per second, 1 is ~11. |
    | 3     | 0x00    | 0x00 | 0x02 | Unit of Measure: <br>**0x00** : kph <br>**0x01** : mph <br>**0x02** : mps |

    Example:

        >>> 0x43 0x46 0x02 0x01 0x01 0x00 0x0d 0x0a

  - **0x03** : Set anti-vibration compensation

    | Param | Default | Min  | Max  | Description |
    | ----- | --------| ---- | ---- | ----------- |
    | 1     | 0x05    | 0x00 | 0x70 | Vibration anti-interference coefficient used to reject false positives.  For example if the sensor is mounted to gently swaying pole. |
    | 2     | 0x00    | ???? | ???? | Undefined |
    | 3     | 0x00    | ???? | ???? | Undefined |

    Example:

        >>> 0x43 0x46 0x03 0x05 0x00 0x00 0x0d 0x0a

  - **0x04** : Set relay trigger duration and trigger speed threshold (only when using the photocoupler function)

    | Param | Default | Min  | Max  | Description |
    | ----- | --------| ---- | ---- | ----------- |
    | 1     | 0x03    | 0x00 | 0xff | Relay closure duration when speed exceeds threshold. |
    | 2     | 0x01    | 0x00 | 0xff | The lowest detection value for the photocoupler pickup in kph. |
    | 3     | 0x00    | ???? | ???? | Undefined |

    Example:

        >>> 0x43 0x46 0x04 0x03 0x01 0x00 0x0d 0x0a

  - **0x05** : Changes the Negotiation Mode (unknown purpose)

    | Param | Default | Min  | Max  | Description |
    | ----- | --------| ---- | ---- | ----------- |
    | 1     | 0x01    | 0x01 | 0x02 | Negotiation Mode: <br>**0x01** : Custom Agreement <br>**0x02** : Standard Protocol |
    | 2-8   |         |      |      | Mode 1:<br>    **0x43 0x46 0x05 0x01 0x00 0x00 0x00 0x00 0x00 0x00**<br> Mode 2 first call:<br> **0xfa 0x31 0x30 0x3d 0xfb**<br> Mode 2 second call:<br> **0xfa 0x55 0xaa 0xff 0xfb** |

    Example:

        >>> 0x43 0x46 0x05 0x01 0x00 0x00 0x00 0x00 0x00 0x00
        <<< Switch to CSR Mode... Done.


  - **0x07** : Read sensor configuration when in Standard Protocol mode.

    | Param | Default | Min  | Max  | Description |
    | ----- | --------| ---- | ---- | ----------- |
    | 1-10  |         |      |      | Pass 0x00 for all parameters, a response will be returned with the current configuration of the sensor. |

    Example:

        >>> 0x43 0x46 0x07 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00
        <<< xFFxFF\r\n
        <<< No.:20230801E v5.0\r\n
        <<< X1:01 X2:00 X3:05 X4:01 X5:00 X6:00 X7:05 X8:03 X9:01 X0:01\r\n

    Output parameters:
  
    | Param | Description |
    | ----- | ----------- |
    | X1    | Represents the minimum speed reported by the sensor. |
    | X2    | Compensation angle between the center of the radar beam axis and the travel direction of objects. |
    | X3    | Radar sensitivity. Smaller values have higher sensitivity and more interference. Larger values are less sensitive and more resilient to interference. |
    | X4    | Tracking Mode: <br>**0x00** : Approaching and retreating <br>**0x01** : Approaching <br>**0x02** : Retreating
    | X5    | The sample rate, higher values lower the rate. A value of 0 is ~22 samples per second, 1 is ~11. |
    | X6    | Unit of Measure: <br>**0x00** : kph <br>**0x01** : mph <br>**0x02** : mps |
    | X7    | Vibration anti-interference coefficient used to reject false positives.  For example if the sensor is mounted to gently swaying pole. |
    | X8    | Relay closure time when speed exceeds Param 9. |
    | X9    | The lowest detection value for the photocoupler pickup in kph. |
    | X0    | Negotiation Mode: <br>**0x01** : Custom Agreement <br>**0x02** : Standard Protocol |


## ESPHome ESP8266 Example Configuration

Example yaml to use in esphome device config:

```yaml
wifi:
  ssid: [REPLACE ME]
  password: [REPLACE ME]

esphome:
  name: esphome-web-xxxxxxx
  friendly_name: Radar
  min_version: 2024.11.0
  name_add_mac_suffix: false
  on_boot:
    priority: 600  # Run after WiFi, before user logic
    then: # Set the default configuration on boot here
      - number.set: { id: compensation_angle, value: 8 }
      - number.set: { id: sensitivity, value: 5 }
      - number.set: { id: min_speed_threshold, value: 20 }
      - number.set: { id: vibration_correction, value: 5 }
      - number.set: { id: relay_trigger_duration, value: 3 }
      - number.set: { id: relay_trigger_speed, value: 1 }
      - number.set: { id: timeout_duration, value: 400 }

esp8266:
  board: esp01_1m

# Enable logging
logger:

# Enable Home Assistant API
api:
  reboot_timeout: 0s

# Optional MQTT Support
#mqtt:
#  broker: [broker ip/domain]
#  username: [REPLACE ME]
#  password: [REPLACE ME]

# Allow Over-The-Air updates
ota:
- platform: esphome

web_server:
  port: 80

external_components:
  - source:
      url: https://github.com/dermodmaster/esphome.ld2415h
      type: git
      ref: main
    components: [ld2415h]
    refresh: 0s

uart:
  tx_pin: GPIO12
  rx_pin: GPIO14
  baud_rate: 9600
  
ld2415h:
  id: radar

# Define all sensors to be handled
sensor:
  - platform: ld2415h
    ld2415h_id: radar
    speed:
      name: "Speed"
      filters:
        - timeout:
            timeout: 0.1s
            value: 0
        - delta: 0.1
    approaching_speed:
      name: "Approaching Speed"
      filters:
        - timeout:
            timeout: 0.1s
            value: 0
        - delta: 0.1
    departing_speed:
      name: "Departing Speed"
      filters:
        - timeout:
            timeout: 0.1s
            value: 0
        - delta: 0.1
    approaching_last_max_speed: # fires max speed if no new measurement is incoming for x ms. (default: 400ms)
      name: "Approaching Last Max Speed"
    departing_last_max_speed:
      name: "Departing Last Max Speed"



# Numbers for all radar settings
number:
  - platform: ld2415h
    ld2415h_id: radar

    # Compensation Angle (0-90 degrees)
    compensation_angle:
      name: "Compensation Angle"
      id: compensation_angle

    # Sensitivity (0-15)
    sensitivity:
      name: "Sensitivity"
      id: sensitivity

    # Minimum Speed Threshold (1-60 km/h)
    min_speed_threshold:
      name: "Min Speed Threshold"
      id: min_speed_threshold

    # Vibration Correction (0-112)
    vibration_correction:
      name: "Vibration Correction"
      id: vibration_correction

    # Relay Trigger Duration (0–255 seconds)
    relay_trigger_duration:
      name: "Relay Trigger Duration"
      id: relay_trigger_duration

    # Relay Trigger Speed (0–255 km/h)
    relay_trigger_speed:
      name: "Relay Trigger Speed"
      id: relay_trigger_speed

    # Relay timeout for firing max speeed measurements if no new measurement was seen for x ms
    timeout_duration:
      name: "Radar Timeout (ms)"
      id: timeout_duration

# Settings with selections
select:
  - platform: ld2415h
    ld2415h_id: radar
    tracking_mode:
      name: "Tracking Mode"
    sample_rate:
      name: "Sample Rate"



```

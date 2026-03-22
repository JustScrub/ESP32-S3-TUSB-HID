# ESP32-S3 USB HID Keyboard with BLE Control

A USB HID keyboard device for ESP32-S3 that can be controlled via physical button or Bluetooth Low Energy (BLE). Based on the [ESP-IDF TUSB HID example](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/device/tusb_hid).

## Features

- **USB HID Keyboard**: Acts as a USB keyboard device using TinyUSB
- **Configurable Key Transmission**: Sends a configurable lowercase letter (a-z, default: 'f') periodically every 1 second
- **Physical Button Control**: Press the BOOT button (GPIO0) to toggle transmission on/off
- **BLE Remote Control**: Control via Bluetooth Low Energy using NimBLE stack
  - Toggle transmission on/off remotely
  - Change the transmitted key (a-z) remotely
- **Visual Feedback**: RGB LED strip (GPIO38) blinks blue when transmitting
- **Non-volatile Key Storage**: Selected key persists only during runtime (resets to 'f' on reboot)

## Hardware Requirements

- ESP32-S3 development board with USB support
- RGB LED strip connected to GPIO38 (typically the onboard RGB LED on ESP32-S3 DevKits)
- BOOT button on GPIO0 (standard on most ESP32 boards)

## Usage

### Initial State
The device starts with transmission **disabled**. Press the BOOT button or use BLE to enable it.

### Physical Button Control
Press the BOOT button (GPIO0) to toggle key transmission on/off.

### BLE Control

1. **Connect to device**: Look for `ESP32-HID-Kbd` in your BLE scanner app (e.g., nRF Connect, LightBlue)

2. **Discover service**: 
   - Service UUID: `12345678-1234-1234-1234-123456789abc`

3. **Characteristics**:
   - **Toggle Transmit** (UUID: `...89abd`)
     - **Description**: "Toggle Transmit"
     - **Operation**: READ to toggle transmission on/off
     - **Returns**: Current state (0x00 = off, 0x01 = on)
   
   - **HID Key** (UUID: `...89abe`)
     - **Description**: "HID Key (a-z)"
     - **READ**: Returns current key as single byte (e.g., 0x61 for 'a')
     - **WRITE**: Set new key (single byte, lowercase a-z only)

### Example BLE Usage
Using nRF Connect or similar:
1. Connect to "ESP32-HID-Kbd"
2. To toggle transmission: Tap **READ** on "Toggle Transmit" characteristic
3. To change key to 'a': **WRITE** `0x61` to "HID Key (a-z)" characteristic
4. To check current key: **READ** from "HID Key (a-z)" characteristic

## Building and Flashing

```bash
idf.py build
idf.py flash monitor
```

## Configuration

Key settings in `sdkconfig.defaults`:
- TinyUSB HID device count
- NimBLE peripheral role enabled
- Bluetooth stack configuration

## Architecture

- `tusb_hid_example_main.c`: Main application, USB HID, button handling, LED control, and API functions
- `nimble.c`: Bluetooth Low Energy interface implementation
- Clean API separation allows easy addition of other control interfaces (e.g., web UI)
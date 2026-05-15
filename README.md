[![PlatformIO CI & Release](https://github.com/cstoffel/ESP8266-BEAN-RX-TX/actions/workflows/main.yml/badge.svg)](https://github.com/cstoffel/ESP8266-BEAN-RX-TX/actions/workflows/main.yml)

# ESP8266 Toyota BEAN Reader

Reads Toyota BEAN (Body Electronics Area Network) bus messages on an ESP8266 (e.g., Wemos D1 Mini) and forwards headlight status to an ESP-NOW listener.

## What It Does

The BEAN bus carries body control data at 10 kbps over a single-wire half-duplex bus. This firmware:

1. Captures edge transitions on the BEAN RX pin using a hardware interrupt.
2. Decodes the raw pulse stream: noise filtering, start-of-frame detection, bit destuffing, byte packing.
3. Validates each frame against the Toyota CRC-8 table.
4. I wanted to listen to for the signal that indicated headlight on/off, so the current code filter for a specific ECU (DID `0xFE` / SID `0x7F`) and extracts the headlight status bit.
5. Transmits a compact binary `LightPacket` to a paired ESP32-C6 via ESP-NOW.

**_You could easily listen for whatever message(s) you are interested in and send those. I would caution about forwarding everything. BEAN is pretty chatty._**

## Hardware

| Signal  | ESP8266 Pin | Wemos D1 Mini Label |
| ------- | ----------- | ------------------- |
| BEAN RX | GPIO5       | D1                  |

A signal conditioning circuit (e.g. LM393 comparator) is required to level-shift the 5 V BEAN bus signal down to 3.3 V logic. Set `INVERT_SIGNAL = true` if the comparator inverts the signal (typical open-collector configuration).

## Software

- **Framework:** Arduino (via PlatformIO)
- **Board:** Wemod D1 Mini (`d1_mini`)
- **Platform:** `espressif8266`
- **Libraries:** `ESP8266WiFi`, `espnow` (bundled with the ESP8266 Arduino core)

## Configuration

All tuneable constants are at the top of [`src/main.cpp`](src/main.cpp):

| Constant               | Default                           | Description                                  |
| ---------------------- | --------------------------------- | -------------------------------------------- |
| `receiverAddress`      | `{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}` | MAC address of the target ESP32-C6.          |
| `BEAN_RX_PIN`          | `5`                               | GPIO pin connected to the BEAN bus signal    |
| `INVERT_SIGNAL`        | `true`                            | Set `false` if your circuit does not invert  |
| `TARGET_ECU_DID`       | `0xFE`                            | Device ID to filter for                      |
| `TARGET_ECU_SID`       | `0x7F`                            | Service ID to filter for                     |
| `LIGHT_STATUS_BITMASK` | `0x08`                            | Bitmask for the headlight bit in the payload |
| `LIGHT_PAYLOAD_BYTE`   | `0`                               | Payload byte index containing light status   |

**NOTE:** `{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}` will broadcast to all receivers. If you want to target a single device, you must these values with the MAC addresss from that device. 

## Build & Flash

```bash
# Install PlatformIO CLI if needed
pip install platformio

# Build
pio run

# Flash
pio run --target upload

# Monitor serial output (115200 baud)
pio device monitor
```

## Serial Commands

| Command   | Effect                             |
| --------- | ---------------------------------- |
| `log on`  | Enable verbose BEAN decode logging |
| `log off` | Disable verbose logging            |

## Test Mode

Set `TEST_MODE_ENABLED = true` in `main.cpp` to transmit synthetic light-on / light-off packets on a 5-second cycle (every 500 ms) without requiring a live BEAN bus. Useful for validating the ESP-NOW link and receiver firmware independently.

## ESP-NOW Packet Format

The packet is a 2-byte packed struct sent as raw bytes:

```
LightPacket (packed, 2 bytes) {
    uint8_t type;  // Always 'L' (0x4C)
    uint8_t on;    // 1 = lights on, 0 = lights off
}
```

Packets are sent:

- **On state change** — immediately when the headlight bit transitions on/off.
- **Periodic sync** — every 500 ms regardless of state change, so the receiver stays in sync if it reboots or misses a packet.

## Radio Watchdog

If no ESP-NOW delivery acknowledgement is received for 10 seconds, the firmware prints `Watchdog: radio hang detected, rebooting.` and calls `ESP.restart()`. This recovers from a hung radio stack without requiring physical intervention.

## CI / CD

GitHub Actions are configured in [`.github/workflows/main.yml`](.github/workflows/main.yml):

| Trigger                | Job              | What it does                                                                                     |
| ---------------------- | ---------------- | ------------------------------------------------------------------------------------------------ |
| Pull request to `main` | `test_build`     | Runs `pio run` to verify the firmware compiles                                                   |
| Push of a `v*` tag     | `create_release` | Builds the firmware and creates a GitHub Release with `firmware.bin` and `firmware.elf` attached |

To cut a release:

```bash
git tag v1.0.0
git push origin v1.0.0
```

## License

This project is licensed under the [GNU General Public License v3.0](LICENSE).

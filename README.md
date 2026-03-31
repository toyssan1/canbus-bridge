# CAN Bus Bridge — ESP32-S3

Dual-CAN bridge firmware for the ESP32-S3. Receives frames on two independent CAN buses, logs them to Serial, and watches for a specific trigger frame to fire a response.

## Hardware

| Function | GPIO |
|---|---|
| TWAI TX (CAN0) | 8 |
| TWAI RX (CAN0) | 9 |
| SPI MOSI (CAN1) | 11 |
| SPI MISO (CAN1) | 12 |
| SPI SCK (CAN1) | 13 |
| MCP2517FD CS | 10 |
| MCP2517FD INT | 3 |
| LED — TWAI TX | 17 |
| LED — TWAI RX | 18 |
| LED — SPI TX | 38 |
| LED — SPI RX | 37 |

## CAN Buses

- **CAN0 (TWAI)** — Uses the ESP32's built-in TWAI peripheral at 500 kbps. Managed by the `ESP32-TWAI-CAN` library.
- **CAN1 (SPI)** — Uses an external MCP2517FD controller over SPI (FSPI) at 1000 kbps. Managed by the `ACAN2517FD` library. Includes automatic recovery if the chip loses sync or the RX path stalls.

## How It Works

1. **Receive & Log** — Each loop iteration drains pending frames from both CAN buses and prints them to Serial (`TWAI 0x1A3 FF 00 ...` / `SPI 0x140810FF 80 ...`).

2. **Trigger** — When a 29-bit extended frame with ID `0x140810FF` arrives on the SPI CAN bus and bit 7 (MSB) of byte 0 is set:
   - An 11-bit standard frame is immediately sent on the TWAI bus with ID `0x062`, byte 0 = `0x01`.
   - 10 ms later, a follow-up frame is sent with byte 0 = `0x00` to clear it.
   - The trigger is one-shot — it won't fire again until bit 7 returns to `0`.

3. **LED Indicators** — Four LEDs pulse briefly (35 ms) on each TX or RX event, giving a visual heartbeat for bus activity.

4. **SPI CAN Health** — If the MCP2517FD fails to initialize at boot, the firmware retries up to 5 times, then continues retrying every 3 seconds in the main loop. An RX stall detector forces a reinit if the interrupt line stays asserted for 750 ms with no frames drained.

## Setup — Arduino IDE

### 1. Install ESP32 Board Support

1. Open **Arduino IDE** → **File** → **Preferences**.
2. In **Additional Board Manager URLs**, add:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Go to **Tools** → **Board** → **Boards Manager**, search for **esp32** by Espressif, and install it.

### 2. Install Required Libraries

Go to **Sketch** → **Include Library** → **Manage Libraries** and install:

| Library | Author | Notes |
|---|---|---|
| **ACAN2517FD** | Pierre Molinaro | Drives the external MCP2517FD SPI CAN controller |
| **ESP32-TWAI-CAN** | handmade0octopus | Drives the ESP32's built-in TWAI CAN peripheral |

### 3. Board & Port Settings

Under **Tools**, set:

| Setting | Value |
|---|---|
| Board | **ESP32S3 Dev Module** |
| USB Mode | **Hardware CDC and JTAG** |
| USB CDC On Boot | **Enabled** |
| Upload Speed | **921600** |
| Port | Your device (e.g. `/dev/cu.usbmodem2101` on Mac, `COMx` on Windows) |

### 4. Open, Compile & Upload

1. Open `12keypad-to-can.ino` in Arduino IDE.
2. Click **Verify** (checkmark) to compile.
3. Click **Upload** (arrow) to flash.
4. Open **Tools** → **Serial Monitor**, set baud to **115200**, and you should see `ESP32-S3 Dual CAN Bridge` followed by CAN frame logging.

---

## Setup — VS Code + PlatformIO

### 1. Install VS Code & PlatformIO

1. Download and install [VS Code](https://code.visualstudio.com/).
2. Open VS Code, go to the **Extensions** panel (⇧⌘X / Ctrl+Shift+X), search for **PlatformIO IDE**, and install it.
3. Wait for PlatformIO to finish its initial setup (it will download toolchains automatically).

### 2. Open the Project

1. In VS Code, click **File** → **Open Folder** and select the project root folder (the one containing `platformio.ini`).
2. PlatformIO will detect the project and download the ESP32 platform and libraries automatically based on `platformio.ini`.

### 3. Build

Click the **checkmark** icon on the bottom toolbar, or run in the terminal:

```bash
pio run
```

### 4. Upload

Connect the ESP32-S3 via USB, then click the **arrow** icon on the bottom toolbar, or run:

```bash
pio run -t upload
```

> **Note:** If your USB port differs from `/dev/cu.usbmodem2101`, edit `upload_port` and `monitor_port` in `platformio.ini`.

### 5. Serial Monitor

Click the **plug** icon on the bottom toolbar, or run:

```bash
pio device monitor
```

Baud rate is set to 115200 in `platformio.ini`. You should see frame logging like:

```
ESP32-S3 Dual CAN Bridge
Starting CAN buses...
TWAI started @ 500k (TX=GPIO8 RX=GPIO9)
SPI MCP2517FD started @ 1000000
Ready.
TWAI 0x1A3 FF 00 12 34 00 00 00 00
SPI  0x140810FF 80 00 00 00 00 00 00 00
TRIGGER -> TWAI 0x062 [01] ok
TRIGGER -> TWAI 0x062 [00] reset ok
```

### PlatformIO Quick Reference

| Command | What it does |
|---|---|
| `pio run` | Build the firmware |
| `pio run -t upload` | Build & flash to the board |
| `pio device monitor` | Open serial monitor (115200 baud) |
| `pio run -t clean` | Clean build artifacts |

---

## Dependencies

| Library | Version | Purpose |
|---|---|---|
| [ACAN2517FD](https://github.com/pierremolinaro/acan2517FD) | ^2.1.16 | SPI CAN (MCP2517FD) driver |
| [ESP32-TWAI-CAN](https://github.com/handmade0octopus/ESP32-TWAI-CAN) | ^1.0.1 | Built-in TWAI peripheral driver |

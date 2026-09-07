# ESP32 Desk Lamp

A local Wi-Fi desk lamp controller for an ESP32-C3 SuperMini. It provides warm, cool, balanced, off, and brightness controls from a browser.

## Hardware

- ESP32-C3 SuperMini board with 4 MB flash
- Two optocouplers rated for the lamp power stage
- Two input resistors, typically 220-330 ohm for a 3.3 V optocoupler LED input
- A properly rated isolated LED driver or MOSFET power stage on the optocoupler output side
- Warm and cool LED channels with their own current limiting and power supplies

ESP32 connections:

| ESP32 pin | Connection |
| --- | --- |
| GPIO4 | Warm optocoupler input LED through its resistor |
| GPIO5 | Cool optocoupler input LED through its resistor |
| GPIO6 | DS1302 CLK |
| GPIO7 | DS1302 DAT |
| GPIO10 | DS1302 RST/CE |
| 3.3V | DS1302 VCC |
| GND | Optocoupler input-side ground |

The ESP32 GPIOs must only connect to the optocoupler input side. Do not connect GPIO4/5 directly to an LED strip, mains circuit, high-current supply, or the isolated output-side ground. Use a qualified, appropriately rated driver stage and enclosure for any voltage above SELV. Verify creepage, clearance, fuse protection, and thermal limits before powering the lamp.

## Configure, build, and upload with PlatformIO

1. Install VS Code and the PlatformIO extension.
2. Edit the local `src/.wifi_secrets.h` file and replace `YOUR_WIFI_SSID` and `YOUR_WIFI_PASSWORD`. This hidden file is ignored by Git.
3. Connect the ESP32-C3 SuperMini over USB.
4. Click PlatformIO **Build**, then **Upload**, then **Serial Monitor** in the VS Code status bar.
5. Browse to `http://lamp.local/` from a device on the same Wi-Fi network. If mDNS is unavailable on that device or network, use the numeric IP address printed in the serial monitor.

The equivalent terminal commands are:

```powershell
pio run
pio run --target upload
pio device monitor
```

The environment uses Arduino with ESP-IDF services, selected in `platformio.ini`. PlatformIO writes generated files to `.pio/`; the old native ESP-IDF `build/` directory is not used by this workflow.

The firmware uses LEDC PWM at 5 kHz. The `warm` and `cool` outputs are active-high by default. If the optocoupler input stage is inverted, change the output polarity in the driver stage rather than connecting the lamp power directly to the ESP32.

## Clock, timer, and schedule

The lamp restores its clock from a DS1302 RTC at startup, then synchronizes from `pool.ntp.org` through the local router when internet is available. A successful internet sync updates the RTC. The `WIB-7` timezone (UTC+7, Jakarta), NTP server, and RTC pins can be changed in `src/lamp_config.h`.

The DS1302 module must be powered from 3.3 V. Its trickle charger is explicitly disabled in firmware so the installed non-rechargeable CR2032 is not charged.

- A countdown timer can turn the lamp on or off after 1–1440 minutes.
- The daily schedule can turn the lamp on and off at selected local times.
- Schedule settings are saved in flash and survive restarts. Countdown timers restart from zero when the device reboots.
- When an automation turns the lamp on, it restores the most recently selected warm, balanced, or cool mode.

## OTA firmware updates

After the first USB upload, future firmware can be installed from the lamp's web page:

1. Build the new firmware with `pio run`.
2. Open the lamp's IP address and choose **Firmware update**.
3. Select `.pio/build/esp32-c3-supermini/firmware.bin`, then click **Install update**.
4. Keep the lamp powered while it uploads. It restarts automatically when validation succeeds.

The custom partition table keeps two application slots; the update is written to the inactive slot before it is selected for the next boot. Changing from the former single-app partition layout requires one final upload over USB. OTA is intended for a trusted local network and the upload endpoint does not include authentication.

## API

- `GET /api/state` returns the current mode and brightness.
- `POST /api/state` accepts JSON such as `{"mode":"warm"}` or `{"brightness":450}`.
- `POST /api/ota` accepts a raw ESP32 application `.bin` and restarts into it after validation.
- The root page serves the browser control panel.

Real Wi-Fi credentials are intentionally not included in this repository.

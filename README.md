# ATtiny85 DRV8833 Motor Driver

https://youtu.be/sycSdI49hlY?si=9vmVVg6HyiEosMC9 How to set up an Uno as a flashing tool.
Minimal potentiometer-controlled DC motor firmware for the Digispark (ATtiny85). Turn the knob, motor spins. Click the pot off, power cuts. No app, no cloud, no sleep state to manage.

[![PlatformIO](https://img.shields.io/badge/PlatformIO-arduino-orange.svg)](https://platformio.org/)
[![MCU](https://img.shields.io/badge/MCU-ATtiny85-blue.svg)](https://www.microchip.com/en-us/product/attiny85)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

---

## Hardware

| Component | Part |
|-----------|------|
| MCU | [Digispark](http://digistump.com/products/1) (ATtiny85 @ 8 MHz internal) |
| Driver | DRV8833 breakout, single channel |
| Motor | 3 V, 1 A DC with flyback diode |
| Control | 10 kΩ linear pot with integral SPST power switch |
| Power | Single Li-ion cell (3.0–4.2 V) |

### Pinout

| Digispark Pin | ATtiny85 | Connected To | Notes |
|---------------|----------|--------------|-------|
| P0 | PB0 / OC0A | DRV8833 AIN1 | PWM output |
| P1 | PB1 | DRV8833 AIN2 | Held LOW (forward / coast) |
| P2 | PB2 / ADC1 | Pot wiper | Add 100 nF cap wiper-to-GND |

### DRV8833 Truth Table (one channel)

| AIN1 | AIN2 | Output |
|------|------|--------|
| PWM | LOW | Forward PWM (fast decay) |
| LOW | LOW | Coast — used as the off state |
| HIGH | HIGH | Brake — not used |

---

## How It Works

The potentiometer drives the ADC. A small deadzone at the bottom (`POT_DEAD_LOW`) ensures the motor stops cleanly before the integral switch clicks off. A matching deadzone at the top (`POT_DEAD_HIGH`) ensures full speed is reachable even if the pot doesn't hit a perfect 1023. Between those bounds, ADC counts map linearly to PWM duty cycle 1–254.

PWM runs at **31.25 kHz** (Timer0, Fast PWM, prescaler = 1 on 8 MHz clock) — above audible range, no motor whine.

There is no sleep logic. The pot switch cuts the entire supply; firmware only runs when the user has turned the pot on.

---

## Wiring

```
Li-ion (+) ──┬──────────────────── DRV8833 VM
             │
         Digispark 5V pin   ← not VIN — bypasses the regulator
             │
         Digispark GND ───── DRV8833 GND ───── Li-ion (–)

Potentiometer:
  CCW end  → GND
  Wiper    → P2  (+ 100 nF cap to GND)
  CW end   → 5V
  Switch   → in series with Li-ion (+) line

DRV8833:
  AIN1 → P0
  AIN2 → P1
  AOUT1/AOUT2 → Motor (in this case, OUT 1 is Positive, OUT 2 is negative)
```

> **Important:** Connect Li-ion (+) to the Digispark **5V pin**, not VIN. The VIN pin goes through the onboard regulator, which wastes power and drops voltage at the motor.

> **Important:** Desolder or disable the Digispark power LED. It draws 3–5 mA continuously from the battery regardless of what the firmware does.

---

## Setup Instructions

### Prerequisites

- [VS Code](https://code.visualstudio.com/) + [PlatformIO IDE](https://platformio.org/install/ide?install=vscode)
- [Git](https://git-scm.com/)

## PlatformIO doesn't work out of the box with Windows 11. You have to update the micronucleus tool and install the drivers, then restart your PC.

PlatformIO's bundled tool-micronucleus (v1.250.210222) ships a Windows binary that commonly crashes on modern Windows 10/11 due to missing Visual C++ runtime DLLs or a bitness mismatch.
Fix — replace the bundled binary with the official one:

Download the latest micronucleus Windows binary from the official releases page https://github.com/micronucleus/micronucleus/releases — grab micronucleus-cli.exe from the latest release assets (not the source zip).
Find PlatformIO's bundled copy. It will be at something like:

%USERPROFILE%\.platformio\packages\tool-micronucleus\micronucleus.exe

Rename the existing one as a backup:

micronucleus.exe  →  micronucleus.exe.bak

Drop the downloaded micronucleus-cli.exe in that folder and rename it to micronucleus.exe.
Run pio run -t upload again — plug in on the prompt as normal.

Also confirm the driver is installed correctly while you're at it. Open Device Manager while the Digispark is plugged in and look for it under "libusb-win32 devices" or "Digispark Bootloader". If it shows up under "Unknown devices" or "HID devices" instead, the Digistump driver install didn't take — re-run Install Drivers.exe as Administrator.
The fact that it found the upload protocol and got to the micronucleus call means the platformio.ini is correct and the build is clean. Once the binary is swapped this should flash on the first try.

## Flashing with PlatformIO
 
The Digispark bootloader requires you to plug in the USB **after** the upload process starts. PlatformIO will prompt you. The Digispark does not use a standard serial bootloader. It uses **micronucleus**, a USB HID bootloader that only activates for the first ~5 seconds after power-on. The flash sequence is therefore different from every other Arduino-compatible board.
 
### 1. Install the USB driver (Windows only)
 
On Windows, the Digispark requires the **Digistump drivers** — it will not enumerate correctly with the default HID driver.
 
Download and run the installer from the [Digistump releases page](https://github.com/digistump/DigistumpArduino/releases) (`Digistump.Drivers.zip` → run `Install Drivers.exe`).
 
macOS and Linux do not need a driver. On Linux you may need a udev rule:
 
```bash
# /etc/udev/rules.d/49-micronucleus.rules
SUBSYSTEMS=="usb", ATTRS{idVendor}=="16d0", ATTRS{idProduct}=="0753", MODE="0660", GROUP="plugdev"
```
 
Then reload: `sudo udevadm control --reload-rules && sudo udevadm trigger`
 
### 2. Start the upload — board unplugged
 
Run the upload command first, **before** plugging in the board:
 
```bash
pio run -t upload
```
 
PlatformIO will build the firmware, then pause with a message like:
 
```
Connecting to USB device...
Please plug in the device (will time out in 60 seconds)...
```
 
### 3. Plug in the Digispark
 
Only plug in the USB **after** you see the above prompt. The micronucleus bootloader activates on power-up and listens for about 5 seconds. PlatformIO will detect it and flash automatically.
 
```
> Device is found!
connecting: 40% complete
> Available space for user applications: 6012 bytes
> Suggested sleep time between sending pages: 7ms
programming: 60% complete
reading and validating: 80% complete
> Uploaded. Outro-ing. Bye.
```
 
### 4. Board resets and runs
 
After flashing completes the board resets automatically and your firmware starts immediately — no button press needed.
 
### Reflashing
 
Unplug the board, run `pio run -t upload` again, then plug back in when prompted. You do not need to hold any button.
 
### Troubleshooting
 
**"Device not found" timeout** — You either plugged in too early (before the prompt) or too late (after the 5-second bootloader window). Unplug, wait, and try again.
 
**Windows: device shows as unknown in Device Manager** — Drivers are not installed. See step 1.
 
**Linux: permission denied** — udev rule is missing or not reloaded. See step 1.
 
**Upload succeeds but firmware doesn't run** — The Digispark has a USB initialisation delay of about 5 seconds on boot (the micronucleus bootloader waits for a connection attempt before handing off). This is normal. Your firmware starts after that delay whenever the board powers up without a flash attempt.

### Tuning

All tunables are `#define`s at the top of `src/main.cpp`:

| Define | Default | Description |
|--------|---------|-------------|
| `POT_DEAD_LOW` | `12` | ADC counts below this = motor off. Raise if motor creeps at minimum. |
| `POT_DEAD_HIGH` | `1011` | ADC counts above this = full speed. Lower if pot can't quite reach max. |
| `LOOP_MS` | `20` | Control loop period in ms (50 Hz). |

---

## Project Structure

```
├── src/
│   └── main.cpp          # All firmware — ~80 lines
├── platformio.ini
└── README.md
```

No library dependencies. Timer0 and ADC are configured directly via registers — smaller flash footprint and no framework overhead.

---

## Resources

- [ATtiny85 Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2586-AVR-8-bit-Microcontroller-ATtiny25-ATtiny45-ATtiny85_Datasheet.pdf)
- [DRV8833 Datasheet](https://www.ti.com/lit/ds/symlink/drv8833.pdf)
- [Digispark Wiki](http://digistump.com/wiki/digispark)
- [PlatformIO ATtiny85 docs](https://docs.platformio.org/en/latest/boards/atmelavr/digispark-tiny.html)

---

## License

MIT.

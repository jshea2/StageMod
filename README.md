# StageMod

StageMod turns an Olimex ESP32-PoE-ISO into a configurable OSC sensor hub with a built-in web UI.


https://github.com/user-attachments/assets/1f41d19c-4316-445e-9120-f764ade31daa


<img width="478" height="868" alt="Screenshot 2026-01-27 at 1 33 12 PM" src="https://github.com/user-attachments/assets/7543e0a2-bbac-4586-8501-da448972fe47" />


## Hardware
- Olimex ESP32-PoE-ISO
- Sensors (digital or analog; up to 6 total)

## Flash firmware (no IDE)
1) Open the web flasher: https://jshea2.github.io/StageMod/web-flasher/
2) Plug in the ESP32-PoE-ISO over USB and click **Connect**
3) If it does not show up, hold **BOOT** while plugging in
4) When prompted, allow **Erase device** and install

## Configure the device
StageMod uses a web UI for setup. You can access it over Ethernet or WiFi.

Network behavior
- If Ethernet is connected, it uses DHCP by default.
- If no Ethernet is present, it starts a WiFi AP and shows a configuration page.

Open the UI
- Default: http://stagemod.local
- Or use the device IP shown by your router
- You can change the hostname later and use that URL instead

## OSC behavior
Set the target OSC client and behavior in the web UI:
- Client IP and port
- Test OSC button sends a one-shot pulse to your configured address
- Heartbeat sends a repeating OSC message at a custom interval
- Up to 6 sensors, each with its own type and OSC address
- Buttons and digital sensors can enable a cooldown to prevent rapid re-triggers
- Optional username/password security

Factory reset
- Hold the onboard button for 5 seconds to restore defaults

## TouchDesigner
Use the included TouchDesigner file to monitor and test OSC:
- `Touchdesigner/StageMod-TouchSensorOSC.toe`
- Easy UI for configuring and monitoring OSC In/Out
- Shows online/offline using a heartbeat OSC message (`/heartbeat`)

## 3D prints
Files are in `3D_Prints/`:
- `StageMod-v1.stl` for printing
- `StageMod-v1.f3d` for editing in Fusion 360

## Build from source (PlatformIO)
1) Clone the repo
2) Open in PlatformIO and select the `esp32-poe` environment
3) Build and upload:
   - `pio run -e esp32-poe -t upload`

## Credits
Original project by [Facing Tomorrow](https://github.com/facingtomorrow). This repo adds OSC support and TouchDesigner UI components.

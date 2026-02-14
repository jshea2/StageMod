# StageMod — Product Spec & Strategy (Draft v0.2)

## 1) Product Summary
- **Purpose:** Network‑connected trigger interface that converts physical inputs (buttons/encoders/pots/time) into OSC/UDP/HTTP/GPIO outputs for show control, installations, and interactive systems.
- **Core value:** Fast, reliable, configurable I/O with web‑based setup and zero‑IDE firmware updates.

## 2) Positioning
- **What it is:** A compact, PoE‑powered controller that turns physical inputs into OSC (and later MQTT/DMX) outputs for live show control and interactive systems.
- **Why it wins:** Local web UI, no cloud required, built for show workflows.

## 3) Form Factor & Enclosure (V1)
- **Standalone box**
- **Mounting:**
  - Bottom clamp hole
  - Top + bottom wall‑mount holes
- **Controls/IO variants:**
  - 3‑button model
  - 1‑pot model
  - 3‑encoder model
  - DIY model (blank panel)

## 4) Hardware Variants (V1 lineup)
1) **StageMod‑3B** — 3 momentary buttons
2) **StageMod‑1P** — 1 potentiometer
3) **StageMod‑3E** — 3 encoders (with switch)
4) **StageMod‑DIY** — blank front panel (user‑wired inputs)

**Common board:** ESP32‑PoE (Ethernet primary, Wi‑Fi secondary)

## 5) Connectivity
- **Primary:** PoE Ethernet
- **Secondary:** Wi‑Fi 2.4 GHz
- **Fallback:** AP mode for setup
- **Discovery:** mDNS (e.g. `stagemod.local`)
- **No cloud required**

## 6) Core Features (Current Firmware)

### Inputs
- Button: momentary / toggle / double / triple / long press
- Encoder: relative output (QLab / Resolume compatible)
- Analog: pot/slider
- Time: date+time / daily / weekly / interval

### Outputs
- OSC (primary)
- UDP Raw
- HTTP Request
- GPIO Out: level / pulse / PWM
- Reboot device
- Reset cooldowns

### Triggers
- Up to 20 triggers
- Each trigger: 1 input → multiple outputs (max 5)
- Reorderable, collapsible UI
- Autosave on checkbox changes

### Web UI
- Local config only
- Settings, Triggers, Serial log
- Network client list (shared between OSC/UDP/HTTP)
- Heartbeat per client
- Security: username/password
- Firmware update from local .bin

## 7) Firmware Update Strategy (V1)
- Local web updater (WLED‑style)
- No cloud required

## 8) Reliability / Performance
- Low‑latency response
- Configurable cooldowns
- Built for continuous use

## 9) Roadmap (V2+)
- MQTT output
- DMX / Art‑Net output
- OSC input (network triggers)
- Schedules with multiple time events
- Advanced routing (conditions, rules)

## 10) Open Decisions
- Target customers (theatre tech, artists, AV integrators, etc.)
- Target price range
- Certifications (FCC/CE/PoE)
- Final enclosure material
- Connector types (terminal blocks vs jacks)
- Labeling + SKU naming


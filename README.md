# OBD-II Diagnostic Scanner — Bare-Metal NXP S32K144

Bare-metal OBD-II diagnostic scanner implemented entirely in C99 on the 
NXP S32K144EVB-Q100 (ARM Cortex-M4F @ 80 MHz) — no RTOS, no HAL, 
no abstraction layers.

Validated on a real vehicle: **Suzuki Swift 2026**.

---

## Features

- **FlexCAN0 driver** from scratch — 500 kbps, standard & extended frames
- **ISO-TP stack** (ISO 15765-2) — Single Frame, First Frame, 
  Consecutive Frames, Flow Control
- **OBD-II protocol layer** (SAE J1979):
  - Service 0x01 — Live PIDs (RPM, coolant temp, vehicle speed, etc.)
  - Service 0x03 — Read DTCs
  - Service 0x04 — Clear DTCs
  - Service 0x09 — Read VIN
- **UART–Bluetooth bridge** via ESP32 (SPP + BLE)
- **Companion Android app** — MVVM architecture, Jetpack Compose
- Clock tree configured at 80 MHz via SCG peripheral
- Validated with logic analyzer and oscilloscope on J13 connector

---

## Hardware

| Component | Details |
|-----------|---------|
| MCU Board | NXP S32K144EVB-Q100 |
| Core | ARM Cortex-M4F @ 80 MHz |
| CAN Transceiver | On-board (EVB) |
| Bluetooth Module | ESP32 (UART bridge) |
| Target Vehicle | Suzuki Swift 2026 |
| Connection | J13 EVB connector → OBD-II port |

---

## Software Stack

| Layer | Technology |
|-------|-----------|
| Language | C99 (bare-metal) |
| IDE | S32 Design Studio 3.6.1 |
| CAN Driver | FlexCAN0 (custom, no SDK) |
| Transport | ISO-TP (ISO 15765-2) |
| Application | OBD-II SAE J1979 |
| Wireless | ESP32 UART bridge |
| Android App | Kotlin, MVVM, Jetpack Compose |

---

## Architecture

NXP S32K144 (ARM Cortex-M4)
│
├── FlexCAN0 Driver (500 kbps)
│   └── ISO-TP Stack (ISO 15765-2)
│       └── OBD-II Layer (SAE J1979)
│           ├── Service 0x01 — Live PIDs
│           ├── Service 0x03 — Read DTCs
│           ├── Service 0x04 — Clear DTCs
│           └── Service 0x09 — VIN
│
└── LPUART1 Driver
    └── ESP32 Bridge (UART → Bluetooth)
        └── Android App (MVVM + Jetpack Compose)

Target: Suzuki Swift 2026 via OBD-II port (J13 connector on EVB)

---

## Project Status

- [x] FlexCAN0 driver
- [x] ISO-TP stack (SF / FF / CF / FC)
- [x] OBD-II services 01 / 03 / 04 / 09
- [x] UART–Bluetooth bridge (ESP32)
- [x] Android companion app
- [x] Validated on real vehicle

---

## Author

**José Jesús Guerrero González**  
Automotive Systems Engineering — IPN UPIIH  
Specialization: Automotive Programming  
📧 jesus.guerrero.inge@gmail.com  
🔗 [LinkedIn](https://linkedin.com/in/jose-jesus-guerrero-gonzalez)

---

## License

MIT License — see [LICENSE](LICENSE) for details.

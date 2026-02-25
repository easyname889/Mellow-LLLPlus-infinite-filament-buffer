# ⬇️ Quick Download — Pre-Compiled Firmware (Just Flash It!)

> **Don't want to compile anything?** Download the ready-to-flash firmware file here:
>
> ### 📦 [**Download fly_buffer_lllp_v1.2.bin**](firmware/fly_buffer_lllp_v1.2.bin)
>
> **How to flash:**
> 1. Hold the **BOOT** button on your Fly Buffer board, then press **RESET** (this enters DFU mode).
> 2. Connect the board to your computer via USB.
> 3. Use [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) (free) or [dfu-util](https://dfu-util.sourceforge.net/) to upload the `.bin` file.
> 4. Press **RESET** again — done! 🎉

---

# I would be super happy if you bought your Mellow filament buffer plus (LLLP's) trough my affiliate link here, you get same price and i get a couple of cents =) 
# ---> Official Mellow LLLP shop https://s.click.aliexpress.com/e/_c3Mx41N7

## To create a infinite filament system you need at least 2 LLLPs, if you have a Elegoo Orangestorm Giga printer i would recommend one front LLLP that is running "front" and that will run separate from the back daisy chained ones. So its 1+2 at minimum (total 3).
---
---
---

# Fly Buffer LLLP — Smart Daisy-Chain Firmware (V1.2)

Custom firmware for the **Mellow Fly Buffer (LLLP)** board (STM32F072C8T6). Replaces factory bang-bang logic with **Round-Robin Continuous Feeding** — allowing multiple buffer units to be daisy-chained into an infinite filament loop.

---

## Table of Contents

1. [Architecture Overview](#-architecture-overview)
2. [Key Features](#-key-features)
3. [Project Structure](#-project-structure)
4. [Complete Function Reference](#-complete-function-reference)
5. [How Round-Robin Mode Works](#-how-round-robin-mode-works-step-by-step)
6. [Motor Control State Machine](#-motor-control-state-machine)
7. [Sensor Logic & Pin Definitions](#-sensor-logic--pin-definitions)
8. [Wiring Guide](#-wiring-guide)
9. [Controls & UI](#-controls--ui)
10. [Configuration Modes](#-configuration-modes)
11. [Safety Failsafes & Timeouts](#-safety-failsafes--timeouts)
12. [Serial (USB) Commands](#-serial-usb-commands)
13. [Installation & Flashing](#-installation--flashing)
14. [Debug Checklist](#-debug-checklist)

---

## 🏗 Architecture Overview

```
┌──────────────────────────────────────────────────┐
│  main.cpp                                        │
│   ├─ setup()  → buffer_init()                    │
│   └─ loop()   → buffer_loop()     (never returns)│
└──────────────┬───────────────────────────────────┘
               │
┌──────────────▼───────────────────────────────────┐
│  lib/buffer/buffer.cpp  (Core Logic)             │
│                                                  │
│  Initialization Layer                            │
│   ├─ buffer_init()                               │
│   │   ├─ buffer_sensor_init()                    │
│   │   ├─ buffer_motor_init()                     │
│   │   ├─ EEPROM restore (timeout, speed, mode)   │
│   │   ├─ Timer6 ISR setup                        │
│   │   └─ iwdg_init()       (Watchdog)            │
│   │                                              │
│  Main Loop (runs forever inside buffer_loop)     │
│   ├─ read_sensor_state()  (Sample all inputs)    │
│   ├─ LED state machine    (Visual feedback)      │
│   ├─ check_speed_combo()  (Config mode)          │
│   ├─ motor_control()      (Core state machine)   │
│   └─ USB_Serial_Analys()  (Serial commands)      │
│                                                  │
│  Interrupt Handlers                              │
│   ├─ timer_it_callback()  (Watchdog + timeout)   │
│   ├─ key1_it_callback()   (Button 1 ISR)         │
│   └─ key2_it_callback()   (Button 2 ISR)         │
└──────────────────────────────────────────────────┘
```

The firmware is a **single-threaded super-loop** architecture. `buffer_loop()` contains an infinite `while(1)` that never returns. A TIM6 timer interrupt runs every 100 ms for watchdog feeding and timeout tracking. Button press/release events are captured via EXTI interrupts.

---

## 🌟 Key Features

### 1. 🔄 Infinite Round-Robin Logic (Daisy Chain)
Link multiple LLLP units together (e.g., 1 → 2 → 3 → 1). When Unit 1 runs out, it signals Unit 2 to take over. When Unit 2 runs out, it triggers Unit 3. Unit 3 loops back to trigger Unit 1 (after you've reloaded it).

### 2. 🧠 Smart Hysteresis Feeding
Unlike the factory "Bang-Bang" control, this firmware uses a latching logic:
*   **Trigger:** Starts feeding at full speed when the buffer arm drops to Hall 3 (empty position).
*   **Latch:** *Continues* feeding through the neutral zone at a configurable maintenance speed.
*   **Stop:** Stops only when the buffer arm reaches Hall 2 (full position).
*   **Result:** Smoother operation and less motor stutter.

### 3. ✂️ Runout Tail Clearance
When the filament runs out, the motor **does not stop immediately**. It continues the feeding loop until the filament tail clears the internal gears (detected by the buffer arm dropping to Hall 3). This ensures the Y-Splitter path is clear for the next unit to take over.

### 4. ⚙️ On-the-Fly Configuration
Change maintenance speeds or switch between Master/Slave modes instantly by holding both buttons. The motor continues running while you configure, ensuring no interruption to the print.

### 5. 🛡️ Robust Safety Failsafes
*   **Feed Timeout:** If the buffer tries to fill for >120 s without success, it stops to prevent grinding.
*   **Unload Timeout:** Prevents endless reversing if the filament sensor fails (120 s).
*   **Y-Sensor Interlock:** A shared sensor prevents a new unit from starting if the previous unit's filament is still blocking the path.
*   **Independent Watchdog (IWDG):** Resets the MCU if the main loop stalls.

---

## 📂 Project Structure

```
Mellow-LLLPlus-infinite-filament-buffer/
├── platformio.ini                    # PlatformIO build config (STM32F072C8, Arduino, TMCstepper)
├── src/
│   └── main.cpp                      # Entry point: setup() + loop()
├── lib/
│   └── buffer/
│       ├── buffer.h                  # Pin defs, structs, enums, function prototypes
│       └── buffer.cpp                # ALL firmware logic (582 lines)
├── boards/
│   └── fly_buffer_f072c8.json        # Custom board definition (48 MHz Cortex-M0, 64 KB Flash)
├── variants/
│   └── F072C8/                       # STM32 HAL variant files (pin mapping, peripherals, linker)
│       ├── variant_FLY_BUFFER_F072Cx.h
│       ├── variant_FLY_BUFFER_F072C.cpp
│       ├── PeripheralPins_FLY_BUFFER_F072C.c
│       ├── PinNamesVar.h
│       ├── boards_entry.txt
│       └── ldscript.ld
├── DEBUG_CHECKLIST.md                # Hardware verification steps
└── README.md                         # This file
```

---

## 📖 Complete Function Reference

### Entry Points (`src/main.cpp`)

| Function | Trigger | Description |
| :--- | :--- | :--- |
| `setup()` | **MCU boot** (called once) | Initializes Serial at 115200 baud, disables DTR, calls `buffer_init()`. |
| `loop()` | **Arduino framework** (called repeatedly) | Calls `buffer_loop()`, which contains an infinite `while(1)` and never returns. |

### Initialization (`lib/buffer/buffer.cpp`)

| Function | Called By | Description |
| :--- | :--- | :--- |
| `buffer_init()` | `setup()` | **Master init.** Calls all sub-inits, restores EEPROM settings, configures Timer6 ISR, starts watchdog. Sets initial LED/pin states based on whether filament is loaded. |
| `buffer_sensor_init()` | `buffer_init()` | Configures all sensor pins as `INPUT`, buttons as `INPUT`, output pins as `OUTPUT`. Attaches EXTI interrupts for KEY1 and KEY2. Initializes `DULIAO` and `DUANLIAO` HIGH. Configures `FRONT_SIGNAL_PIN` with internal pull-up. |
| `buffer_motor_init()` | `buffer_init()` | Configures EN/STEP/DIR pins, enables driver. Initializes TMC2209 via UART at 9600 baud: sets current to 800 mA, 16 microsteps, SpreadCycle mode, PWM autoscale. |
| `iwdg_init()` | `buffer_init()` | Configures the **Independent Watchdog (IWDG)** with prescaler 256 and reload value 1249 (~10 s timeout). Prevents MCU hang if main loop stalls. |

### Main Loop (`lib/buffer/buffer.cpp`)

| Function | Called By | Frequency | Description |
| :--- | :--- | :--- | :--- |
| `buffer_loop()` | `loop()` | **Continuous** (never returns) | The super-loop. Each iteration: reads sensors → updates LEDs → checks config combo → runs motor state machine → processes serial input. Increments `g_run_cnt` for watchdog monitoring. |
| `read_sensor_state()` | `buffer_loop()` | Every loop iteration | Samples all digital inputs into the `Buffer` struct: Hall 1/2/3 sensor states, filament runout sensor, and both button states. |
| `motor_control()` | `buffer_loop()` | Every loop iteration | **Core state machine.** Evaluates inputs with strict priority ordering (P0–P6) to decide motor direction and speed. Handles daisy chain signals, runout tail clearance, hysteresis latching, and all button actions. See [Motor Control State Machine](#-motor-control-state-machine). |
| `check_speed_combo()` | `buffer_loop()` | Every loop iteration | Detects if both KEY1+KEY2 are held simultaneously. If so, enters config mode: cycles `maintenance_divider` from 0 → 6, blinks red LED to indicate current mode, saves to EEPROM on release. Returns `true` if combo was triggered. |
| `USB_Serial_Analys()` | `buffer_loop()` | Every loop iteration | Reads characters from USB Serial. On newline, parses commands: `rt` (read timeout), `timeout <ms>` (set timeout), `info` (print settings), `speed <rpm>` (set motor speed). |

### Helper / Utility Functions

| Function | Called By | Description |
| :--- | :--- | :--- |
| `update_run_latch(bool)` | `motor_control()`, `check_speed_combo()` | Updates the `run_latch` flag. If in Mode 6 (Slave), persists the state to EEPROM so the unit can survive power cycles mid-chain. |
| `show_config_blink(uint8_t mode)` | `check_speed_combo()` | Visual feedback during config mode. Mode 0 = 1 long flash, Mode 1–5 = counted blinks, Mode 6 = 6 fast blinks. Feeds watchdog during delays. |
| `fastAtof(const char*)` | `USB_Serial_Analys()` | Lightweight string-to-float parser (no `stdlib` dependency). Used for USB `speed` command. |
| `buffer_debug()` | Not called (debug utility) | Debug function for dumping TMC2209 register values via Serial. Currently unused. |

### Interrupt Service Routines (ISRs)

| Function | Trigger | Priority | Description |
| :--- | :--- | :--- | :--- |
| `timer_it_callback()` | **TIM6 overflow** (every ~100 ms) | NVIC Priority 0 (highest) | Feeds the IWDG watchdog. Every 50 ticks (~5 s), checks if `g_run_cnt` is still incrementing — if zero, the main loop has stalled and forces a reset via infinite busy-wait (watchdog bites). Also increments `front_time` if `is_front` flag is active. |
| `key1_it_callback()` | **EXTI on PB13** (pin change) | NVIC Priority 1 | Captures button press/release timestamps for KEY1. Sets `key1_press_flag` / `key1_release_flag` for the main loop to process. Debounce: ignores releases > 500 ms after press. |
| `key2_it_callback()` | **EXTI on PB12** (pin change) | NVIC Priority 1 | Same as above, for KEY2. |

---

## 🔄 How Round-Robin Mode Works (Step-by-Step)

This is the core innovation of this firmware. Here is exactly how the daisy chain triggers, with timing and signal details.

### Prerequisites
- All units must be set to **Mode 6** (`maintenance_divider == 6`, a.k.a. Slave Mode).
- All units must have filament loaded (runout sensor LOW).
- Units are wired in a signal loop (Unit 1 PB15 → Unit 2 PB5 → Unit 3 PB15 → Unit 1 PB5).
- The Y-Splitter sensor (PB14) is **shared** across all units (wired in parallel).

### Sequence of Events

```
 ┌─────────────────────────────────────────────────────────────────────────────────┐
 │                        WIRING DIAGRAM (3 Units)                                │
 │                   All boards share common GND!                                 │
 └─────────────────────────────────────────────────────────────────────────────────┘

  DAISY CHAIN SIGNAL RING (wire one-to-one in a circle)
  ─────────────────────────────────────────────────────

  ┌──────────────────┐          ┌──────────────────┐          ┌──────────────────┐
  │     UNIT 1       │          │     UNIT 2       │          │     UNIT 3       │
  │                  │          │                  │          │                  │
  │  OUT PB15 ───────────────>  IN PB5            │          │                  │
  │                  │          │                  │          │                  │
  │                  │          │  OUT PB15 ───────────────>  IN PB5            │
  │                  │          │                  │          │                  │
  │  IN PB5  <───────────────────────────────────────────────  OUT PB15         │
  │  (pull-up)       │          │                  │          │  (Loop Back)     │
  └──────────────────┘          └──────────────────┘          └──────────────────┘

  Signal Logic:
    OUT PB15 = LOW  → "I am busy / feeding"
    OUT PB15 = HIGH → "I am done / empty — next unit GO!"
    IN  PB5  reads the previous unit's PB15 output


  Y-SPLITTER SENSOR BUS (wire in parallel to ALL units)
  ─────────────────────────────────────────────────────

                          ┌─────────────────┐
                          │  Y-Splitter     │
                          │  Switch/Sensor  │
                          │                 │
                          │  SIG ───────────┼──┬────────────────┬────────────────┐
                          │  GND ───────────┼──┼──┬─────────────┼──┬─────────────┼──┐
                          └─────────────────┘  │  │             │  │             │  │
                                               │  │             │  │             │  │
                                     Unit 1  PB14 GND  Unit 2 PB14 GND  Unit 3 PB14 GND

  Signal Logic:
    HIGH (~4.6V) = Path CLEAR  → no filament in Y-splitter → next unit allowed to start
    LOW  (0V)    = BLOCKED     → filament still in Y-splitter → next unit WAITS

  ⚠️ PB14 is 5V tolerant. The sensor can run on 5V logic.
  ⚠️ BOTH conditions must be true for a slave to start:
     1. IN PB5 = HIGH  (previous unit done)
     2. PB14   = HIGH  (Y-path clear)
```

#### Step 1 — User presses KEY2 on Unit 1
- `update_run_latch(true)` is called → `run_latch = true`.
- `DULIAO` (PB15) stays **LOW** — Unit 1 is busy.
- Motor enters **P6: Normal Auto Operation** — feeds filament using hysteresis (Hall 3 triggers full speed → latched through neutral → stops at Hall 2).

#### Step 2 — Unit 1 runs out of filament
- Runout sensor (`ENDSTOP_3`, PB7) goes **HIGH** → `has_filament = false`.
- Firmware enters **P5: Runout Logic (Tail Clearance)**.
- Motor does **NOT** stop. It continues running forward at maintenance speed to push the filament tail out of the internal gears.
- `DULIAO` stays **LOW** (still busy, tail is clearing).

#### Step 3 — Tail clears (Hall 3 triggers)
- Buffer arm drops to empty position → `buffer1_pos1_sensor_state` (Hall 3) goes **HIGH**.
- Motor **stops**.
- `update_run_latch(false)` → `run_latch = false`.
- `DULIAO` (PB15) goes **HIGH** → **"I'm done, next unit go!"**

#### Step 4 — Unit 2 wakes up
- Unit 2's main loop is running `motor_control()` every iteration.
- At **P4/Daisy Chain Signal Check** (line 356–373):
  - `digitalRead(FRONT_SIGNAL_PIN)` → reads **HIGH** (Unit 1 said "done").
  - `digitalRead(CHAIN_Y_SENSOR_PIN)` → reads **HIGH** (Y-path is clear, tail was cleared by Unit 1).
  - Both conditions met → `update_run_latch(true)` → Unit 2 is now active.
- Unit 2 enters **P6: Normal Auto Operation**, begins feeding.
- Unit 2's `DULIAO` stays **LOW** (busy).

#### Step 5 — Unit 2 runs out → triggers Unit 3
- Same as Steps 2–4, but with Unit 2 signaling Unit 3.

#### Step 6 — Unit 3 runs out → triggers Unit 1 (loop!)
- Unit 3's `DULIAO` goes HIGH → Unit 1's `FRONT_SIGNAL_PIN` reads HIGH.
- If you've reloaded filament on Unit 1, it starts again. **Infinite loop!**

### Timing / Delays

| Event | Duration | Source |
| :--- | :--- | :--- |
| Tail clearance feed | Variable (until Hall 3 triggers) | Motor runs at `VACTRUAL_VALUE / effective_div` speed |
| Chain handover detection | **< 1 loop iteration** (~microseconds) | Polled every `buffer_loop()` iteration |
| DULIAO HIGH → next unit starts | **< 1 loop iteration** (~microseconds) | Digital signal, no debounce needed |
| Feed timeout (safety) | **120 seconds** | `millis()` timer in P6 |
| Unload timeout (safety) | **120 seconds** | `millis()` timer in P0 |
| Watchdog timeout | **~10 seconds** | IWDG hardware timer |

### Why It's "Round-Robin" and Not Just "Next"
The wiring forms a **physical ring**: Unit N's output connects back to Unit 1's input. There is no "master controller" — each unit independently monitors its input pin and decides to start when it sees the signal. The chain can run indefinitely as long as at least one unit has filament loaded.

---

## ⚙️ Motor Control State Machine

`motor_control()` is the most complex function. It evaluates conditions in **strict priority order** (P0 being highest priority). The first matching condition wins.

```
┌────────────────┐
│ ENTRY          │
│ Read buttons   │
│ Read sensors   │
└───────┬────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│ Guard: Both Keys Pressed? → RETURN (combo handled above) │
└───────────────────┬───────────────────────────────────────┘
                    │ No
                    ▼
┌──────────────────────────────────────────────────────────┐
│ P0: Auto Unload Active?                                  │
│     └─ YES → Run motor REVERSE at full speed             │
│              → Stop if filament ejected OR 120s timeout   │
└───────────────────┬──────────────────────────────────────┘
                    │ No
                    ▼
┌──────────────────────────────────────────────────────────┐
│ P1: Manual Hold (button pressed, not yet long-held)?     │
│     └─ YES → STOP motor immediately (debounce safety)    │
└───────────────────┬──────────────────────────────────────┘
                    │ No
                    ▼
┌──────────────────────────────────────────────────────────┐
│ P2: Pulse Active (2s manual feed/reverse)?               │
│     └─ YES → Run motor in pulse direction                │
│              → Stop if buffer full (fwd) or 2s elapsed   │
└───────────────────┬──────────────────────────────────────┘
                    │ No
                    ▼
┌──────────────────────────────────────────────────────────┐
│ P3: Auto Mode Paused?                                    │
│     └─ YES → STOP motor, clear all latches               │
└───────────────────┬──────────────────────────────────────┘
                    │ No
                    ▼
┌──────────────────────────────────────────────────────────┐
│ P4: Chain Not Allowed (Slave waiting for signal)?        │
│     └─ YES → STOP motor, wait for FRONT_SIGNAL + Y_CLR  │
└───────────────────┬──────────────────────────────────────┘
                    │ No
                    ▼
┌──────────────────────────────────────────────────────────┐
│ P5: Filament Runout?                                     │
│     └─ Hall 3 triggered → STOP, signal DULIAO HIGH       │
│     └─ Hall 3 NOT yet   → FORWARD at maintenance speed   │
│                           (Tail clearance — keep pushing) │
└───────────────────┬──────────────────────────────────────┘
                    │ No
                    ▼
┌──────────────────────────────────────────────────────────┐
│ P6: Normal Auto Operation (Hysteresis)                   │
│     ├─ Hall 1 (Pos 3)  → REVERSE (buffer overfull)       │
│     ├─ Hall 3 (Pos 1)  → FORWARD full speed + LATCH      │
│     ├─ Hall 2 (Pos 2)  → STOP (buffer full)              │
│     └─ Neutral Zone:                                     │
│         ├─ filling_latch=T → FORWARD full speed           │
│         └─ filling_latch=F → FORWARD at maintenance speed │
│                              (or STOP if mode=0)          │
│     + Feed timeout: 120 s → ERROR + STOP                  │
└───────────────────┬──────────────────────────────────────┘
                    │
                    ▼
┌──────────────────────────────────────────────────────────┐
│ DRIVE_MOTOR:                                             │
│   If motor_state changed from last_motor_state:          │
│     → Forward: VACTUAL = -VACTRUAL_VALUE                 │
│     → Back:    VACTUAL = +VACTRUAL_VALUE                 │
│     → Stop:    VACTUAL = 0                               │
│   Safe direction change: always stops before reversing.  │
└──────────────────────────────────────────────────────────┘
```

### Hysteresis (Latching) Explained

The `filling_latch` flag prevents motor stutter in the neutral zone:

1. **Hall 3 triggers** (buffer empty) → Motor goes FORWARD at full speed, `filling_latch = true`.
2. **Buffer arm enters neutral zone** (no Hall active) → `filling_latch` keeps motor at full speed.
3. **Hall 2 triggers** (buffer full) → Motor STOPS, `filling_latch = false`.

Without this latch, the motor would slow down every time the arm crosses the neutral zone, causing inconsistent filament feeding.

---

## 📡 Sensor Logic & Pin Definitions

### Hall Sensors (Buffer Arm Position)

The buffer arm moves between three Hall sensor positions to indicate how much filament is buffered:

| Sensor | Pin | Variable | Meaning When HIGH |
| :--- | :--- | :--- | :--- |
| **Hall 3** | PB4 (`HALL1`) | `buffer1_pos1_sensor_state` | Buffer **Empty** — arm at bottom, needs more filament |
| **Hall 2** | PB3 (`HALL2`) | `buffer1_pos2_sensor_state` | Buffer **Full** — arm at top, stop feeding |
| **Hall 1** | PB2 (`HALL3`) | `buffer1_pos3_sensor_state` | Buffer **Overfull** — too much filament, reverse motor |

> **⚠️ Note on naming:** The code maps `HALL1` (PB2) to `pos3` and `HALL3` (PB4) to `pos1`. This is intentional — `pos1` means "position 1 = empty", `pos3` means "position 3 = overfull".

### Other Sensors

| Sensor | Pin | Active State | Description |
| :--- | :--- | :--- | :--- |
| **Filament Runout** | PB7 (`ENDSTOP_3`) | LOW = Loaded, HIGH = Empty | Detects if filament is physically present in the spool holder. |
| **Y-Splitter Sensor** | PB14 (`EXTENSION_PIN7`) | HIGH (~4.6 V) = Clear, LOW = Blocked | **Shared across all units.** Prevents a new unit from feeding when the Y-path is blocked by the previous unit's filament tail. 5 V tolerant pin. |
| **Daisy Input** | PB5 (`FRONT_SIGNAL_PIN`) | HIGH = Previous done, LOW = Previous busy | Signal from the previous unit in the chain. Internal pull-up enabled. |

### Output Pins

| Pin | Name | Purpose |
| :--- | :--- | :--- |
| PB15 | `DULIAO` | **Daisy Output** — Signal to next unit. LOW = busy, HIGH = done/empty. |
| PA1 | `DUANLIAO` | Legacy/debug output. Set HIGH at init. |
| PA15 | `ERR_LED` | Red LED — error and config feedback. |
| PB0 | `START_LED` | Blue LED — status indicator. |

---

## 🔧 Wiring Guide

### Pin Definitions

| Function | Pin Name | STM32 Pin | Note |
| :--- | :--- | :--- | :--- |
| **Daisy Output** | `DULIAO` | **PB15** | Connect to NEXT unit's Input. |
| **Daisy Input** | `FRONT_SIGNAL_PIN` | **PB5** | Connect to PREVIOUS unit's Output. |
| **Y-Sensor** | `EXTENSION_PIN7` | **PB14** | **5V Tolerant.** Shared across all units. |
| **Runout Sensor** | `ENDSTOP_3` | **PB7** | Internal Filament Sensor. |

### ⛓️ Daisy Chain Wiring (3-Unit Example)

To create an infinite loop, wire the signal pins in a circle and share the Y-Sensor. **All boards must share a common Ground (GND).**

1.  **Signal Loop (The Relay):**
    *   **Unit 1 [PB15]** → **Unit 2 [PB5]**
    *   **Unit 2 [PB15]** → **Unit 3 [PB5]**
    *   **Unit 3 [PB15]** → **Unit 1 [PB5]** *(The Loop Back)*

2.  **Y-Splitter Sensor (The Safety):**
    *   Connect the Y-Splitter Switch Signal to **Pin PB14** on **ALL** units (Parallel connection).
    *   Connect Switch GND to **GND** on **ALL** units.
    *   *Logic:* High (4.6V) = Clear/Go. Low (0V) = Blocked/Wait.

```
       ┌──────────────────────────────────────────────────────────┐
       │                    Y-Sensor (PB14)                       │
       │              Shared Bus (All Units Parallel)             │
       └───────┬──────────────────┬──────────────────┬────────────┘
               │                  │                  │
        ┌──────▼──────┐   ┌──────▼──────┐   ┌──────▼──────┐
        │   UNIT 1    │   │   UNIT 2    │   │   UNIT 3    │
        │             │   │             │   │             │
        │ PB15 (OUT) ─┼──>│ PB5  (IN)  │   │             │
        │             │   │ PB15 (OUT) ─┼──>│ PB5  (IN)  │
        │ PB5  (IN) <─┼───┼─────────────┼───┤ PB15 (OUT) │
        └─────────────┘   └─────────────┘   └─────────────┘
              GND ──────────── GND ──────────── GND
```

---

## 🎮 Controls & UI

### Button Functions

| Button | Action | Duration | Function |
| :--- | :--- | :--- | :--- |
| **Key 2 (Feed)** | Click | < 2 s | **Pulse Forward:** Feeds filament for 2 s (stops early if Hall 2 or Hall 1 triggers). |
| **Key 2 (Feed)** | Hold | > 1 s | **Manual Chain Start:** Sets `run_latch = true`, enabling auto-feed. This is how you **start the chain** on the first unit. |
| **Key 1 (Back)** | Click | < 2 s | **Pulse Reverse:** Reverses motor for 2 s. Clears latch and cancels auto-unload. |
| **Key 1 (Back)** | Hold | 3 s | **Pause/Resume:** Toggles `auto_mode`. When paused, motor stops and latch is cleared. Stops the entire chain. |
| **Key 1 (Back)** | Hold | 5 s | **Auto-Unload:** Latches motor in reverse until filament sensor goes HIGH (ejected) or 120 s timeout. |
| **Key 1 + 2** | Hold | N/A | **Config Mode:** Cycles through Speed/Slave modes (see below). Motor continues running during config. |

### LED Status Indicators

| Color | Pattern | Meaning |
| :--- | :--- | :--- |
| 🔵 **Blue** | **Solid** | Filament Loaded & Active (feeding or ready). |
| 🔵 **Blue** | **Fast Blinking** (200 ms) | **Runout:** Filament Empty — waiting for chain signal or reload. |
| 🔵 **Blue** | **Slow Blinking** (1 s) | **Slave Waiting:** Mode 6, loaded, but `run_latch = false` — waiting for previous unit to finish. |
| 🔴 **Red** | **Toggle** | Toggled on 3 s hold (pause) or 5 s hold (unload) as confirmation. |
| 🔴 **Red** | **Count Blink** | **Config Mode:** Indicates current mode number (see below). |

---

## ⚙️ Configuration Modes

Hold **Key 1 + Key 2** together to cycle through modes. The Red LED will flash to indicate the new setting. The mode is saved to EEPROM and persists across power cycles.

| Mode | `maintenance_divider` | Red LED Flash | Description |
| :--- | :--- | :--- | :--- |
| **0** | `0` | 1 Long Flash (1 s) | **Factory Mode:** Motor stops completely in the neutral zone. No maintenance feed. |
| **1** | `1` | 1 Blink | **Fastest Maintenance:** Neutral zone speed = `VACTRUAL_VALUE / 1` (full speed). |
| **2** | `2` | 2 Blinks | Neutral zone speed = `VACTRUAL_VALUE / 2` (50%). |
| **3** | `3` | 3 Blinks | **Default.** Neutral zone speed = `VACTRUAL_VALUE / 3` (~33%). |
| **4** | `4` | 4 Blinks | Neutral zone speed = `VACTRUAL_VALUE / 4` (25%). |
| **5** | `5` | 5 Blinks | **Slowest Maintenance:** Neutral zone speed = `VACTRUAL_VALUE / 5` (20%). |
| **6** | `6` | 6 Fast Blinks (80 ms) | **🔗 SLAVE MODE:** Enables Daisy Chain Logic. Unit waits for input signal before starting. Uses Mode 3 speed internally for neutral zone and tail clearance. |

### How to Start the Chain
1.  Set all units to **Mode 6** (Slave Mode).
2.  Load filament into all units. (Blue LEDs should be Solid or Slow Blinking).
3.  Hold **Key 2 (Feed)** for >1 s on the **First Unit** you want to start (or click to pulse-start).
4.  The chain will now run infinitely. When a spool empties, the next unit takes over automatically.

---

## 🛡️ Safety Failsafes & Timeouts

| Failsafe | Timeout | Trigger | Action |
| :--- | :--- | :--- | :--- |
| **Feed Timeout** | 120 s | Motor running forward continuously for >120 s without Hall 2 triggering. | Motor stops, `auto_mode = false`, `is_error = true`. |
| **Unload Timeout** | 120 s | Auto-unload running for >120 s without filament sensor going HIGH. | Motor stops, `auto_unload_active = false`, `is_error = true`. |
| **IWDG (Watchdog)** | ~10 s | Main loop has not incremented `g_run_cnt` in ~5 s. | Timer ISR detects stall, enters infinite loop → IWDG bites → MCU reset. |
| **Y-Sensor Interlock** | N/A | `CHAIN_Y_SENSOR_PIN` reads LOW (blocked). | Slave unit will not start even if `FRONT_SIGNAL_PIN` is HIGH. Prevents filament collision. |
| **Direction Change Safety** | N/A | Motor being commanded to reverse direction. | `VACTUAL(0)` is sent first to stop the motor before reversing. Protects TMC2209 driver and gears. |

---

## 💻 Serial (USB) Commands

Connect via USB at **115200 baud**. Send commands terminated with `\n` (newline).

| Command | Example | Description |
| :--- | :--- | :--- |
| `rt` | `rt` | Prints current timeout value (ms). |
| `timeout <ms>` | `timeout 90000` | Sets feed/error timeout in milliseconds. Saved to EEPROM. |
| `speed <rpm>` | `speed 300` | Sets motor speed in RPM. Recalculates `VACTRUAL_VALUE`. Saved to EEPROM. Valid range: 0–1000. |
| `info` | `info` | Prints current timeout and maintenance_divider settings. |

### Speed Calculation Formula

```
VACTRUAL_VALUE = SPEED × Move_Divide_NUM × 200 / 60 / 0.715
```

Where:
- `SPEED` = RPM (default: 260)
- `Move_Divide_NUM` = Microsteps (default: 16)
- `200` = Steps per revolution (1.8° stepper)
- `0.715` = TMC2209 VACTUAL conversion factor

---

## 📥 Installation & Flashing

### Requirements
- [VS Code](https://code.visualstudio.com/) with [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) extension
- USB cable (the board uses DFU upload protocol)

### Build & Flash

1. Open the project folder in VS Code / PlatformIO.
2. Verify pin definitions in `lib/buffer/buffer.h` match your board hardware.
3. Build: **PlatformIO → Build** (or `pio run`).
4. Enter DFU mode on the board (hold BOOT button while pressing RESET).
5. Flash: **PlatformIO → Upload** (or `pio run --target upload`).

### Build Flags (from `platformio.ini`)

| Flag | Purpose |
| :--- | :--- |
| `-D USBCON` | Enable USB communication |
| `-D USBD_USE_CDC` | USB CDC (Serial over USB) |
| `-D HSE_VALUE=8000000L` | External oscillator = 8 MHz |
| `-D USE_HSE` | Use external high-speed oscillator |
| `-D TMC2208_BAUDRATE=9600` | TMC driver UART baud rate |

---

## 🔍 Debug Checklist

See [DEBUG_CHECKLIST.md](DEBUG_CHECKLIST.md) for a detailed hardware verification guide including:

1. **Pin Verification** — Confirm EN/STEP/DIR/UART/Signal pins match your board revision.
2. **Sensor Logic Levels** — Expected voltage levels for runout, Y-sensor, and daisy chain signals.
3. **Pre-Flight Tests** — Step-by-step functional verification (power on → load → manual feed → chain handover → tail clearance).

---

## 📋 Logic Analysis Summary

The firmware logic has been verified and is correctly structured:

- ✅ **Priority ordering is correct** — Safety actions (P0–P1) always override normal operation (P5–P6).
- ✅ **Hysteresis latch prevents stutter** — `filling_latch` ensures continuous feeding through the neutral zone.
- ✅ **Tail clearance works** — Runout does NOT immediately stop; it pushes until Hall 3 to clear the Y-path.
- ✅ **Chain handover requires dual condition** — Both `FRONT_SIGNAL_PIN` (previous done) AND `Y_SENSOR_PIN` (path clear) must be HIGH.
- ✅ **Direction reversal is safe** — Motor always stops (`VACTUAL(0)`) before changing direction.
- ✅ **Watchdog prevents hangs** — IWDG + timer ISR detect main loop stalls.
- ✅ **EEPROM persistence** — Mode, speed, timeout, and slave state survive power cycles.
- ✅ **Overflow-safe timers** — All `millis()` comparisons use subtraction (`now - start > threshold`), which is unsigned overflow safe.

---

## 📄 License

See [LICENSE](LICENSE) for details.

---

## Firmware file ready to flash will be uploaded as well!

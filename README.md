aliexpress.com/item/1005008820016221.html
---

# Fly Buffer LLLP - Smart Daisy-Chain Firmware (V1.2)

This custom firmware for the **Mellow Fly Buffer (LLLP)** board completely overhauls the logic to allow for **Round-Robin Continuous Feeding**. It enables multiple buffer units to be daisy-chained together, automatically switching to the next spool when one runs empty, creating an infinite filament loop for 3D printing.

## 🌟 Key Features

### 1. 🔄 Infinite Round-Robin Logic (Daisy Chain)
Link multiple LLLP units together (e.g., 1 → 2 → 3 → 1). When Unit 1 runs out, it signals Unit 2 to take over. When Unit 2 runs out, it triggers Unit 3. Unit 3 loops back to trigger Unit 1 (after you've reloaded it).

### 2. 🧠 Smart Hysteresis Feeding
Unlike the factory "Bang-Bang" control, this firmware uses a latching logic:
*   **Trigger:** Starts feeding at full speed when the buffer is empty (Hall 3).
*   **Latch:** *Continues* feeding through the neutral zone with the speed you choose (this is the slower maintainance speed).
*   **Stop:** Stops only when the buffer is full (Hall 2).
*   **Result:** Smoother operation and less motor stutter.

### 3. ✂️ Runout Tail Clearance
When the filament runs out, the motor **does not stop immediately**. It continues the feeding loop until the filament tail clears the internal gears (detected by the buffer arm dropping to Hall 3). This ensures the Y-Splitter path is clear for the next unit to take over.

### 4. ⚙️ On-the-Fly Configuration
Change maintenance speeds or switch between Master/Slave modes instantly by holding both buttons. The motor continues running while you configure, ensuring no interruption to the print.

### 5. 🛡️ Robust Safety Failsafes
*   **Feed Timeout:** If the buffer tries to fill for >120s without success, it stops to prevent grinding.
*   **Unload Timeout:** Prevents endless reversing if the filament sensor fails.
*   **Y-Sensor Interlock:** A shared sensor prevents a new unit from starting if the previous unit's filament is still blocking the path.

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
    *   **Unit 1 [PB15]** $\rightarrow$ **Unit 2 [PB5]**
    *   **Unit 2 [PB15]** $\rightarrow$ **Unit 3 [PB5]**
    *   **Unit 3 [PB15]** $\rightarrow$ **Unit 1 [PB5]** *(The Loop Back)*

2.  **Y-Splitter Sensor (The Safety):**
    *   Connect the Y-Splitter Switch Signal to **Pin PB14** on **ALL** units (Parallel connection).
    *   Connect Switch GND to **GND** on **ALL** units.
    *   *Logic:* High (4.6V) = Blocked/Wait. Low (0V) = Clear/Go.

---

## 🎮 Controls & UI

### Button Functions

| Button | Action | Duration | Function |
| :--- | :--- | :--- | :--- |
| **Key 2 (Feed)** | Click | < 2s | **Pulse Forward:** Feeds for 2s. **Manually Starts Chain/Latch.** |
| **Key 1 (Back)** | Click | < 2s | **Pulse Reverse:** Reverses for 2s. |
| **Key 1 (Back)** | Hold | 3 sec | **Pause/Resume:** Toggles Auto-Feed Loop. Stops the Chain. |
| **Key 1 (Back)** | Hold | 5 sec | **Auto-Unload:** Latches reverse until filament is ejected (or timeout). |
| **Key 1 + 2** | Hold | N/A | **Config Mode:** Cycles through Speed/Slave modes (see below). |

### LED Status Indicators

| Color | Pattern | Meaning |
| :--- | :--- | :--- |
| 🔵 **Blue** | **Solid** | Filament Loaded & Ready. |
| 🔵 **Blue** | **Blinking** | **Runout:** Filament Empty. |
| 🔴 **Red** | **Fast Blink** | **Error:** Blockage detected or Timeout. |
| 🔴 **Red** | **Count Blink** | **Config Mode:** Indicates current Mode setting (see below). |

---

## ⚙️ Configuration Modes

Hold **Key 1 + Key 2** together to cycle through modes. The Red LED will flash to indicate the new setting.

| Mode | Red LED Flash | Description |
| :--- | :--- | :--- |
| **0** | 1 Long Flash | **Factory Mode:** Motor stops completely in the neutral zone (No maintenance feed). |
| **1-5** | 1 to 5 Flashes | **Maintenance Speed:** Sets the idle feed speed (1=Fastest, 5=Slowest). |
| **6** | 6 Fast Flashes | **🔗 SLAVE MODE:** Enables Daisy Chain Logic. Unit waits for signal before starting. |

### How to Start the Chain
1.  Set all units to **Mode 6** (Slave Mode).
2.  Load filament into all units. (Blue LEDs should be Solid and filament past the feeding cog).
3.  Press **Key 2 (Feed)** once on the **First Unit** you want to start.
4.  The chain will now run infinitely. When a spool empties, the next unit takes over automatically.

---

## 📥 Installation
1.  Open the project in VS Code / PlatformIO.
2.  Ensure `buffer.h` pin definitions match the wiring table above.
3.  Build and Upload to the STM32F0 on the Fly Buffer board.

## Firmware file ready to flash will be uploaded as well!

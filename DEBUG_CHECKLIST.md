# Fly Buffer (LLLP) - V3.0 Debug & Verification Checklist

## 1. Hardware Pin Verification (CRITICAL)
Before running the firmware, you **MUST** verify the following pin definitions match your actual hardware. The code uses standard/assumed pins which might differ from your board revision.

| Signal | Code Definition | Description | Verify? |
| :--- | :--- | :--- | :--- |
| **UART Pin** | `Serial1` | TMC2209 UART | [ ] |
| **FRONT_SIG** | `PB5` | Input from Prev Unit | [ ] |
| **DULIAO** | `PB15` | Output to Next Unit | [ ] |
| **Y-SENSOR** | `PB14` | Shared Line | [ ] |

**Action:** Open `lib/buffer/buffer.h` and update the `#define` lines if your board uses different pins.

---

## 2. Sensor Logic Levels

Measure voltage between the Signal Pin and GND to verify states.

### A. Filament Runout Sensor (`ENDSTOP_3` / PB7)
*   **Loaded (Filament Present):** 0V (Low)
*   **Empty (No Filament):** 3.3V (High)
*   *Note: If reversed, check the sensor wiring (NO vs NC).*

### B. Y-Splitter Sensor (`EXTENSION_PIN7` / PB14)
*   **Blocked (Filament Present):** 0V (Low)
*   **Clear (Empty Path):** ~4.6V (High) - *Must be > 2.0V*
*   *Note: This signal is shared. All units must show High when clear.*

### C. Daisy Chain Signal
**Measure at `FRONT_SIGNAL_PIN` (PB5) on Unit 2 (Slave):**
*   **Previous Unit Busy:** 0V (Low)
*   **Previous Unit Done:** 3.3V (High)

---

## 3. Pre-Flight Functional Test

1.  **Power On (No Filament):**
    *   **LEDs:** Start LED (Blue) should be **OFF**. Err LED (Red) should be **OFF**.
    *   **Behavior:** Motor should NOT move.

2.  **Load Filament:**
    *   Insert filament into Runout Sensor.
    *   **LEDs:** Start LED (Blue) turns **SOLID ON**.
    *   **Behavior:** Unit is now "Armed".

3.  **Manual Feed:**
    *   Click **Key 2 (Feed)**.
    *   Motor should pulse Forward for 2 seconds.

4.  **Daisy Chain Handover (Simulated):**
    *   **Setup:** Set Unit to Mode 6 (Slave). (Hold both buttons, cycle until 6 blinks).
    *   **Initial State:** Unit is Loaded (Solid Blue or Heartbeat Blue). Motor Stopped.
    *   **Trigger:** Use a jumper wire to connect `FRONT_SIGNAL_PIN` (PB5) to 3.3V (Simulate Prev Unit Done).
    *   **Condition:** Ensure Y-Sensor is High (Clear).
    *   **Result:** Unit should START feeding (Solid Blue).

5.  **Tail Clearance Test:**
    *   While feeding, cut the filament before the Runout Sensor.
    *   **Behavior:** Runout Sensor goes High (Empty).
    *   **Result:** Motor **MUST CONTINUE** running Forward.
    *   **Stop:** Trigger Hall 3 (Empty Position) manually. Motor should STOP.

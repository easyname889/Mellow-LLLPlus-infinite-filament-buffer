#ifndef BUFFER_H
#define BUFFER_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TMCStepper.h>
#include <EEPROM.h>

// -------------------------------------------------------------------------
// PIN DEFINITIONS
// -------------------------------------------------------------------------

// Sensors
#define HALL1               PB2
#define HALL2               PB3
#define HALL3               PB4
#define ENDSTOP_3           PB7
#define EXTENSION_PIN7      PB14 // Y-Sensor
#define CHAIN_Y_SENSOR_PIN  EXTENSION_PIN7

// Buttons
#define KEY1                PB13
#define KEY2                PB12

// LEDs
#define ERR_LED             PA15
#define START_LED           PB0

// Chain Communication
#define FRONT_SIGNAL_PIN    PB5  // Input from prev unit
#define DULIAO              PB15 // Output to next unit
#define DUANLIAO            PA1  // Legacy / Debug?

// Pulse / MDM
#define MDM_DPIN            PB9
#define BACK_SIGNAL_PIN     MDM_DPIN
#define PULSE2_PIN          PA3
#define SIGNAL_COUNT_PUL_Pin       GPIO_PIN_1 // Placeholder
#define SIGNAL_COUNT_PUL_GPIO_Port GPIOA      // Placeholder
#define SIGNAL_COUNT_DIR_Pin       GPIO_PIN_1 // Placeholder
#define SIGNAL_COUNT_DIR_GPIO_Port GPIOB      // Placeholder

// Motor Driver Pins (Placeholders - Verify Hardware!)
#ifndef EN_PIN
#define EN_PIN              PA10
#endif
#ifndef STEP_PIN
#define STEP_PIN            PA9
#endif
#ifndef DIR_PIN
#define DIR_PIN             PA8
#endif

// TMC2209 Config
#define DRIVER_ADDRESS      0b00
#define R_SENSE             0.11f
#define UART                Serial1

// -------------------------------------------------------------------------
// STRUCTS & ENUMS
// -------------------------------------------------------------------------

struct Buffer {
    uint8_t buffer1_pos1_sensor_state;
    uint8_t buffer1_pos2_sensor_state;
    uint8_t buffer1_pos3_sensor_state;
    uint8_t buffer1_material_swtich_state; // Kept typo for compatibility with V1.0 logic (will fix in V3.0)
    uint8_t key1;
    uint8_t key2;
};

enum Motor_State {
    Stop,
    Forward,
    Back
};

// -------------------------------------------------------------------------
// GLOBALS (Externs)
// -------------------------------------------------------------------------
extern TMC2209Stepper driver;
extern Buffer buffer;
extern Motor_State motor_state;

extern bool is_front;
extern uint32_t front_time;
extern uint32_t timeout;
extern bool is_error;
extern String serial_buf;
extern float SPEED;
extern uint32_t VACTRUAL_VALUE;
extern uint32_t Move_Divide_NUM;
extern uint32_t I_CURRENT;

extern uint8_t maintenance_divider;
extern bool auto_mode;
extern bool run_latch;
extern bool visual_override;

// -------------------------------------------------------------------------
// FUNCTION PROTOTYPES
// -------------------------------------------------------------------------
void buffer_init();
void buffer_loop();
void read_sensor_state();
void motor_control();
void buffer_sensor_init();
void buffer_motor_init();
void iwdg_init();
void timer_it_callback();
void Signal_Dir_Init();
void USB_Serial_Analys();
void Dir_IT_Callback();
void key1_it_callback();
void key2_it_callback();
float fastAtof(const char *s);

#endif

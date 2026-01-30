#ifndef BUFFER_H
#define BUFFER_H

#include <Arduino.h>
#include <TMCStepper.h>
#include <EEPROM.h>
#include <HardwareTimer.h>
#include <stm32f0xx.h>

// --- PIN DEFINITIONS ---
// Hall Sensors
#define HALL1             PB2
#define HALL2             PB3
#define HALL3             PB4

// Filament Sensors
#define ENDSTOP_3         PB7   // Internal Filament Sensor
#define MDM_DPIN          PB9   // MDM Breakage Pin
#define PULSE2_PIN        PA3   // MDM Pulse Pin
// But based on V1.0 code, HALL1 is PB2 and MDM pulse is also on a pin that uses interrupts.

// Buttons
#define KEY1              PB13
#define KEY2              PB12

// LEDs
#define ERR_LED           PA15
#define START_LED         PB0
#define DUANLIAO          PA1   // Filament runout output to printer
#define DULIAO            PB15  // Daisy chain output to next unit

// Daisy Chain Signals
#define FRONT_SIGNAL_PIN  PB5
#define BACK_SIGNAL_PIN   PB8

// Y-Sensor
#define CHAIN_Y_SENSOR_PIN PB14 // EXTENSION_PIN7

// Extension Pins (general purpose)
#define EXTENSION_PIN1    PA0
#define EXTENSION_PIN2    PA1
#define EXTENSION_PIN7    PB14

// Motor Driver (TMC2209)
#define EN_PIN            PA8
#define STEP_PIN          PB1
#define DIR_PIN           PB6

// --- TMC2209 SETTINGS ---
#define R_SENSE           0.11f
#define DRIVER_ADDRESS    0b00
#define UART              Serial1
#define I_CURRENT         800
#define Move_Divide_NUM   16

// --- HAL MACRO SIMULATION ---
#define SIGNAL_COUNT_DIR_GPIO_Port GPIOB
#define SIGNAL_COUNT_DIR_Pin       GPIO_PIN_6
#define SIGNAL_COUNT_PUL_GPIO_Port GPIOB
#define SIGNAL_COUNT_PUL_Pin       GPIO_PIN_1
#define SIGNAL_COUNT_PUL_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define SIGNAL_COUNT_TIM_CLK_ENABLE()  __HAL_RCC_TIM2_CLK_ENABLE()
#define SIGNAL_COUNT_Get_TIM       TIM2
#define SIGNAL_COUNT_Get_HTIM      htim2

// --- ENUMS & STRUCTS ---
enum Motor_State {
    Forward,
    Back,
    Stop
};

struct Buffer {
    uint8_t buffer1_pos1_sensor_state;
    uint8_t buffer1_pos2_sensor_state;
    uint8_t buffer1_pos3_sensor_state;
    uint8_t buffer1_material_swtich_state;
    uint8_t key1;
    uint8_t key2;
};

struct BlockageDetect {
    bool blockage_flag;
    uint32_t mdm_pulse_cnt;
    float actual_distance;
    float target_distance;
    uint32_t last_pulse_cnt;
    uint32_t pulse_cnt;
    int32_t pulse_cnt_sub;
    int32_t extrusion_pulse_cnt;
    float distance_error;
    float allow_error;
};

// --- GLOBAL VARIABLES ---
extern TMC2209Stepper driver;
extern Buffer buffer;
extern Motor_State motor_state;
extern uint32_t VACTUAL_VALUE;
extern float SPEED;
extern uint32_t timeout;
extern bool connet_mdm_flag;
extern BlockageDetect blockage_detect;
extern uint32_t blockage_inform_times;
extern uint8_t maintenance_divider;
extern bool auto_mode;
extern bool is_error;
extern bool is_front;
extern uint32_t front_time;

// --- FUNCTION PROTOTYPES ---
void buffer_init();
void buffer_loop();
void buffer_sensor_init();
void buffer_motor_init();
void read_sensor_state();
void motor_control();
void timer_it_callback();
void key1_it_callback();
void key2_it_callback();
void Blockage_Detect();
void USB_Serial_Analys();
bool Check_Connet_MDM();
void Pulse_Receive_Init();
void Signal_Dir_Init();
void REIN_TIM_SIGNAL_COUNT_Init();
void REIN_TIM_SIGNAL_COUNT_DeInit();
float fastAtof(const char *s);

#endif

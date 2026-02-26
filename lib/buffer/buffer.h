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
#define HALL1       PB2 //光感3
#define HALL2       PB3 //光感2
#define HALL3       PB4 //光感1
#define ENDSTOP_3           PB7

// Buttons
#define KEY1                PB13
#define KEY2                PB12

// LEDs
#define ERR_LED             PA15
#define START_LED           PA8

// Chain Communication
#define FRONT_SIGNAL_PIN    PB5  // Input from prev unit
#define BACK_SIGNAL_PIN     PB6
#define DULIAO              PB15 // Output to next unit
#define DUANLIAO            PB15

// Extensions from original code
#define EXTENSION_PIN1 PA2
#define EXTENSION_PIN2 PA3
#define EXTENSION_PIN3 PB11
#define EXTENSION_PIN4 PB10
#define EXTENSION_PIN5 PA5
#define EXTENSION_PIN6 PA4
#define EXTENSION_PIN7 PB14
#define CHAIN_Y_SENSOR_PIN  EXTENSION_PIN7

// Motor Driver Pins
#define EN_PIN      PA6
#define DIR_PIN     PA7
#define STEP_PIN    PC13
#define UART        PB1

// MDM / Blockage
#define PULSE1_PIN EXTENSION_PIN5
#define PULSE2_PIN EXTENSION_PIN4
#define MDM_DPIN EXTENSION_PIN6

// SIGNAL_COUNT Macros from original
#define SIGNAL_COUNT_DIR_CLK_ENABLE()		__HAL_RCC_GPIOB_CLK_ENABLE()
#define SIGNAL_COUNT_DIR_GPIO_Port			(GPIOB)
#define SIGNAL_COUNT_DIR_Pin				(GPIO_PIN_11)
#define SIGNAL_COUNT_DIR_Get_IRQn			(EXTI4_15_IRQn)

#define SIGNAL_COUNT_PUL_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()
#define SIGNAL_COUNT_PUL_GPIO_Port			(GPIOA)
#define SIGNAL_COUNT_PUL_Pin						(GPIO_PIN_5)
#define SIGNAL_COUNT_TIM_CLK_ENABLE()		__HAL_RCC_TIM2_CLK_ENABLE()
#define	SIGNAL_COUNT_Get_TIM						(TIM2)
#define	SIGNAL_COUNT_Get_HTIM						(htim2)


// TMC2209 Config
#define DRIVER_ADDRESS      0b00
#define R_SENSE             0.11f

// -------------------------------------------------------------------------
// GLOBALS & STRUCTS
// -------------------------------------------------------------------------
// Speed Globals (will be externed in cpp)
extern int32_t SPEED;
extern int32_t VACTRUAL_VALUE;
#define Move_Divide_NUM			((int32_t)(64))
#define I_CURRENT (500)
#define WRITE_EN_PIN(x) digitalWrite(EN_PIN,x)
#define FORWARD		1
#define BACK		0
#define STOP        0

struct Buffer {
    bool buffer1_pos1_sensor_state;
    bool buffer1_pos2_sensor_state;
    bool buffer1_pos3_sensor_state;
    bool buffer1_material_swtich_state;
    bool key1;
    bool key2;
};

enum Motor_State {
    Forward=0,
    Stop,
    Back
};

typedef struct BlockageDetect
{
	int32_t target_distance; //目标距离
	int32_t actual_distance; //实际距离
	int32_t distance_error; //距离误差
	float allow_error; //允许误差
	bool blockage_flag; //堵料标志位

	int32_t mdm_pulse_cnt=0;//接收到段堵料模块发送的脉冲数
	int16_t last_pulse_cnt=0;//上次脉冲数
	int16_t pulse_cnt=0;//接收到主控板发送的脉冲数
	int16_t pulse_cnt_sub=0;//脉冲差值
	int32_t extrusion_pulse_cnt=0;//挤出脉冲数
	float encoder_length; //编码器长度

}BlockageDetect;

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
void USB_Serial_Analys();
void key1_it_callback();
void key2_it_callback();
float fastAtof(const char *s);
void Recv_MDM_Pulse_IT_Callback(void);
void Dir_IT_Callback(void);
void buffer_debug(void);

extern TMC2209Stepper driver;
extern Buffer buffer;
extern Motor_State motor_state;
extern bool is_front;
extern uint32_t front_time;
extern uint32_t timeout;
extern bool is_error;
extern String serial_buf;
extern uint32_t steps;
extern uint8_t maintenance_divider;
extern bool auto_mode;
extern bool run_latch;
extern bool visual_override;
extern bool connet_mdm_flag;
extern BlockageDetect blockage_detect;

#endif

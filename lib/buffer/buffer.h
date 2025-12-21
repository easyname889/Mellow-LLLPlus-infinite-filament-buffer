/**
  ***************************************************************************************
  * @file    buffer.cpp
  * @author  lijihu (Modified by Assistant v2.8 Final)
  * @version V2.8.0
  * @date    2025/05/10
  * @brief   Fly Buffer Firmware - Round Robin Edition
  *
  * @note    --- FIRMWARE V2.8 FEATURE SUMMARY ---
  *
  *          1. MODES (Set by holding both buttons):
  *             - Mode 0: Factory Mode. Motor stops in the neutral zone. (1 LONG Red Blink)
  *             - Mode 1-5: Maintenance Speeds. Motor runs slowly in neutral zone. (1-5 Red Blinks)
  *             - Mode 6: Chain Slave Mode. Waits for external trigger. (6 FAST Red Blinks)
  *
  *          2. BUTTON LOGIC:
  *             - KEY 1 (Reverse):
  *               - Click (<2s):  Pulse motor in reverse for 2 seconds.
  *               - Hold (3s):    Toggle Auto-Feed Loop ON/OFF (Pause/Resume).
  *               - Hold (5s):    Start Auto-Unload. Motor reverses until filament is clear.
  *
  *             - KEY 2 (Feed):
  *               - Click (<2s):  Pulse motor forward for 2 seconds. In Chain Mode, this STARTS the loop.
  *
  *             - KEY 1 + KEY 2:
  *               - Hold:         Cycle through Modes 0-6 on-the-fly without stopping the motor.
  *
  *          3. LED STATUS (V2.8 Update):
  *             - Filament Loaded: SOLID BLUE (Red Off).
  *             - Filament Empty:  BLINKING BLUE (Red Off).
  *             - Blockage/Error:  FAST BLINKING RED (Blue Off).
  *             - Config Mode:     RED (blinks to show selected mode).
  *
  *          4. DAISY CHAIN WIRING (1 -> 2 -> 3 -> 1 Round Robin):
  *             - SIGNAL:
  *               - Unit 1 DULIAO (PB15) -> Unit 2 FRONT_SIGNAL_PIN (PB5)
  *               - Unit 2 DULIAO (PB15) -> Unit 3 FRONT_SIGNAL_PIN (PB5)
  *               - Unit 3 DULIAO (PB15) -> Unit 1 FRONT_SIGNAL_PIN (PB5)
  *             - Y-SENSOR (Shared):
  *               - Sensor Signal (4.6V) -> CHAIN_Y_SENSOR_PIN (PB14) on ALL units.
  *               - Sensor GND -> GND on ALL units.
  *               - NOTE: PB14 is 5V TOLERANT (safe for 4.6V).
  *             - GROUND:
  *               - All units must share a common ground.
  ***************************************************************************************
**/

#include "buffer.h"

// GPIO/TIM Macros
#define SIGNAL_COUNT_READ_DIR_IO()    (SIGNAL_COUNT_DIR_GPIO_Port -> IDR & SIGNAL_COUNT_DIR_Pin)
#define SIGNAL_COUNT_READ_COUNT()     (SIGNAL_COUNT_Get_TIM -> CNT)
#define SIGNAL_COUNT_UP()             (SIGNAL_COUNT_Get_TIM -> CR1 &= ~(TIM_CR1_DIR))
#define SIGNAL_COUNT_DOWN()           (SIGNAL_COUNT_Get_TIM -> CR1 |=  (TIM_CR1_DIR))

#include "stm32f0xx_hal_iwdg.h"

// Hardware Pin Defs
#define CHAIN_Y_SENSOR_PIN EXTENSION_PIN7 // PB14 (5V Tolerant)

TMC2209Stepper driver(UART, UART, R_SENSE, DRIVER_ADDRESS);
Buffer buffer={0}; 
Motor_State motor_state=Stop;
static Motor_State last_motor_state=Stop;

bool is_front=false; 
uint32_t front_time=0; 
const uint32_t DEFAULT_TIMEOUT = 60000;
uint32_t timeout=60000; 
bool is_error=false; 
String serial_buf;

static HardwareTimer timer(TIM6);
TIM_HandleTypeDef htim2;

// Button State
bool key1_press_flag=false;
bool key2_press_flag=false;
bool key1_release_flag=false;
bool key2_release_flag=false;
uint32_t key1_press_times=0;
uint32_t key2_press_times=0;
uint32_t key1_release_times=0;
uint32_t key2_release_times=0;
uint8_t key1_press_cnt=0;
uint8_t key2_press_cnt=0;

uint32_t inform_flag=false;
uint32_t inform_times=0;

// Motor/Encoder Settings
const uint32_t DEFAULT_STEPS = 916;
uint32_t steps=916; 
BlockageDetect blockage_detect={0}; 
bool connet_mdm_flag=false;
uint32_t blockage_inform_times=0;

const float DEFAULT_ENCODER_LENGTH = 1.73;
float encoder_length=1.73; 
const float DEFAULT_ALLOW_ERROR_SCALE = 2;
float allow_error_scale=2; 

// EEPROM Map
const int EEPROM_ADDR_TIMEOUT = 0;
const int EEPROM_ADDR_STEPS = 4;
const int EEPROM_ADDR_ENCODER_LENGTH = 8;
const int EEPROM_ADDR_ERROR_SCALE = 12;
const int EEPROM_ADDR_SPEED = 16;
const int EEPROM_ADDR_MAINT_DIV = 20; 

// Logic Globals
uint8_t maintenance_divider = 3; 
bool auto_mode = true;           
bool run_latch = false; // V2.6: Keeps unit running after start signal is lost

// Visuals
uint32_t lastToggleTime = 0; 
bool visual_override = false; 

IWDG_HandleTypeDef hiwdg;
static volatile uint32_t g_run_cnt=0;

// Prototypes
void key1_it_callback(void);
void key2_it_callback(void);
void USB_Serial_Analys(void);
bool Check_Connet_MDM(void);
void REIN_TIM_SIGNAL_COUNT_Init(void);
void REIN_TIM_SIGNAL_COUNT_DeInit(void);
void Pulse_Receive_Init(void);
void Blockage_Detect(void);
void Main_Logic(void);
float fastAtof(const char *s);
void Signal_Dir_Init(void);

// -------------------------------------------------------------------------
// INITIALIZATION
// -------------------------------------------------------------------------
void iwdg_init(void) {
    IWDG->KR = 0x5555; IWDG->PR = IWDG_PRESCALER_256; IWDG->RLR = 125*10-1; IWDG->KR = 0xCCCC;         
}

void buffer_init(){
    // 1. Init Sensors & LEDs First (Fast Boot)
    buffer_sensor_init();

    // 2. Immediate Status Update (V2.8 Visuals)
    int initial_runout_state = digitalRead(ENDSTOP_3);
    if (initial_runout_state == 0) {
        // Loaded: Blue Solid, Red Off
        digitalWrite(START_LED, HIGH); 
        digitalWrite(ERR_LED, LOW); 
        digitalWrite(DULIAO, LOW);     
    } else {
        // Empty: Blue Off (will blink in loop), Red Off
        digitalWrite(START_LED, LOW); 
        digitalWrite(ERR_LED, LOW);    
        digitalWrite(DULIAO, HIGH);    
    }

    NVIC_SetPriority(TIM6_DAC_IRQn,0);
    NVIC_SetPriority(EXTI4_15_IRQn,1);

    if(Check_Connet_MDM()){ connet_mdm_flag=true; Serial.println("MDM OK"); }
    else{ connet_mdm_flag=false; Serial.println("MDM NO"); }

    Pulse_Receive_Init();
    buffer_motor_init();
    Signal_Dir_Init();
  
    // Load Settings
    EEPROM.get(EEPROM_ADDR_TIMEOUT, timeout);
    if (timeout == 0xFFFFFFFF || timeout == 0) { timeout = DEFAULT_TIMEOUT; EEPROM.put(EEPROM_ADDR_TIMEOUT, timeout); }

    EEPROM.get(EEPROM_ADDR_SPEED, SPEED);
    if (SPEED < 0 || SPEED > 1000) SPEED=260;
    VACTRUAL_VALUE=(uint32_t)(SPEED*Move_Divide_NUM*200/60/0.715);

    EEPROM.get(EEPROM_ADDR_MAINT_DIV, maintenance_divider);
    if (maintenance_divider > 6) { maintenance_divider = 3; EEPROM.put(EEPROM_ADDR_MAINT_DIV, maintenance_divider); }
    Serial.print("Mode: "); Serial.println(maintenance_divider);

    timer.pause();
    timer.setPrescaleFactor(4800);
    timer.setOverflow(1000);
    timer.attachInterrupt(&timer_it_callback);
    timer.resume();

    iwdg_init();
}

// -------------------------------------------------------------------------
// COMBO CHECK (ON THE FLY + FIXED VISUALS)
// -------------------------------------------------------------------------
bool check_speed_combo() {
    static bool prev_combo_state = false;
    static uint32_t last_combo_time = 0;
    
    bool current_combo = (digitalRead(KEY1) == LOW && digitalRead(KEY2) == LOW);
    
    if (current_combo && !prev_combo_state) {
        if (millis() - last_combo_time > 200) {
            // Increment
            maintenance_divider++;
            if (maintenance_divider > 6) maintenance_divider = 0;
            EEPROM.put(EEPROM_ADDR_MAINT_DIV, maintenance_divider);
            
            // Safety: Reset Run Latch when changing modes
            run_latch = false;

            // Start Visual Feedback
            digitalWrite(ERR_LED, LOW); // Start clean
            visual_override = true; 
            
            if (maintenance_divider == 0) {
                // Mode 0: One LONG blink (1s)
                digitalWrite(ERR_LED, HIGH); delay(1000);
                digitalWrite(ERR_LED, LOW);
                IWDG->KR = 0xAAAA;
            } 
            else if (maintenance_divider == 6) {
                // Mode 6: Six FAST blinks (Strobe)
                 for(int i=0; i<6; i++) {
                     digitalWrite(ERR_LED, HIGH); delay(80);
                     digitalWrite(ERR_LED, LOW); delay(80);
                     IWDG->KR = 0xAAAA;
                 }
            } 
            else {
                // Mode 1-5: Standard Count
                for(int i=0; i < maintenance_divider; i++) {
                    digitalWrite(ERR_LED, HIGH); delay(200);
                    digitalWrite(ERR_LED, LOW); delay(200);
                    IWDG->KR = 0xAAAA;
                }
            }
            
            last_combo_time = millis();
        }
    } else if (!current_combo) {
        // Clear override when released
        if (millis() - last_combo_time > 500) visual_override = false;
    }
    prev_combo_state = current_combo;
    return current_combo;
}

// -------------------------------------------------------------------------
// MAIN LOOP
// -------------------------------------------------------------------------
void buffer_loop()
{
    while (1)
    {
        uint32_t nowTime=millis();
        read_sensor_state();
        bool has_filament = (buffer.buffer1_material_swtich_state == 0);

        if (!visual_override) {
            // Blockage (Priority 1) -> Red Fast Blink
            if (blockage_detect.blockage_flag) {
                if (nowTime - lastToggleTime >= 50) {
                    lastToggleTime = nowTime; digitalToggle(ERR_LED);
                }
                digitalWrite(START_LED, LOW);
            }
            // Runout / Empty (Priority 2) -> BLUE Slow Blink (V2.8)
            else if (!has_filament) {
                if (nowTime - lastToggleTime >= 500) {
                    lastToggleTime = nowTime; digitalToggle(START_LED);
                }
                digitalWrite(ERR_LED, LOW); // Keep Red Off
            }
            // Normal / Loaded (Priority 3) -> Blue Solid (V2.8)
            else {
                digitalWrite(ERR_LED, LOW);
                digitalWrite(START_LED, HIGH); 
            }
        }
        
        check_speed_combo();

        if(connet_mdm_flag) Blockage_Detect();
        motor_control();
        USB_Serial_Analys();

        if (blockage_detect.blockage_flag) {
            if (millis() - blockage_inform_times > 3000) {
                digitalWrite(DULIAO, HIGH);
                blockage_detect.blockage_flag = false;
            }
        }
        g_run_cnt++;
    }
}

// -------------------------------------------------------------------------
// HARDWARE SETUP
// -------------------------------------------------------------------------
void buffer_sensor_init(){
  pinMode(HALL1,INPUT); pinMode(HALL2,INPUT); pinMode(HALL3,INPUT); pinMode(ENDSTOP_3,INPUT);
  pinMode(KEY1,INPUT); pinMode(KEY2,INPUT);
  attachInterrupt(KEY1,&key1_it_callback,CHANGE); attachInterrupt(KEY2,&key2_it_callback,CHANGE);

  pinMode(DUANLIAO,OUTPUT); pinMode(ERR_LED,OUTPUT); pinMode(START_LED,OUTPUT); pinMode(DULIAO,OUTPUT);

  // Y-Sensor (5V Tolerant Pin PB14)
  pinMode(CHAIN_Y_SENSOR_PIN, INPUT); 
  
  pinMode(EXTENSION_PIN2, OUTPUT); digitalWrite(EXTENSION_PIN2, HIGH);

  digitalWrite(DULIAO, HIGH); digitalWrite(DUANLIAO, HIGH);

  pinMode(FRONT_SIGNAL_PIN,INPUT_PULLUP); pinMode(BACK_SIGNAL_PIN,INPUT_PULLUP);
}

void buffer_motor_init(){
  pinMode(EN_PIN, OUTPUT); pinMode(STEP_PIN, OUTPUT); pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);      
  driver.begin(); driver.beginSerial(9600); driver.I_scale_analog(false); driver.toff(5);                 
  driver.rms_current(I_CURRENT); driver.microsteps(Move_Divide_NUM); driver.VACTUAL(STOP);           
  driver.en_spreadCycle(true); driver.pwm_autoscale(true);
}

void read_sensor_state(void){
    buffer.buffer1_pos1_sensor_state= digitalRead(HALL3);
    buffer.buffer1_pos2_sensor_state= digitalRead(HALL2);    
    buffer.buffer1_pos3_sensor_state= digitalRead(HALL1);        
    buffer.buffer1_material_swtich_state=digitalRead(ENDSTOP_3);    
    buffer.key1=digitalRead(KEY1);
    buffer.key2=digitalRead(KEY2);
}

// -------------------------------------------------------------------------
// MOTOR CONTROL LOGIC (CORE V2.6)
// -------------------------------------------------------------------------
void motor_control(void)
{
  static bool half_speed_active = false;
  static bool filling_latch = false; 
  static uint32_t pulse_end_time = 0;
  static Motor_State pulse_direction = Stop;
  static bool auto_unload_active = false;
  static uint32_t unload_timer = 0;
  static uint32_t feed_timer_start = 0;
  static uint32_t key1_start = 0;
  static uint32_t key2_start = 0;
  static bool key1_handled_3s = false; 
  static bool key1_handled_5s = false; 
  static bool key2_handled = false;
  static bool suppress_manual_buttons = false;

  bool key1_pressed = (buffer.key1 == 0);
  bool key2_pressed = (buffer.key2 == 0);
  bool has_filament = (buffer.buffer1_material_swtich_state == 0);

  // 1. Combo Check
  bool combo_active = check_speed_combo(); 
  if (combo_active) {
      suppress_manual_buttons = true;
      key1_start = 0; key2_start = 0; pulse_end_time = 0; auto_unload_active = false; 
  }
  if (!key1_pressed && !key2_pressed) suppress_manual_buttons = false;

  // 2. Input Processing
  if (!suppress_manual_buttons) {
      // Key 1 (Reverse/Pause/Unload)
      if (key1_pressed) {
          if (key1_start == 0) key1_start = millis(); 
          if (!key1_handled_3s && (millis() - key1_start > 3000)) {
              auto_mode = !auto_mode; digitalToggle(ERR_LED); key1_handled_3s = true;
          }
          if (!key1_handled_5s && (millis() - key1_start > 5000)) {
              auto_unload_active = true; unload_timer = millis();
              auto_mode = true; 
              // Visual confirm for unload start:
              digitalToggle(ERR_LED); digitalWrite(START_LED, HIGH); 
              key1_handled_5s = true;
          }
      } else {
          if (key1_start != 0 && !key1_handled_3s && !key1_handled_5s) {
              if (millis() - key1_start < 2000) { 
                  pulse_direction = Back; pulse_end_time = millis() + 2000;
                  auto_unload_active = false; 
              }
          }
          key1_start = 0; key1_handled_3s = false; key1_handled_5s = false;
      }

      // Key 2 (Feed)
      if (key2_pressed) {
          if (key2_start == 0) key2_start = millis();
          if (auto_unload_active) { auto_unload_active = false; motor_state = Stop; }
      } else {
          if (key2_start != 0) {
              if (millis() - key2_start < 2000) { 
                  pulse_direction = Forward; pulse_end_time = millis() + 2000;
              }
          }
          key2_start = 0;
      }
  }

  // 3. Daisy Chain Check
  bool chain_allowed_to_run = true;
  if (maintenance_divider == 6) { 
      if (run_latch) {
          chain_allowed_to_run = true;
      } else {
          bool prev_unit_done = digitalRead(FRONT_SIGNAL_PIN); 
          bool y_path_blocked = (digitalRead(CHAIN_Y_SENSOR_PIN) == HIGH); 
          if (prev_unit_done && !y_path_blocked) {
              run_latch = true; 
              chain_allowed_to_run = true;
          } else {
              chain_allowed_to_run = false;
          }
      }
  }

  // 4. State Decision
  if (auto_unload_active) {
      if (!has_filament) { motor_state = Stop; auto_unload_active = false; }
      else if (millis() - unload_timer > 120000) { 
          motor_state = Stop; auto_unload_active = false; is_error = true; 
      }
      else {
          motor_state = Back; driver.VACTUAL(VACTRUAL_VALUE); 
          last_motor_state = motor_state; digitalWrite(DULIAO, LOW); return; 
      }
  }

  if (suppress_manual_buttons) goto SENSOR_LOGIC;
  
  else if (key1_pressed || key2_pressed) {
      motor_state = Stop; half_speed_active = false; filling_latch = false; 
      run_latch = false; pulse_end_time = 0; feed_timer_start = 0; 
      digitalWrite(DULIAO, LOW); goto DRIVE_MOTOR; 
  }
  
  else if (millis() < pulse_end_time) {
      if (pulse_direction == Forward && (buffer.buffer1_pos2_sensor_state || buffer.buffer1_pos3_sensor_state)) {
          motor_state = Stop; pulse_end_time = 0; 
      } else {
          motor_state = pulse_direction;
          if (pulse_direction == Forward) run_latch = true;
      }
      half_speed_active = false; digitalWrite(DULIAO, LOW); 
      goto DRIVE_MOTOR;
  }
  
  else if (!auto_mode) {
      motor_state = Stop; half_speed_active = false; filling_latch = false; 
      run_latch = false; feed_timer_start = 0; digitalWrite(DULIAO, LOW); 
  }
  
  else if (!chain_allowed_to_run) {
      motor_state = Stop; half_speed_active = false; filling_latch = false; 
      feed_timer_start = 0; digitalWrite(DULIAO, LOW); 
  }
  
  else if (!has_filament && buffer.buffer1_pos1_sensor_state) {
      motor_state = Stop; half_speed_active = false; filling_latch = false; 
      run_latch = false; feed_timer_start = 0; digitalWrite(DULIAO, HIGH); 
  }
  
  else {
      SENSOR_LOGIC:
      digitalWrite(DULIAO, LOW);

      if (!has_filament && buffer.buffer1_pos1_sensor_state) {
          motor_state = Stop; half_speed_active = false; filling_latch = false;
          digitalWrite(DULIAO, HIGH); goto DRIVE_MOTOR;
      }
      
      if (buffer.buffer1_pos3_sensor_state) { 
        last_motor_state = motor_state; motor_state = Back;
        half_speed_active = false; filling_latch = false; feed_timer_start = 0;
      }
      else if (buffer.buffer1_pos1_sensor_state) { 
        last_motor_state = motor_state; motor_state = Forward;
        filling_latch = true;
        if (half_speed_active) driver.VACTUAL(-VACTRUAL_VALUE); 
        half_speed_active = false; 
      }
      else if (buffer.buffer1_pos2_sensor_state) { 
        last_motor_state = motor_state; motor_state = Stop;
        half_speed_active = false; filling_latch = false; feed_timer_start = 0;
      }
      else { 
        if (filling_latch) {
            motor_state = Forward;
            if (half_speed_active) { driver.VACTUAL(-VACTRUAL_VALUE); half_speed_active = false; }
        } 
        else {
            uint8_t effective_div = (maintenance_divider == 6) ? 3 : maintenance_divider;
            if (effective_div > 0) {
                if (!half_speed_active) {
                  driver.VACTUAL(-VACTRUAL_VALUE / effective_div); 
                  motor_state = Forward; last_motor_state = motor_state; half_speed_active = true;
                }
            } else {
                last_motor_state = motor_state; motor_state = Stop; 
                half_speed_active = false; feed_timer_start = 0;
            }
        }
      }
      
      if (motor_state == Forward) {
          if (feed_timer_start == 0) feed_timer_start = millis();
          else if (millis() - feed_timer_start > 120000) {
              is_error = true; motor_state = Stop; auto_mode = false; feed_timer_start = 0;
          }
      } else {
          feed_timer_start = 0;
      }
  }

  DRIVE_MOTOR:
  if(motor_state == last_motor_state) return; 

  switch(motor_state) {
    case Forward:
      if(last_motor_state == Back) driver.VACTUAL(STOP);
      driver.VACTUAL(-VACTRUAL_VALUE); break;
    case Back:
      if(last_motor_state == Forward) driver.VACTUAL(STOP);
      driver.VACTUAL(VACTRUAL_VALUE); break;
    case Stop:
      driver.VACTUAL(0); break;
  }
  last_motor_state = motor_state; 
}

// -------------------------------------------------------------------------
// HELPERS (LINKER FIX)
// -------------------------------------------------------------------------
void timer_it_callback(){
    static uint32_t i=0; i++;
    if(i>=50) { if(g_run_cnt == 0) { Serial.println("CPU Reset"); while(1){delay(1);} } g_run_cnt=0; i=0; }
    IWDG->KR = 0xAAAA;   
    if (is_front) { front_time+=100; if (front_time > timeout) is_error = true; }
}

void key1_it_callback(void){
    if(!digitalRead(KEY1)){
        key1_press_times=millis(); key1_press_cnt++; key1_press_flag=true;
    } else {
        if(millis()-key1_press_times<=500){
            key1_release_flag=true; key1_release_times=millis();
        } else {
            key1_release_flag=false; key1_press_cnt=0;
        }
        key1_press_flag=false;
    }
}

void key2_it_callback(void){
    if(!digitalRead(KEY2)){
        key2_press_times=millis(); key2_press_cnt++; key2_press_flag=true;
    } else {
        if(millis()-key2_press_times<=500){
            key2_release_flag=true; key2_release_times=millis();
        } else {
            key2_release_flag=false; key2_press_cnt=0;
        }
        key2_press_flag=false;
    }    
}

void Recv_MDM_Pulse_IT_Callback(void){ blockage_detect.mdm_pulse_cnt++; }
void Dir_IT_Callback(void){ if(SIGNAL_COUNT_READ_DIR_IO()) SIGNAL_COUNT_UP(); else SIGNAL_COUNT_DOWN(); }

void buffer_debug(void){
    static int i=0; if(i<0x1ff){ driver.GCONF(i); driver.PWMCONF(i); i++; }
    if(driver.CRCerror) Serial.println("CRCerror");
    else { Serial.print("GCONF:0x"); Serial.println(driver.GCONF(),HEX); }
    delay(1000);
}

void USB_Serial_Analys(void){
    static String serial_buf;
    if(Serial.available()){
        char c=Serial.read();
        if(c=='\n'){
            if(strstr(serial_buf.c_str(),"rt")) { Serial.print("TO="); Serial.println(timeout); }
            else if(strstr(serial_buf.c_str(),"timeout")){
                int index=serial_buf.indexOf(" ");
                if(index!=-1) { timeout=serial_buf.substring(index+1).toInt(); EEPROM.put(EEPROM_ADDR_TIMEOUT, timeout); }
            }
            else if(strstr(serial_buf.c_str(),"steps")){
                int index=serial_buf.indexOf(" ");
                if(index!=-1) {
                    serial_buf=serial_buf.substring(index+1);
                    steps=serial_buf.toInt();
                    EEPROM.put(EEPROM_ADDR_STEPS, steps);
                    Serial.print("set steps succeed! steps="); Serial.println(steps);
                    if(connet_mdm_flag) { REIN_TIM_SIGNAL_COUNT_DeInit(); REIN_TIM_SIGNAL_COUNT_Init(); }
                }
            }        
            else if(strstr(serial_buf.c_str(),"clear")){
                blockage_detect.actual_distance=0; blockage_detect.target_distance=0;
                blockage_detect.mdm_pulse_cnt=0; blockage_detect.pulse_cnt=0;                
            }
            else if(strstr(serial_buf.c_str(),"encoder")){
                int index=serial_buf.indexOf(" ");
                if(index!=-1) {
                    serial_buf=serial_buf.substring(index+1);
                    encoder_length=fastAtof(serial_buf.c_str());
                    EEPROM.put(EEPROM_ADDR_ENCODER_LENGTH, encoder_length);
                    blockage_detect.allow_error = encoder_length*allow_error_scale;
                    Serial.print("set encoder length succeed! encoder_length="); Serial.println(encoder_length);
                }
            }
            else if(strstr(serial_buf.c_str(),"info")){
                Serial.println("encoder_length="+String(encoder_length));
                Serial.println("timeout="+String(timeout));
                Serial.println("steps="+String(steps));
                Serial.println("allow_error_scale="+String(allow_error_scale));
                Serial.println("allow_error="+String(blockage_detect.allow_error));
                Serial.println("maintenance_divider="+String(maintenance_divider));
            }            
            else if(strstr(serial_buf.c_str(),"scale")){
                int index=serial_buf.indexOf(" ");
                if(index!=-1) {
                    serial_buf=serial_buf.substring(index+1);
                    allow_error_scale=fastAtof(serial_buf.c_str());
                    EEPROM.put(EEPROM_ADDR_ERROR_SCALE, allow_error_scale);
                    blockage_detect.allow_error = encoder_length*allow_error_scale;
                    Serial.print("set scale length succeed! allow_error_scale="); Serial.println(allow_error_scale);
                }
            }
            else if(strstr(serial_buf.c_str(),"speed")){
                int index=serial_buf.indexOf(" ");
                if(index!=-1) {
                    serial_buf=serial_buf.substring(index+1);
                    SPEED=fastAtof(serial_buf.c_str());
                    VACTRUAL_VALUE=(uint32_t)(SPEED*Move_Divide_NUM*200/60/0.715);
                    EEPROM.put(EEPROM_ADDR_SPEED, SPEED);
                    Serial.print("set speed succeed! speed="); Serial.println(SPEED);
                }
            }            
            serial_buf="";
        } else serial_buf+=c;
    }
}

bool Check_Connet_MDM(void){
    delay(1000);
    pinMode(MDM_DPIN,INPUT);
    bool mdm_state=digitalRead(MDM_DPIN);
    if(mdm_state){ pinMode(MDM_DPIN,INPUT_PULLDOWN); return digitalRead(MDM_DPIN); }
    else{ pinMode(MDM_DPIN,INPUT_PULLUP); return !digitalRead(MDM_DPIN); }
}

void REIN_TIM_SIGNAL_COUNT_Init(void){
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    SIGNAL_COUNT_PUL_CLK_ENABLE(); 
    GPIO_InitStruct.Pin = SIGNAL_COUNT_PUL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; 
    GPIO_InitStruct.Pull = GPIO_NOPULL;        
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
    HAL_GPIO_Init(SIGNAL_COUNT_PUL_GPIO_Port, &GPIO_InitStruct);
    SIGNAL_COUNT_TIM_CLK_ENABLE(); 
    SIGNAL_COUNT_Get_HTIM.Instance = SIGNAL_COUNT_Get_TIM;
    SIGNAL_COUNT_Get_HTIM.Init.Prescaler = 0; SIGNAL_COUNT_Get_HTIM.Init.CounterMode = TIM_COUNTERMODE_UP;                   
    SIGNAL_COUNT_Get_HTIM.Init.Period = 65536-1; SIGNAL_COUNT_Get_HTIM.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;               
    HAL_TIM_Base_Init(&SIGNAL_COUNT_Get_HTIM);
    HAL_TIM_Base_Start(&SIGNAL_COUNT_Get_HTIM);
}

void REIN_TIM_SIGNAL_COUNT_DeInit(void){
    HAL_TIM_Base_Stop(&SIGNAL_COUNT_Get_HTIM);                                        
    HAL_GPIO_DeInit(SIGNAL_COUNT_PUL_GPIO_Port, SIGNAL_COUNT_PUL_Pin);    
}

void Signal_Dir_Init(void){
    pinMode(DIR_PIN,INPUT); attachInterrupt(DIR_PIN,&Dir_IT_Callback,CHANGE);
}

void Pulse_Receive_Init(void){
    EEPROM.get(EEPROM_ADDR_ERROR_SCALE, allow_error_scale);
    if(isnan(allow_error_scale)) allow_error_scale = DEFAULT_ALLOW_ERROR_SCALE;
    if(connet_mdm_flag){ pinMode(PULSE2_PIN,INPUT); attachInterrupt(PULSE2_PIN,&Recv_MDM_Pulse_IT_Callback,RISING); }
    EEPROM.get(EEPROM_ADDR_ENCODER_LENGTH, encoder_length);
    if(isnan(encoder_length)) encoder_length = DEFAULT_ENCODER_LENGTH;
    EEPROM.get(EEPROM_ADDR_STEPS, steps);
    if(steps > 51200) steps = DEFAULT_STEPS;
    if(connet_mdm_flag) REIN_TIM_SIGNAL_COUNT_Init();
    blockage_detect.allow_error = encoder_length*allow_error_scale;
}

void Blockage_Detect(void){
    static uint32_t last_target_distance=blockage_detect.target_distance;
    static uint32_t last_timer_cnt=TIM2->CNT;
    static uint32_t last_time=0;

    if(last_timer_cnt==TIM2->CNT){
        if(millis()-last_time>=500){
            blockage_detect.actual_distance=0; blockage_detect.target_distance=0;
            blockage_detect.mdm_pulse_cnt=0; blockage_detect.extrusion_pulse_cnt=0;
        }
    } else {
        last_timer_cnt=TIM2->CNT; last_time=millis();
        blockage_detect.last_pulse_cnt=blockage_detect.pulse_cnt;
        blockage_detect.pulse_cnt=TIM2->CNT;
        blockage_detect.pulse_cnt_sub=blockage_detect.pulse_cnt-blockage_detect.last_pulse_cnt;
        blockage_detect.extrusion_pulse_cnt+=blockage_detect.pulse_cnt_sub;
        if(blockage_detect.extrusion_pulse_cnt<0) blockage_detect.target_distance=0;
        else blockage_detect.target_distance=(blockage_detect.extrusion_pulse_cnt)/steps;
    }
    blockage_detect.actual_distance=blockage_detect.mdm_pulse_cnt*encoder_length;
    blockage_detect.distance_error=blockage_detect.actual_distance-blockage_detect.target_distance;
    
    static bool detect_blockage=false;
    static uint32_t detect_blockage_time=0;
    
    if(!detect_blockage){
        if(blockage_detect.target_distance!=last_target_distance){
            if(abs(blockage_detect.distance_error)>blockage_detect.allow_error&&blockage_detect.target_distance>=blockage_detect.allow_error){
                detect_blockage=true;
                detect_blockage_time=millis();
                blockage_detect.actual_distance=0;
                blockage_detect.target_distance=0;
                blockage_detect.mdm_pulse_cnt=0;
                blockage_detect.extrusion_pulse_cnt=0;
            }
            last_target_distance=blockage_detect.target_distance;
        }
    }
    else if(blockage_detect.target_distance>blockage_detect.allow_error){
        if(millis()-detect_blockage_time>=100){
            if(abs(blockage_detect.distance_error)>blockage_detect.allow_error){
                blockage_detect.blockage_flag=true;
                blockage_inform_times=millis();
                digitalWrite(DULIAO,LOW);            
                detect_blockage=false;
                detect_blockage_time=0;    
                blockage_detect.actual_distance=0;
                blockage_detect.target_distance=0;
                blockage_detect.mdm_pulse_cnt=0;
                blockage_detect.extrusion_pulse_cnt=0;                
            }
            else{
                detect_blockage=false;
                detect_blockage_time=0;
            }
        }
    }
}

float fastAtof(const char *s) {
    float val = 0.0f; int sign = 1;
    if (*s == '-') { sign = -1; s++; } else if (*s == '+') { s++; }
    while (*s >= '0' && *s <= '9') { val = val * 10.0f + (*s - '0'); s++; }
    if (*s == '.') { s++; float frac = 1.0f; while (*s >= '0' && *s <= '9') { frac *= 0.1f; val += (*s - '0') * frac; s++; } }
    return sign * val;
}

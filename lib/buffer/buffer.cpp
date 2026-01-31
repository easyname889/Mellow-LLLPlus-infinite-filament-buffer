#include "buffer.h"
#include "stm32f0xx_hal_iwdg.h"

// -------------------------------------------------------------------------
// GLOBALS
// -------------------------------------------------------------------------
TMC2209Stepper driver(UART, R_SENSE, DRIVER_ADDRESS);
Buffer buffer = {0};
Motor_State motor_state = Stop;
static Motor_State last_motor_state = Stop;

bool is_front = false;
uint32_t front_time = 0;
uint32_t timeout = 60000;
bool is_error = false;
String serial_buf;

static HardwareTimer timer(TIM6);
TIM_HandleTypeDef htim2;

// Button State Globals
bool key1_press_flag = false;
bool key2_press_flag = false;
bool key1_release_flag = false;
bool key2_release_flag = false;
uint32_t key1_press_times = 0;
uint32_t key2_press_times = 0;
uint32_t key1_release_times = 0;
uint32_t key2_release_times = 0;
uint8_t key1_press_cnt = 0;
uint8_t key2_press_cnt = 0;

uint32_t inform_flag = false;
uint32_t inform_times = 0;

// Motor/Encoder Settings
uint32_t steps = 916;
BlockageDetect blockage_detect = {0};
bool connet_mdm_flag = false;
uint32_t blockage_inform_times = 0;

float encoder_length = 1.73;
float allow_error_scale = 2;

// EEPROM Addresses
const int EEPROM_ADDR_TIMEOUT = 0;
const int EEPROM_ADDR_STEPS = 4;
const int EEPROM_ADDR_ENCODER_LENGTH = 8;
const int EEPROM_ADDR_ERROR_SCALE = 12;
const int EEPROM_ADDR_SPEED = 16;
const int EEPROM_ADDR_MAINT_DIV = 20;
const int EEPROM_ADDR_LAST_STATE = 24;

// Logic Globals
uint8_t maintenance_divider = 3;
bool auto_mode = true;
bool run_latch = false;
bool visual_override = false;

// Speed Globals
float SPEED = 260;
uint32_t VACTRUAL_VALUE = 0;
uint32_t Move_Divide_NUM = 16;
uint32_t I_CURRENT = 800;


IWDG_HandleTypeDef hiwdg;
static volatile uint32_t g_run_cnt = 0;
uint32_t lastToggleTime = 0;

// -------------------------------------------------------------------------
// INITIALIZATION
// -------------------------------------------------------------------------
void iwdg_init(void) {
    IWDG->KR = 0x5555; IWDG->PR = IWDG_PRESCALER_256; IWDG->RLR = 125*10-1; IWDG->KR = 0xCCCC;
}

void buffer_init(){
    buffer_sensor_init();

    // Fast Boot Status
    int initial_runout_state = digitalRead(ENDSTOP_3);
    if (initial_runout_state == 0) {
        digitalWrite(START_LED, HIGH);
        digitalWrite(ERR_LED, LOW);
        digitalWrite(DULIAO, LOW);
    } else {
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

    EEPROM.get(EEPROM_ADDR_TIMEOUT, timeout);
    if (timeout == 0xFFFFFFFF || timeout == 0) { timeout = 60000; EEPROM.put(EEPROM_ADDR_TIMEOUT, timeout); }

    EEPROM.get(EEPROM_ADDR_SPEED, SPEED);
    if (SPEED < 0 || SPEED > 1000) SPEED=260;
    VACTRUAL_VALUE=(uint32_t)(SPEED*Move_Divide_NUM*200/60/0.715);

    EEPROM.get(EEPROM_ADDR_MAINT_DIV, maintenance_divider);
    if (maintenance_divider > 6) { maintenance_divider = 3; EEPROM.put(EEPROM_ADDR_MAINT_DIV, maintenance_divider); }
    Serial.print("Mode: "); Serial.println(maintenance_divider);

    if (maintenance_divider == 6) {
        bool saved_state = false;
        EEPROM.get(EEPROM_ADDR_LAST_STATE, saved_state);
        if (saved_state != 0 && saved_state != 1) saved_state = false;
        run_latch = saved_state;
    } else {
        run_latch = false;
    }

    timer.pause();
    timer.setPrescaleFactor(4800);
    timer.setOverflow(1000);
    timer.attachInterrupt(&timer_it_callback);
    timer.resume();

    iwdg_init();
}

void update_run_latch(bool new_state) {
    if (run_latch != new_state) {
        run_latch = new_state;
        if (maintenance_divider == 6) {
            EEPROM.put(EEPROM_ADDR_LAST_STATE, run_latch);
        }
    }
}

// -------------------------------------------------------------------------
// COMBO CHECK
// -------------------------------------------------------------------------
void show_config_blink(uint8_t mode) {
    digitalWrite(ERR_LED, LOW);
    delay(200);
    IWDG->KR = 0xAAAA; g_run_cnt++;

    if (mode == 0) {
        digitalWrite(ERR_LED, HIGH); delay(1000);
        digitalWrite(ERR_LED, LOW);
        IWDG->KR = 0xAAAA; g_run_cnt++;
    }
    else if (mode == 6) {
         for(int i=0; i<6; i++) {
             digitalWrite(ERR_LED, HIGH); delay(80);
             digitalWrite(ERR_LED, LOW); delay(80);
             IWDG->KR = 0xAAAA; g_run_cnt++;
         }
    }
    else {
        for(int i=0; i < mode; i++) {
            digitalWrite(ERR_LED, HIGH); delay(200);
            digitalWrite(ERR_LED, LOW); delay(200);
            IWDG->KR = 0xAAAA; g_run_cnt++;
        }
    }
}

bool check_speed_combo() {
    if (digitalRead(KEY1) == LOW && digitalRead(KEY2) == LOW) {
        delay(50);
        if (digitalRead(KEY1) == LOW && digitalRead(KEY2) == LOW) {

            visual_override = true;
            digitalWrite(START_LED, LOW);

            while (digitalRead(KEY1) == LOW && digitalRead(KEY2) == LOW) {
                maintenance_divider++;
                if (maintenance_divider > 6) maintenance_divider = 0;
                show_config_blink(maintenance_divider);

                for(int w=0; w<5; w++) {
                    delay(100);
                    IWDG->KR = 0xAAAA; g_run_cnt++;
                    if (digitalRead(KEY1) == HIGH || digitalRead(KEY2) == HIGH) break;
                }
            }
            EEPROM.put(EEPROM_ADDR_MAINT_DIV, maintenance_divider);
            update_run_latch(false);
            visual_override = false;
            digitalWrite(ERR_LED, LOW);
            return true;
        }
    }
    return false;
}

// -------------------------------------------------------------------------
// MAIN LOOP
// -------------------------------------------------------------------------
void buffer_loop()
{
    int8_t last_led_mode = -1; // -1:None, 0:Blockage, 1:Runout, 2:SlaveWait, 3:Active

    while (1)
    {
        uint32_t nowTime=millis();
        read_sensor_state();
        bool has_filament = (buffer.buffer1_material_swtich_state == 0);

        if (!visual_override) {
            // 1. Error/Blockage
            if (blockage_detect.blockage_flag) {
                if (last_led_mode != 0) {
                    digitalWrite(START_LED, LOW);
                    last_led_mode = 0;
                }
                if (nowTime - lastToggleTime >= 50) {
                    lastToggleTime = nowTime; digitalToggle(ERR_LED);
                }
            }
            // 2. Runout (Empty) -> Fast Blue Blink
            else if (!has_filament) {
                if (last_led_mode != 1) {
                    digitalWrite(ERR_LED, LOW);
                    last_led_mode = 1;
                }
                if (nowTime - lastToggleTime >= 200) { // Fast Blink for Empty
                    lastToggleTime = nowTime; digitalToggle(START_LED);
                }
            }
            // 3. Slave Waiting (Mode 6 + Loaded + Not Running) -> Slow Blink (Waiting)
            else if (maintenance_divider == 6 && !run_latch && has_filament) {
                if (last_led_mode != 2) {
                     digitalWrite(ERR_LED, LOW);
                     last_led_mode = 2;
                }
                if (nowTime - lastToggleTime >= 1000) { // 1 sec Blink (Breathing)
                     lastToggleTime = nowTime; digitalToggle(START_LED);
                }
            }
            // 4. Active / Running -> Solid Blue
            else {
                if (last_led_mode != 3) {
                    digitalWrite(ERR_LED, LOW);
                    digitalWrite(START_LED, HIGH);
                    last_led_mode = 3;
                }
            }
        }

        if (check_speed_combo()) {
            last_led_mode = -1;
        }

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

  pinMode(CHAIN_Y_SENSOR_PIN, INPUT);
  pinMode(EXTENSION_PIN2, OUTPUT); digitalWrite(EXTENSION_PIN2, HIGH); // V1.0 legacy?

  digitalWrite(DULIAO, HIGH); digitalWrite(DUANLIAO, HIGH);

  pinMode(FRONT_SIGNAL_PIN,INPUT_PULLUP); pinMode(BACK_SIGNAL_PIN,INPUT_PULLUP);
}

void buffer_motor_init(){
  pinMode(EN_PIN, OUTPUT); pinMode(STEP_PIN, OUTPUT); pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  driver.begin(); driver.beginSerial(9600); driver.I_scale_analog(false); driver.toff(5);
  driver.rms_current(I_CURRENT); driver.microsteps(Move_Divide_NUM); driver.VACTUAL(Stop); // VACTUAL takes int32_t speed
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
// MOTOR CONTROL
// -------------------------------------------------------------------------
void motor_control(void)
{
  static bool half_speed_active = false;
  static bool filling_latch = false;
  static uint32_t pulse_start_time = 0;
  static bool pulsing_active = false;
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

  // Safety exit if holding combo
  if (key1_pressed && key2_pressed) return;

  // --- INPUT PROCESSING ---
  if (!suppress_manual_buttons) {
      // Key 1 (Reverse/Pause)
      if (key1_pressed) {
          if (key1_start == 0) key1_start = millis();
          if (!key1_handled_3s && (millis() - key1_start > 3000)) {
              auto_mode = !auto_mode;
              if(!auto_mode) update_run_latch(false); // Unlatch on pause
              digitalToggle(ERR_LED); key1_handled_3s = true;
          }
          if (!key1_handled_5s && (millis() - key1_start > 5000)) {
              auto_unload_active = true; unload_timer = millis();
              auto_mode = true;
              update_run_latch(false); // Safety unlatch
              digitalToggle(ERR_LED); digitalWrite(START_LED, HIGH);
              key1_handled_5s = true;
          }
      } else {
          if (key1_start != 0 && !key1_handled_3s && !key1_handled_5s) {
              if (millis() - key1_start < 2000) {
                  pulse_direction = Back; pulse_start_time = millis(); pulsing_active = true;
                  auto_unload_active = false; update_run_latch(false);
              }
          }
          key1_start = 0; key1_handled_3s = false; key1_handled_5s = false;
      }

      // Key 2 (Feed/Manual Start)
      if (key2_pressed) {
          if (key2_start == 0) key2_start = millis();
          if (!key2_handled && (millis() - key2_start > 1000)) {
              // 1s Hold: Manual Start for Chain
              auto_mode = true;
              update_run_latch(true); // START CHAIN
              digitalToggle(ERR_LED);
              key2_handled = true;
          }
          if (auto_unload_active) { auto_unload_active = false; motor_state = Stop; }
      } else {
          if (key2_start != 0 && !key2_handled) {
              if (millis() - key2_start < 2000) {
                  pulse_direction = Forward; pulse_start_time = millis(); pulsing_active = true;
              }
          }
          key2_start = 0; key2_handled = false;
      }
  }

  // Check Pulse Timeout (Overflow Protected)
  if (pulsing_active && (millis() - pulse_start_time >= 2000)) {
      pulsing_active = false;
  }

  // --- DAISY CHAIN SIGNAL CHECK ---
  bool chain_allowed_to_run = true;
  if (maintenance_divider == 6) {
      if (run_latch) {
          chain_allowed_to_run = true;
      } else {
          bool prev_unit_done = digitalRead(FRONT_SIGNAL_PIN);
          // V3.0: High (4.18V) = Clear. Low = Blocked.
          bool y_path_clear = (digitalRead(CHAIN_Y_SENSOR_PIN) == HIGH);

          if (prev_unit_done && y_path_clear) {
              update_run_latch(true); // Auto Start
              chain_allowed_to_run = true;
          } else {
              chain_allowed_to_run = false;
          }
      }
  }

  // --- STATE DECISIONS ---

  // P0: Auto Unload
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

  // P1: Manual Hold (Stop Safety) - Skipped if handled (latch start)
  else if ((key1_pressed && !key1_handled_3s && !key1_handled_5s) || (key2_pressed && !key2_handled)) {
      motor_state = Stop; half_speed_active = false; filling_latch = false;
      // Do NOT update latch here, just stop motor
      pulsing_active = false; feed_timer_start = 0;
      digitalWrite(DULIAO, LOW); goto DRIVE_MOTOR;
  }

  // P2: Pulse (2s)
  else if (pulsing_active) {
      if (pulse_direction == Forward && (buffer.buffer1_pos2_sensor_state || buffer.buffer1_pos3_sensor_state)) {
          motor_state = Stop; pulsing_active = false;
      } else {
          motor_state = pulse_direction;
      }
      half_speed_active = false; digitalWrite(DULIAO, LOW);
      goto DRIVE_MOTOR;
  }

  // P3: Auto Paused
  else if (!auto_mode) {
      motor_state = Stop; half_speed_active = false; filling_latch = false;
      update_run_latch(false); feed_timer_start = 0; digitalWrite(DULIAO, LOW);
  }

  // P4: Chain Wait (Slave Mode)
  else if (!chain_allowed_to_run) {
      motor_state = Stop; half_speed_active = false; filling_latch = false;
      feed_timer_start = 0; digitalWrite(DULIAO, LOW);
  }

  // P5: Runout Logic (Tail Clearance Fix)
  else if (!has_filament) {
      // If Hall 3 (Empty) triggered -> Done
      if (buffer.buffer1_pos1_sensor_state) {
          motor_state = Stop; half_speed_active = false; filling_latch = false;
          update_run_latch(false); // Unlatch
          feed_timer_start = 0;
          digitalWrite(DULIAO, HIGH); // Signal Next Unit
      } else {
          // If Hall 3 NOT triggered -> Tail is in gear -> Push it!
          motor_state = Forward;
          uint8_t effective_div = (maintenance_divider == 6) ? 3 : maintenance_divider;
          if (effective_div == 0) effective_div = 3;
          driver.VACTUAL(-VACTRUAL_VALUE / effective_div);
          half_speed_active = false;
          digitalWrite(DULIAO, LOW); // Still Busy
          return; // Skip standard logic
      }
  }

  // P6: Normal Auto Operation
  else {
      SENSOR_LOGIC:
      digitalWrite(DULIAO, LOW);

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
      if(last_motor_state == Back) driver.VACTUAL(0); // STOP before reverse
      driver.VACTUAL(-VACTRUAL_VALUE); break;
    case Back:
      if(last_motor_state == Forward) driver.VACTUAL(0); // STOP before reverse
      driver.VACTUAL(VACTRUAL_VALUE); break;
    case Stop:
      driver.VACTUAL(0); break;
  }
  last_motor_state = motor_state;
}

// -------------------------------------------------------------------------
// HELPERS
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

#define SIGNAL_COUNT_READ_DIR_IO()    (SIGNAL_COUNT_DIR_GPIO_Port -> IDR & SIGNAL_COUNT_DIR_Pin)
#define SIGNAL_COUNT_READ_COUNT()     (SIGNAL_COUNT_Get_TIM -> CNT)
#define SIGNAL_COUNT_UP()             (SIGNAL_COUNT_Get_TIM -> CR1 &= ~(TIM_CR1_DIR))
#define SIGNAL_COUNT_DOWN()           (SIGNAL_COUNT_Get_TIM -> CR1 |=  (TIM_CR1_DIR))

void Dir_IT_Callback(void){ if(SIGNAL_COUNT_READ_DIR_IO()) SIGNAL_COUNT_UP(); else SIGNAL_COUNT_DOWN(); }

void buffer_debug(void){
    static int i=0; if(i<0x1ff){ driver.GCONF(i); driver.PWMCONF(i); i++; }
    if(driver.CRCerror) Serial.println("CRCerror");
    else { Serial.print("GCONF:0x"); Serial.println(driver.GCONF(),HEX); }
    delay(1000);
}

void USB_Serial_Analys(void){
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
    if(isnan(allow_error_scale)) allow_error_scale = 2; // Default
    if(connet_mdm_flag){ pinMode(PULSE2_PIN,INPUT); attachInterrupt(PULSE2_PIN,&Recv_MDM_Pulse_IT_Callback,RISING); }
    EEPROM.get(EEPROM_ADDR_ENCODER_LENGTH, encoder_length);
    if(isnan(encoder_length)) encoder_length = 1.73; // Default
    EEPROM.get(EEPROM_ADDR_STEPS, steps);
    if(steps > 51200) steps = 916; // Default
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

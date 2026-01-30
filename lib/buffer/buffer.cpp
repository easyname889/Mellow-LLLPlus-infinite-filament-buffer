#include "buffer.h"

// --- GLOBAL VARIABLES DEFINITIONS ---
TMC2209Stepper driver(&UART, R_SENSE, DRIVER_ADDRESS);
Buffer buffer = {0};
Motor_State motor_state = Stop;
static Motor_State last_motor_state = Stop;

bool is_front = false;
uint32_t front_time = 0;
uint32_t timeout = 60000;
bool is_error = false;

HardwareTimer timer(TIM6);
TIM_HandleTypeDef htim2;

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

uint32_t steps = 916;
bool connet_mdm_flag = false;

float encoder_length = 1.73;
float allow_error_scale = 2.0;
float SPEED = 260.0;
uint32_t VACTUAL_VALUE = 0;

// EEPROM Map
const int EEPROM_ADDR_TIMEOUT = 0;
const int EEPROM_ADDR_STEPS = 4;
const int EEPROM_ADDR_ENCODER_LENGTH = 8;
const int EEPROM_ADDR_ERROR_SCALE = 12;
const int EEPROM_ADDR_SPEED = 16;
const int EEPROM_ADDR_MAINT_DIV = 20;

uint8_t maintenance_divider = 3;
bool auto_mode = true;
bool run_latch = false;

uint32_t lastToggleTime = 0;
bool visual_override = false;

static volatile uint32_t g_run_cnt = 0;

// -------------------------------------------------------------------------
// INITIALIZATION
// -------------------------------------------------------------------------
void iwdg_init(void) {
    IWDG->KR = 0x5555; IWDG->PR = 6; IWDG->RLR = 125*10-1; IWDG->KR = 0xCCCC;
}

void buffer_init(){
    buffer_sensor_init();

    int initial_runout_state = digitalRead(ENDSTOP_3);
    if (initial_runout_state == 0) { // Loaded
        digitalWrite(START_LED, HIGH);
        digitalWrite(ERR_LED, LOW);
        digitalWrite(DULIAO, LOW);
    } else { // Empty
        digitalWrite(START_LED, LOW);
        digitalWrite(ERR_LED, LOW);
        digitalWrite(DULIAO, HIGH);
    }

    NVIC_SetPriority(TIM6_DAC_IRQn, 0);
    NVIC_SetPriority(EXTI4_15_IRQn, 1);

    if(Check_Connet_MDM()){ connet_mdm_flag = true; Serial.println("MDM OK"); }
    else { connet_mdm_flag = false; Serial.println("MDM NO"); }

    Pulse_Receive_Init();
    buffer_motor_init();
    Signal_Dir_Init();

    // Load Settings
    EEPROM.get(EEPROM_ADDR_TIMEOUT, timeout);
    if (timeout == 0xFFFFFFFF || timeout == 0) {
        timeout = 60000;
        EEPROM.put(EEPROM_ADDR_TIMEOUT, timeout);
    }

    EEPROM.get(EEPROM_ADDR_SPEED, SPEED);
    if (isnan(SPEED) || SPEED < 0 || SPEED > 1000) SPEED = 260;
    VACTUAL_VALUE = (uint32_t)(SPEED * Move_Divide_NUM * 200 / 60 / 0.715);

    EEPROM.get(EEPROM_ADDR_MAINT_DIV, maintenance_divider);
    if (maintenance_divider > 6) { maintenance_divider = 3; EEPROM.put(EEPROM_ADDR_MAINT_DIV, maintenance_divider); }
    Serial.print("Mode: "); Serial.println(maintenance_divider);

    timer.pause();
    timer.setPrescaleFactor(4800);
    timer.setOverflow(1000);
    timer.attachInterrupt(timer_it_callback);
    timer.resume();

    iwdg_init();
}

// -------------------------------------------------------------------------
// COMBO CHECK
// -------------------------------------------------------------------------
bool check_speed_combo() {
    static bool prev_combo_state = false;
    static uint32_t last_combo_time = 0;

    bool current_combo = (digitalRead(KEY1) == LOW && digitalRead(KEY2) == LOW);

    if (current_combo && !prev_combo_state) {
        if (millis() - last_combo_time > 200) {
            maintenance_divider++;
            if (maintenance_divider > 6) maintenance_divider = 0;
            EEPROM.put(EEPROM_ADDR_MAINT_DIV, maintenance_divider);
            Serial.print("Mode changed to: "); Serial.println(maintenance_divider);

            run_latch = false;
            digitalWrite(ERR_LED, LOW);
            visual_override = true;

            if (maintenance_divider == 0) {
                digitalWrite(ERR_LED, HIGH); delay(1000);
                digitalWrite(ERR_LED, LOW);
                IWDG->KR = 0xAAAA;
            }
            else if (maintenance_divider == 6) {
                 for(int i=0; i<6; i++) {
                     digitalWrite(ERR_LED, HIGH); delay(80);
                     digitalWrite(ERR_LED, LOW); delay(80);
                     IWDG->KR = 0xAAAA;
                 }
            }
            else {
                for(int i=0; i < maintenance_divider; i++) {
                    digitalWrite(ERR_LED, HIGH); delay(200);
                    digitalWrite(ERR_LED, LOW); delay(200);
                    IWDG->KR = 0xAAAA;
                }
            }
            last_combo_time = millis();
        }
    } else if (!current_combo) {
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
        uint32_t nowTime = millis();
        read_sensor_state();
        bool has_filament = (buffer.buffer1_material_swtich_state == 0);
        digitalWrite(DUANLIAO, has_filament ? HIGH : LOW);

        if (!visual_override) {
            if (is_error) {
                if (nowTime - lastToggleTime >= 50) {
                    lastToggleTime = nowTime;
                    digitalWrite(ERR_LED, !digitalRead(ERR_LED));
                }
                digitalWrite(START_LED, LOW);
            }
            else if (!has_filament) {
                if (nowTime - lastToggleTime >= 500) {
                    lastToggleTime = nowTime;
                    digitalWrite(START_LED, !digitalRead(START_LED));
                }
                digitalWrite(ERR_LED, LOW);
            }
            else {
                digitalWrite(ERR_LED, LOW);
                digitalWrite(START_LED, HIGH);
            }
        }

        check_speed_combo();

        motor_control();
        USB_Serial_Analys();

        g_run_cnt++;
    }
}

// -------------------------------------------------------------------------
// HARDWARE SETUP
// -------------------------------------------------------------------------
void buffer_sensor_init(){
  pinMode(HALL1, INPUT); pinMode(HALL2, INPUT); pinMode(HALL3, INPUT); pinMode(ENDSTOP_3, INPUT);
  pinMode(KEY1, INPUT); pinMode(KEY2, INPUT);
  attachInterrupt(KEY1, key1_it_callback, CHANGE); attachInterrupt(KEY2, key2_it_callback, CHANGE);

  pinMode(DUANLIAO, OUTPUT); pinMode(ERR_LED, OUTPUT); pinMode(START_LED, OUTPUT); pinMode(DULIAO, OUTPUT);
  pinMode(CHAIN_Y_SENSOR_PIN, INPUT);
  pinMode(EXTENSION_PIN2, OUTPUT); digitalWrite(EXTENSION_PIN2, HIGH);

  digitalWrite(DULIAO, HIGH); digitalWrite(DUANLIAO, HIGH);

  pinMode(FRONT_SIGNAL_PIN, INPUT_PULLUP); pinMode(BACK_SIGNAL_PIN, INPUT_PULLUP);
}

void buffer_motor_init(){
  pinMode(EN_PIN, OUTPUT); pinMode(STEP_PIN, OUTPUT); pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  UART.begin(9600);
  driver.begin();
  driver.pdn_disable(true);
  driver.I_scale_analog(false); driver.toff(5);
  driver.rms_current(I_CURRENT); driver.microsteps(Move_Divide_NUM); driver.VACTUAL(0);
  driver.en_spreadCycle(true); driver.pwm_autoscale(true);
}

void read_sensor_state(void){
    buffer.buffer1_pos1_sensor_state = digitalRead(HALL3);
    buffer.buffer1_pos2_sensor_state = digitalRead(HALL2);
    buffer.buffer1_pos3_sensor_state = digitalRead(HALL1);
    buffer.buffer1_material_swtich_state = digitalRead(ENDSTOP_3);
    buffer.key1 = digitalRead(KEY1);
    buffer.key2 = digitalRead(KEY2);
}

// -------------------------------------------------------------------------
// MOTOR CONTROL LOGIC
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
      if (key1_pressed) {
          if (key1_start == 0) key1_start = millis();
          if (!key1_handled_3s && (millis() - key1_start > 3000)) {
              auto_mode = !auto_mode; key1_handled_3s = true;
              is_error = false; // Reset error on pause/resume
          }
          if (!key1_handled_5s && (millis() - key1_start > 5000)) {
              auto_unload_active = true; unload_timer = millis();
              auto_mode = true;
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
  if (is_error) {
      motor_state = Stop; goto DRIVE_MOTOR;
  }

  if (auto_unload_active) {
      if (!has_filament) { motor_state = Stop; auto_unload_active = false; }
      else if (millis() - unload_timer > 120000) {
          motor_state = Stop; auto_unload_active = false; is_error = true;
      }
      else {
          motor_state = Back;
          digitalWrite(DULIAO, LOW); goto DRIVE_MOTOR;
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
        motor_state = Back;
        half_speed_active = false; filling_latch = false; feed_timer_start = 0;
      }
      else if (buffer.buffer1_pos1_sensor_state) {
        motor_state = Forward;
        filling_latch = true;
        half_speed_active = false;
      }
      else if (buffer.buffer1_pos2_sensor_state) {
        motor_state = Stop;
        half_speed_active = false; filling_latch = false; feed_timer_start = 0;
      }
      else {
        if (filling_latch) {
            motor_state = Forward;
            half_speed_active = false;
        }
        else {
            uint8_t effective_div = (maintenance_divider == 6) ? 3 : maintenance_divider;
            if (effective_div > 0) {
                motor_state = Forward;
                half_speed_active = true;
            } else {
                motor_state = Stop;
                half_speed_active = false; feed_timer_start = 0;
            }
        }
      }

      if (motor_state == Forward && !half_speed_active) {
          is_front = true;
          if (feed_timer_start == 0) feed_timer_start = millis();
          else if (millis() - feed_timer_start > timeout) {
              is_error = true; motor_state = Stop; auto_mode = false; feed_timer_start = 0;
          }
      } else {
          is_front = false;
          feed_timer_start = 0;
      }
  }

  DRIVE_MOTOR:
  static uint32_t last_vactual_val = 0;
  int32_t target_vactual = 0;
  if (motor_state == Forward) {
      uint8_t effective_div = (half_speed_active) ? ((maintenance_divider == 6) ? 3 : maintenance_divider) : 1;
      if (effective_div == 0) effective_div = 1;
      target_vactual = -(int32_t)(VACTUAL_VALUE / effective_div);
  } else if (motor_state == Back) {
      target_vactual = (int32_t)VACTUAL_VALUE;
  }

  if (motor_state != last_motor_state || (uint32_t)abs(target_vactual) != last_vactual_val) {
      driver.VACTUAL(target_vactual);
      last_vactual_val = (uint32_t)abs(target_vactual);
      last_motor_state = motor_state;
  }
}

// -------------------------------------------------------------------------
// HELPERS
// -------------------------------------------------------------------------
void timer_it_callback(){
    static uint32_t i=0; i++;
    if(i>=50) { if(g_run_cnt == 0) { Serial.println("CPU Reset"); } g_run_cnt=0; i=0; }
    IWDG->KR = 0xAAAA;
    if (is_front) { front_time+=100; if (front_time > timeout) is_error = true; }
    else { front_time = 0; }
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

void Dir_IT_Callback(void){ if((SIGNAL_COUNT_DIR_GPIO_Port->IDR & SIGNAL_COUNT_DIR_Pin)) SIGNAL_COUNT_Get_TIM->CR1 &= ~(TIM_CR1_DIR); else SIGNAL_COUNT_Get_TIM->CR1 |= (TIM_CR1_DIR); }

void USB_Serial_Analys(void){
    static String serial_buf;
    while(Serial.available()){
        char c=Serial.read();
        if(c=='\n'){
            if(serial_buf.startsWith("rt")) { Serial.print("TO="); Serial.println(timeout); }
            else if(serial_buf.startsWith("timeout ")){
                timeout=serial_buf.substring(8).toInt(); EEPROM.put(EEPROM_ADDR_TIMEOUT, timeout);
                Serial.print("Saved TO="); Serial.println(timeout);
            }
            else if(serial_buf.startsWith("steps ")){
                steps=serial_buf.substring(6).toInt();
                EEPROM.put(EEPROM_ADDR_STEPS, steps);
                Serial.print("Saved steps="); Serial.println(steps);
                if(connet_mdm_flag) { REIN_TIM_SIGNAL_COUNT_DeInit(); REIN_TIM_SIGNAL_COUNT_Init(); }
            }
            else if(serial_buf.startsWith("encoder ")){
                encoder_length=fastAtof(serial_buf.substring(8).c_str());
                EEPROM.put(EEPROM_ADDR_ENCODER_LENGTH, encoder_length);
                Serial.print("Saved encoder="); Serial.println(encoder_length);
            }
            else if(serial_buf.startsWith("info")){
                Serial.println("encoder_length="+String(encoder_length));
                Serial.println("timeout="+String(timeout));
                Serial.println("steps="+String(steps));
                Serial.println("allow_error_scale="+String(allow_error_scale));
                Serial.println("maintenance_divider="+String(maintenance_divider));
            }
            else if(serial_buf.startsWith("scale ")){
                allow_error_scale=fastAtof(serial_buf.substring(6).c_str());
                EEPROM.put(EEPROM_ADDR_ERROR_SCALE, allow_error_scale);
                Serial.print("Saved scale="); Serial.println(allow_error_scale);
            }
            else if(serial_buf.startsWith("speed ")){
                SPEED=fastAtof(serial_buf.substring(6).c_str());
                VACTUAL_VALUE=(uint32_t)(SPEED*Move_Divide_NUM*200/60/0.715);
                EEPROM.put(EEPROM_ADDR_SPEED, SPEED);
                Serial.print("Saved speed="); Serial.println(SPEED);
            }
            serial_buf="";
        } else if (c != '\r') serial_buf+=c;
    }
}

bool Check_Connet_MDM(void){
    delay(100);
    pinMode(MDM_DPIN,INPUT);
    bool mdm_state=digitalRead(MDM_DPIN);
    if(mdm_state){ pinMode(MDM_DPIN,INPUT_PULLDOWN); delay(10); return digitalRead(MDM_DPIN); }
    else{ pinMode(MDM_DPIN,INPUT_PULLUP); delay(10); return !digitalRead(MDM_DPIN); }
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
    pinMode(DIR_PIN,INPUT); attachInterrupt(DIR_PIN, Dir_IT_Callback, CHANGE);
}

void Pulse_Receive_Init(void){
    EEPROM.get(EEPROM_ADDR_ERROR_SCALE, allow_error_scale);
    if(isnan(allow_error_scale) || allow_error_scale <= 0.01f) allow_error_scale = 2.0;
    EEPROM.get(EEPROM_ADDR_ENCODER_LENGTH, encoder_length);
    if(isnan(encoder_length) || encoder_length <= 0.001f) encoder_length = 1.73;
    EEPROM.get(EEPROM_ADDR_STEPS, steps);
    if(steps == 0 || steps > 51200) steps = 916;
    if(connet_mdm_flag) REIN_TIM_SIGNAL_COUNT_Init();
}

float fastAtof(const char *s) {
    float val = 0.0f; int sign = 1;
    if (*s == '-') { sign = -1; s++; } else if (*s == '+') { s++; }
    while (*s >= '0' && *s <= '9') { val = val * 10.0f + (*s - '0'); s++; }
    if (*s == '.') { s++; float frac = 1.0f; while (*s >= '0' && *s <= '9') { frac *= 0.1f; val += (*s - '0') * frac; s++; } }
    return sign * val;
}

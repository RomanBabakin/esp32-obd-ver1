#include <Arduino.h> 
#include <can_common.h>
#include <esp32_can.h>
#include <esp32_obd2.h>
#include <TFT_eSPI.h> //драйвер дисплея
#include <lvgl.h> //библиотека визуализации
#include <ui.h> //компоненты визуализации из Squareline Studio 1.4.0
#include <Adafruit_NeoPixel.h> //для управления адресным светодиодом на плате
#include <Wire.h> 
#include <Adafruit_Sensor.h> 
#include <Adafruit_MPU6050.h> //библиотека акселлерометра
#include <math.h> // Для sqrt и fabsf

#define CAN_RX_PIN  4
#define CAN_TX_PIN  5
#define WLED_PIN 48 
#define LED_COUNT 12 
Adafruit_NeoPixel strip(LED_COUNT, WLED_PIN, NEO_GRB + NEO_KHZ800);

static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 280;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);

#if LV_USE_LOG != 0
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

int sda_pin = 47; 
int scl_pin = 45; 
Adafruit_MPU6050 mpu;

const int button_1_Pin = 12;
const int button_2_Pin = 13;

const int TONE_OUTPUT_PIN = 21;
const int TONE_PWM_CHANNEL = 0;

int button_1_State = 0;
int button_2_State = 0; 
byte lastbutton_1_State = HIGH;
byte lastbutton_2_State = HIGH;

int screenSelected = 1;
int buzzerStatus = 0;
int buzzerStatus_prev = 0;
int buzzerLoopCount = 0;

int obdRPM = 800;
int obdOilTemp = 82;
int obdCoolantTemp = 95; 
int obdAirTemp = 35;
double obdBoost = 200; 
double obdBoostBars = 0.2; 
double displayedBoost = 200; 
int obdSpeed = 0;
int calculatedGear = 0;

int obdRPM_tmp = 800;
int obdOilTemp_tmp = 82;
int obdCoolantTemp_tmp = 95; 
int obdAirTemp_tmp = 35;
double obdBoost_tmp = 200;
int obdSpeed_tmp = 0;

int obdOilTemp_prev = 82;
int obdCoolantTemp_prev = 95; 
int obdAirTemp_prev = 35;
int obdRPM_prev = 400;
double obdBoost_prev = 0.1;
double obdBoostBars_prev = 0.1;
double displayedBoost_prev = 700;
int obdSpeed_prev = 0;

int obdParameter = 1;

// Векторы осей автомобиля в координатах сенсора, определяются при калибровке
float u_vehicle_forward_S[3] = {1.0f, 0.0f, 0.0f}; // Продольная ось автомобиля (вперед)
float u_vehicle_left_S[3]    = {0.0f, 1.0f, 0.0f}; // Поперечная ось автомобиля (влево)
const float gravity_mps2 = 9.80665f; 

String textWrapper = "String";

volatile bool canConnected = false;

TaskHandle_t Task1_buzzer;
TaskHandle_t Task2_obdReader;
TaskHandle_t Task_CanManager; 

void tftPrintCoolantTemp ();
void tftPrintOilTemp ();
void tftPrintAirTemp ();
void tftPrintRPM ();
void tftPrintBoost();
void tftPrintAccel();
void tftPrintGear();

// Вспомогательная функция для нормализации вектора
void normalizeVector(float v[3]) {
    float mag = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (mag > 0.0001f) { // Избегаем деления на ноль
        v[0] /= mag;
        v[1] /= mag;
        v[2] /= mag;
    } else {
        v[0] = 0; v[1] = 0; v[2] = 0; // В случае нулевого вектора
    }
}

// Вспомогательная функция для векторного произведения v_out = a x b
void crossProduct(const float a[3], const float b[3], float v_out[3]) {
    v_out[0] = a[1]*b[2] - a[2]*b[1];
    v_out[1] = a[2]*b[0] - a[0]*b[2];
    v_out[2] = a[0]*b[1] - a[1]*b[0];
}

// Вспомогательная функция для скалярного произведения
float dotProduct(const float a[3], const float b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}


void my_disp_flush( lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p ) {
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );
    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();
    lv_disp_flush_ready( disp_drv );
}

void obdRead () {
  if (obdParameter == 1) {
    obdRPM_tmp = OBD2.pidRead(ENGINE_RPM); 
    if (obdRPM_tmp < 10000) {obdRPM = obdRPM_tmp;}; 
  } else if (obdParameter == 2) {
    obdBoost_tmp = OBD2.pidRead(INTAKE_MANIFOLD_ABSOLUTE_PRESSURE); 
    if (obdBoost_tmp < 1500) {obdBoost = obdBoost_tmp;};
    obdBoostBars = (obdBoost/100.0)-1.0; 
    displayedBoost= obdBoostBars*1000; 
  } else if (obdParameter == 3) {
    obdOilTemp_tmp = OBD2.pidRead(ENGINE_OIL_TEMPERATURE); 
    if (obdOilTemp_tmp < 300) {obdOilTemp = obdOilTemp_tmp;};
  } else if (obdParameter == 4) {
    obdRPM_tmp = OBD2.pidRead(ENGINE_RPM);
    if (obdRPM_tmp < 10000) {obdRPM = obdRPM_tmp;};
  } else if (obdParameter == 5) {
    obdBoost_tmp = OBD2.pidRead(INTAKE_MANIFOLD_ABSOLUTE_PRESSURE);
    if (obdBoost_tmp < 1500) {obdBoost = obdBoost_tmp;};
    obdBoostBars = (obdBoost/100.0)-1.0; 
    displayedBoost= obdBoostBars*1000;
  } else if (obdParameter == 6) {
    obdCoolantTemp_tmp = OBD2.pidRead(ENGINE_COOLANT_TEMPERATURE); 
    if (obdCoolantTemp_tmp < 300) {obdCoolantTemp = obdCoolantTemp_tmp;};
  } else if (obdParameter == 7) {
    obdRPM_tmp = OBD2.pidRead(ENGINE_RPM);
    if (obdRPM_tmp < 10000) {obdRPM = obdRPM_tmp;};
  } else if (obdParameter == 8) {
    obdBoost_tmp = OBD2.pidRead(INTAKE_MANIFOLD_ABSOLUTE_PRESSURE);
    if (obdBoost_tmp < 1500) {obdBoost = obdBoost_tmp;};
    obdBoostBars = (obdBoost/100.0)-1.0; 
    displayedBoost= obdBoostBars*1000;
  } else if (obdParameter == 9) {
    obdAirTemp_tmp = OBD2.pidRead(AIR_INTAKE_TEMPERATURE); 
    if (obdAirTemp_tmp < 300) {obdAirTemp = obdAirTemp_tmp;};
    obdParameter = 0;
  }; 
  obdParameter = obdParameter + 1;
};

void obdClioRead () {
  if (obdParameter == 1) {
    obdRPM_tmp = OBD2.pidRead(ENGINE_RPM);
    if (obdRPM_tmp < 10000) {obdRPM = obdRPM_tmp;};
  } else if (obdParameter == 2) {
    obdCoolantTemp_tmp = OBD2.pidRead(ENGINE_COOLANT_TEMPERATURE);
    if (obdCoolantTemp_tmp < 300) {obdCoolantTemp = obdCoolantTemp_tmp;};
  } else if (obdParameter == 3) {
    obdRPM_tmp = OBD2.pidRead(ENGINE_RPM);
    if (obdRPM_tmp < 10000) {obdRPM = obdRPM_tmp;};
  } else if (obdParameter == 4) {
    obdSpeed_tmp = OBD2.pidRead(VEHICLE_SPEED);
    if (obdSpeed_tmp < 300) {obdSpeed = obdSpeed_tmp;};
  } else if (obdParameter == 5) {
    obdRPM_tmp = OBD2.pidRead(ENGINE_RPM);
    if (obdRPM_tmp < 10000) {obdRPM = obdRPM_tmp;};
  } else if (obdParameter == 6) {
    obdAirTemp_tmp = OBD2.pidRead(AIR_INTAKE_TEMPERATURE);
    if (obdAirTemp_tmp < 300) {obdAirTemp = obdAirTemp_tmp;};
    obdParameter = 0;
  }
  obdParameter = obdParameter + 1;
}

void buzzerTask (void * parameter) {
  for (;;) {
  vTaskDelay(pdMS_TO_TICKS(10)); 
  if (buzzerStatus == 0) {
    vTaskDelay(pdMS_TO_TICKS(100));
  } else if (buzzerStatus == 1) {
    if (buzzerStatus != buzzerStatus_prev) {
      buzzerLoopCount = 0;
      buzzerStatus_prev = buzzerStatus;
    };
    if (buzzerLoopCount <= 3) {
      ledcWriteTone(TONE_PWM_CHANNEL, 1000);
      vTaskDelay(pdMS_TO_TICKS(70));
      ledcWrite(TONE_PWM_CHANNEL, 0);
      vTaskDelay(pdMS_TO_TICKS(100));
      buzzerLoopCount = buzzerLoopCount + 1;
    } else {
      vTaskDelay(pdMS_TO_TICKS(100));
    };
  } else if (buzzerStatus == 2) {
   if (buzzerStatus != buzzerStatus_prev) {
      buzzerLoopCount = 0;
      buzzerStatus_prev = buzzerStatus;
    };
    if (buzzerLoopCount <= 3) {
      ledcWriteTone(TONE_PWM_CHANNEL, 1900);
      vTaskDelay(pdMS_TO_TICKS(70));
      ledcWrite(TONE_PWM_CHANNEL, 0);
      vTaskDelay(pdMS_TO_TICKS(100));
      buzzerLoopCount = buzzerLoopCount + 1;
    } else {
      vTaskDelay(pdMS_TO_TICKS(100));
    };
  } else if (buzzerStatus == 3) {
    buzzerLoopCount = 0;
    buzzerStatus_prev = buzzerStatus;
    ledcWriteTone(TONE_PWM_CHANNEL, 1000);
    vTaskDelay(pdMS_TO_TICKS(100));
    ledcWrite(TONE_PWM_CHANNEL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    ledcWriteTone(TONE_PWM_CHANNEL, 1000);
    vTaskDelay(pdMS_TO_TICKS(100));
    ledcWrite(TONE_PWM_CHANNEL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    ledcWriteTone(TONE_PWM_CHANNEL, 1000);
    vTaskDelay(pdMS_TO_TICKS(100));
    ledcWrite(TONE_PWM_CHANNEL, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
  } else if (buzzerStatus == 4) {
    buzzerLoopCount = 0;
    buzzerStatus_prev = buzzerStatus;
    ledcWriteTone(TONE_PWM_CHANNEL, 1900);
    vTaskDelay(pdMS_TO_TICKS(100));
    ledcWrite(TONE_PWM_CHANNEL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    ledcWriteTone(TONE_PWM_CHANNEL, 1900);
    vTaskDelay(pdMS_TO_TICKS(100));
    ledcWrite(TONE_PWM_CHANNEL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    ledcWriteTone(TONE_PWM_CHANNEL, 1900);
    vTaskDelay(pdMS_TO_TICKS(100));
    ledcWrite(TONE_PWM_CHANNEL, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
  };
  } 
}; 

void obdReaderTask (void * parameter) {
  for (;;) {
    if (canConnected) {
      obdRead(); 
      vTaskDelay(pdMS_TO_TICKS(10)); 
    } else {
      vTaskDelay(pdMS_TO_TICKS(500)); 
    }
  } 
};

void canConnectionManagerTask(void *pvParameters) {
    CAN0.setCANPins((gpio_num_t)CAN_RX_PIN, (gpio_num_t)CAN_TX_PIN);
    CAN0.begin(500000); 

    for (;;) {
        if (!canConnected) {
            Serial.print(F("Attempting to connect to OBD2 CAN bus ... "));
            strip.setPixelColor(0, 0, 0, 200); // Синий
            strip.show();

            if (OBD2.begin()) { 
                Serial.println(F("success"));
                canConnected = true;
                strip.setPixelColor(0, 200, 0, 0); // Зеленый
                strip.show();
            } else {
                Serial.println(F("failed! Retrying in 5s..."));
                canConnected = false; 
                vTaskDelay(pdMS_TO_TICKS(5000)); 
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10000)); 
        }
    }
}

// Обновленная функция калибровки акселерометра
void calibrateAccelerometer() {
  Serial.println("Начинаю калибровку акселерометра. Автомобиль должен стоять РОВНО и НЕПОДВИЖНО.");
  
  const int samples = 100; 
  double sum_ax_mps2 = 0;
  double sum_ay_mps2 = 0;
  double sum_az_mps2 = 0;
  
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp; 
    mpu.getEvent(&a, &g, &temp);
    sum_ax_mps2 += a.acceleration.x;
    sum_ay_mps2 += a.acceleration.y;
    sum_az_mps2 += a.acceleration.z;
    delay(10); 
  }
  Serial.println();
  
  float avg_ax_S = sum_ax_mps2 / samples;
  float avg_ay_S = sum_ay_mps2 / samples;
  float avg_az_S = sum_az_mps2 / samples;

  Serial.print("Средние сырые значения акселерометра (м/с^2): Ax="); Serial.print(avg_ax_S);
  Serial.print(", Ay="); Serial.print(avg_ay_S);
  Serial.print(", Az="); Serial.println(avg_az_S);

  // Вектор силы тяжести в координатах сенсора
  float g_S[3] = {avg_ax_S, avg_ay_S, avg_az_S};

  // 1. Вертикальная ось автомобиля (vZ_S, направлена ВВЕРХ) в координатах сенсора
  float vZ_S[3] = {-g_S[0], -g_S[1], -g_S[2]};
  normalizeVector(vZ_S);

  // 2. Предполагаемая поперечная ось сенсора (sY_S, например, ВЛЕВО автомобиля)
  // Мы предполагаем, что ось Y сенсора примерно соответствует автомобильной "влево".
  // Если это не так, эту "догадку" нужно изменить (например, на (1,0,0) если ось X сенсора = "влево")
  float sY_guess_S[3] = {0.0f, 1.0f, 0.0f}; // Ось Y сенсора

  // 3. Продольная ось автомобиля (vX_S, ВПЕРЕД) в координатах сенсора
  // vX_S = normalize(sY_guess_S x vZ_S)
  crossProduct(sY_guess_S, vZ_S, u_vehicle_forward_S);
  normalizeVector(u_vehicle_forward_S);

  // 4. Поперечная ось автомобиля (vY_S, ВЛЕВО) в координатах сенсора
  // vY_S = normalize(vZ_S x vX_S)
  crossProduct(vZ_S, u_vehicle_forward_S, u_vehicle_left_S);
  normalizeVector(u_vehicle_left_S); // Должен быть уже нормализован, если vZ_S и vX_S ортонормальны

  Serial.println("Калибровка акселерометра завершена.");
  Serial.print("Вектор ВПЕРЕД автомобиля (в коорд. сенсора): Vx="); Serial.print(u_vehicle_forward_S[0]);
  Serial.print(", Vy="); Serial.print(u_vehicle_forward_S[1]);
  Serial.print(", Vz="); Serial.println(u_vehicle_forward_S[2]);
  Serial.print("Вектор ВЛЕВО автомобиля (в коорд. сенсора): Vx="); Serial.print(u_vehicle_left_S[0]);
  Serial.print(", Vy="); Serial.print(u_vehicle_left_S[1]);
  Serial.print(", Vz="); Serial.println(u_vehicle_left_S[2]);
}


void setup() {
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  Serial.begin(115200);
  
  strip.begin(); 
  strip.setBrightness(10);
  strip.setPixelColor(0, 0, 255, 0); // Красный
  strip.show();

  pinMode(button_1_Pin, INPUT_PULLUP);
  pinMode(button_2_Pin, INPUT_PULLUP);

  ledcAttachPin(TONE_OUTPUT_PIN, TONE_PWM_CHANNEL);

  Serial.println(F("OBD2 data printer"));

  tft.init();
  // digitalWrite(TFT_BL, LOW); // Пользователь закомментировал
  
  // String LVGL_Arduino = "Hello Arduino! "; // Пользователь закомментировал
  // LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch(); // Пользователь закомментировал
  // Serial.println( LVGL_Arduino ); // Пользователь закомментировал
  // Serial.println( "I am LVGL_Arduino" ); // Пользователь закомментировал
  lv_init();
  #if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print ); 
  #endif
  lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init( &disp_drv );
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register( &disp_drv );
  // lv_obj_t *label = lv_label_create( lv_scr_act() ); // Пользователь закомментировал связанный код
  // lv_label_set_text( label, "Hello Ardino and LVGL!"); // Пользователь закомментировал
  // lv_obj_align( label, LV_ALIGN_CENTER, 0, 0 ); // Пользователь закомментировал
  ui_init();


  Wire.setPins(sda_pin, scl_pin); 
  Wire.begin(); 

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G); 
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   

  calibrateAccelerometer(); 

  xTaskCreatePinnedToCore(
    canConnectionManagerTask,   
    "CAN Manager Task",         
    4096,                       
    NULL,                       
    2,                          
    &Task_CanManager,           
    0);                         

  xTaskCreatePinnedToCore(
    buzzerTask, 
    "Buzzer Task",    
    8000,      
    NULL,    
    2,    
    &Task1_buzzer,    
    0);

  xTaskCreatePinnedToCore(
    obdReaderTask, 
    "OBD Reader Task",    
    8000,      
    NULL,    
    2,    
    &Task2_obdReader,    
    0);
};

void loop() {
  button_1_State = digitalRead(button_1_Pin);
  if (button_1_State != lastbutton_1_State) {
    lastbutton_1_State = button_1_State;
    if (button_1_State == LOW) {
      Serial.println(F("Button 1 click"));
      // digitalWrite(TFT_BL, HIGH); // Пользователь закомментировал 
      if (screenSelected == 1) {
        screenSelected = 2;
        lv_scr_load(ui_Screen2);
      } else if (screenSelected == 2) {
        screenSelected = 3;
        lv_scr_load(ui_Screen3);
      } else if (screenSelected == 3) {
        screenSelected = 4;
        lv_scr_load(ui_Screen4);
      } else if (screenSelected == 4) {
        screenSelected = 5;
        lv_scr_load(ui_Screen5);
      } else if (screenSelected == 5) {
        screenSelected = 1;
        lv_scr_load(ui_Screen1);
      }
    }
  } 

  button_2_State = digitalRead(button_2_Pin);
  if (button_2_State != lastbutton_2_State) {
    lastbutton_2_State = button_2_State;
    if (button_2_State == LOW) {
      Serial.println(F("Button 2 click"));

      if (obdOilTemp == 135) {obdOilTemp = 98;};
      obdOilTemp = obdOilTemp +1;
      if (obdCoolantTemp == 135) {obdCoolantTemp= 105;};
      obdCoolantTemp = obdCoolantTemp +1;
      obdAirTemp = obdAirTemp +1;
      obdBoostBars = obdBoostBars +0.1;
      if (obdBoostBars >= 1.3) { 
        obdBoostBars = 0;
      };
      displayedBoost = displayedBoost + 100;
      if (displayedBoost >= 1300) { 
        displayedBoost = 0;
      };
      obdRPM = obdRPM + 80;
    }
  } 
  
  if (screenSelected == 1) {
    tftPrintCoolantTemp();
    tftPrintOilTemp();
    tftPrintAirTemp();
  } else if (screenSelected == 2) {
    tftPrintBoost();
  } else if (screenSelected == 3) {
    tftPrintRPM();
  } else if (screenSelected == 4) {
    tftPrintAccel();
  } else if (screenSelected == 5) {
    tftPrintGear();
  };
  
  lv_timer_handler(); 
  delay(5); 
};

// Функция вывода температуры ОЖ с исходной логикой buzzerStatus
void tftPrintCoolantTemp () {
    if (obdCoolantTemp < 115) {
      if (buzzerStatus == 3) {buzzerStatus = 0;};
      lv_obj_clear_state(ui_coolantTemp, LV_STATE_USER_2);
      lv_obj_clear_state(ui_coolantLabel, LV_STATE_USER_2);
      lv_obj_clear_state(ui_coolantRed, LV_STATE_USER_2);
      lv_obj_clear_state(ui_coolantTemp, LV_STATE_USER_3);
      lv_obj_clear_state(ui_coolantLabel, LV_STATE_USER_3);
      lv_obj_clear_state(ui_coolantRed, LV_STATE_USER_3);
    } else if (115 <= obdCoolantTemp && obdCoolantTemp  < 130) {
      if (buzzerStatus == 0) {buzzerStatus = 1;}; // Предупреждение ОЖ, если нет других сигналов
      // if (buzzerStatus == 2) {buzzerStatus = 1;}; // Эта строка была закомментирована в оригинале
      if (buzzerStatus == 3) {buzzerStatus = 0;}; // Сброс тревоги ОЖ, если температура упала
      lv_obj_add_state(ui_coolantTemp, LV_STATE_USER_2);
      lv_obj_add_state(ui_coolantLabel, LV_STATE_USER_2);
      lv_obj_add_state(ui_coolantRed, LV_STATE_USER_2);
      lv_obj_clear_state(ui_coolantTemp, LV_STATE_USER_3);
      lv_obj_clear_state(ui_coolantLabel, LV_STATE_USER_3);
      lv_obj_clear_state(ui_coolantRed, LV_STATE_USER_3);
    } else if (130 <= obdCoolantTemp) {
      if (buzzerStatus != 4) {buzzerStatus = 3;}; // Тревога ОЖ, если нет тревоги по маслу
      lv_obj_add_state(ui_coolantTemp, LV_STATE_USER_3);
      lv_obj_add_state(ui_coolantLabel, LV_STATE_USER_3);
      lv_obj_add_state(ui_coolantRed, LV_STATE_USER_3);
    };
    lv_label_set_text_fmt(ui_coolantTemp, "%d °C", obdCoolantTemp);
}

// Функция вывода температуры масла с исходной логикой buzzerStatus
void tftPrintOilTemp () {
    if (obdOilTemp < 110) {
      if (buzzerStatus == 4) {buzzerStatus = 0;}; // Сброс тревоги масла
      lv_obj_clear_state(ui_oilTemp, LV_STATE_USER_2);
      lv_obj_clear_state(ui_oilLabel, LV_STATE_USER_2);
      lv_obj_clear_state(ui_oilRed, LV_STATE_USER_2);
      lv_obj_clear_state(ui_oilTemp, LV_STATE_USER_3);
      lv_obj_clear_state(ui_oilLabel, LV_STATE_USER_3);
      lv_obj_clear_state(ui_oilRed, LV_STATE_USER_3);
    } else if (110 <= obdOilTemp && obdOilTemp  < 120) {
      if (buzzerStatus == 0) {buzzerStatus = 2;}; // Предупреждение по маслу
      if (buzzerStatus == 1) {buzzerStatus = 2;}; // Предупреждение по маслу перекрывает предупреждение ОЖ
      if (buzzerStatus == 4) {buzzerStatus = 0;}; // Сброс тревоги масла, если температура упала
      lv_obj_add_state(ui_oilTemp, LV_STATE_USER_2);
      lv_obj_add_state(ui_oilLabel, LV_STATE_USER_2);
      lv_obj_add_state(ui_oilRed, LV_STATE_USER_2);
      lv_obj_clear_state(ui_oilTemp, LV_STATE_USER_3);
      lv_obj_clear_state(ui_oilLabel, LV_STATE_USER_3);
      lv_obj_clear_state(ui_oilRed, LV_STATE_USER_3);
    } else if (120 <= obdOilTemp) {
      buzzerStatus = 4; // Тревога по маслу - наивысший приоритет
      lv_obj_add_state(ui_oilTemp, LV_STATE_USER_3);
      lv_obj_add_state(ui_oilLabel, LV_STATE_USER_3);
      lv_obj_add_state(ui_oilRed, LV_STATE_USER_3);
    };
    lv_label_set_text_fmt(ui_oilTemp, "%d °C", obdOilTemp);
}

void tftPrintAirTemp () {
    if (obdAirTemp < 50) {
      lv_obj_clear_state(ui_intakeTemp, LV_STATE_USER_2);
      lv_obj_clear_state(ui_intakeLabel, LV_STATE_USER_2);
    } else if  (50 <= obdAirTemp) {
      lv_obj_add_state(ui_intakeTemp, LV_STATE_USER_2);
      lv_obj_add_state(ui_intakeLabel, LV_STATE_USER_2);
    };
    lv_label_set_text_fmt(ui_intakeTemp, "%d °C", obdAirTemp);
}

void tftPrintBoost () {
  lv_label_set_text_fmt(ui_boost, "%2.2f", obdBoostBars);
  lv_bar_set_value(ui_boostBar, (int32_t)displayedBoost, LV_ANIM_OFF); 
}

void tftPrintRPM () {
  lv_label_set_text_fmt(ui_rpm, "%d", obdRPM);
  lv_arc_set_value(ui_rpmArc, (int16_t)obdRPM); 
};

// Обновленная функция отображения ускорений
void tftPrintAccel() {
  sensors_event_t event_a, event_g, event_temp;
  mpu.getEvent(&event_a, &event_g, &event_temp); 
  
  float current_a_S[3] = {event_a.acceleration.x, event_a.acceleration.y, event_a.acceleration.z};

  // Продольное ускорение автомобиля (вдоль u_vehicle_forward_S)
  float longitudinal_G = dotProduct(current_a_S, u_vehicle_forward_S) / gravity_mps2;
  
  // Поперечное ускорение автомобиля (вдоль u_vehicle_left_S)
  float lateral_G = dotProduct(current_a_S, u_vehicle_left_S) / gravity_mps2;
  
  // Обновление LVGL виджетов
  // Поперечное ускорение (X на экране), текстовый вывод всегда положительный
  lv_label_set_text_fmt(ui_accelX, "X: %1.1f G", fabsf(lateral_G)); 
  // Продольное ускорение (Y на экране), текстовый вывод всегда положительный
  lv_label_set_text_fmt(ui_accelY, "Y: %1.1f G", fabsf(longitudinal_G)); 
  
  if (ui_accelPointer != NULL) {
    // Для поперечного (X на экране): инвертируем lateral_G для инверсии направления указателя
    lv_obj_set_x(ui_accelPointer, (lv_coord_t)(-lateral_G * 35)); 
    // Для продольного (Y на экране), инвертируем движение курсора по просьбе пользователя:
    lv_obj_set_y(ui_accelPointer, (lv_coord_t)((longitudinal_G * 35) + 180)); 
  }
}

void tftPrintGear () {
  if (obdRPM < 1500) {
    lv_obj_clear_state(ui_gearBackground, LV_STATE_USER_1);
    lv_obj_clear_state(ui_gearBackground, LV_STATE_USER_2);
    lv_obj_clear_state(ui_gearBackground, LV_STATE_USER_3);
  } else if (1500 <= obdRPM && obdRPM < 7000) {
    lv_obj_add_state(ui_gearBackground, LV_STATE_USER_1);
    lv_obj_clear_state(ui_gearBackground, LV_STATE_USER_2);
    lv_obj_clear_state(ui_gearBackground, LV_STATE_USER_3);
  } else if (7000 <= obdRPM && obdRPM < 7500) {
    lv_obj_clear_state(ui_gearBackground, LV_STATE_USER_1);
    lv_obj_add_state(ui_gearBackground, LV_STATE_USER_2);
    lv_obj_clear_state(ui_gearBackground, LV_STATE_USER_3);
  } else if (7500 <= obdRPM && obdRPM < 9900) {
    lv_obj_clear_state(ui_gearBackground, LV_STATE_USER_1);
    lv_obj_clear_state(ui_gearBackground, LV_STATE_USER_2);
    lv_obj_add_state(ui_gearBackground, LV_STATE_USER_3);
  };
  
  if (canConnected && obdSpeed > 0) { 
    float ratio = (float)obdRPM / obdSpeed;
    if (100.0 < ratio && ratio < 140.0) {
      lv_label_set_text( ui_gearDisp, "1");
    } else if (65.0 < ratio && ratio < 85.0) {
      lv_label_set_text( ui_gearDisp, "2");
    } else if (53.0 < ratio && ratio < 59.0) {
      lv_label_set_text( ui_gearDisp, "3");
    } else if (44.0 < ratio && ratio < 48.0) {
      lv_label_set_text( ui_gearDisp, "4");
    } else if (37.0 < ratio && ratio < 41.0) {
      lv_label_set_text( ui_gearDisp, "5");
    } else if (32.0 < ratio && ratio < 35.0) {
      lv_label_set_text( ui_gearDisp, "6");
    } else {
      lv_label_set_text( ui_gearDisp, "N");
    };
  } else {
      lv_label_set_text( ui_gearDisp, "N"); 
  }
}

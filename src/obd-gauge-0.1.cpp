// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
#include <Arduino.h>
#include <esp32_can.h> // the ESP32_OBD2 library depends on the https://github.com/collin80/esp32_can and https://github.com/collin80/can_common CAN libraries
#include <esp32_obd2.h>
#include <TFT_eSPI.h>
// #include "Free_Fonts.h"
#include <lvgl.h>
#include <ui.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#define CAN_RX_PIN  4
#define CAN_TX_PIN  5
//work fine with rx (orange) = 4, tx (red) = 5
#define WLED_PIN 48
#define LED_COUNT 12
Adafruit_NeoPixel strip(LED_COUNT, WLED_PIN, NEO_GRB + NEO_KHZ800);

static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 320;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

int sda_pin = 47; // I2C SDA
int scl_pin = 45; // I2C SCL

Adafruit_MPU6050 mpu;

const int button_1_Pin = 12;
const int button_2_Pin = 13;
const int redLedPin = 9;
const int yellowLedPin = 11;
const int greenLedPin = 10;
const int TONE_OUTPUT_PIN = 21;
const int TONE_PWM_CHANNEL = 0;

int button_1_State = 0;
int button_2_State = 0; 
int shiftLightState = 0; 
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
int obdBoost = 200; //kPa
double obdBoostBars = 0.2; //to Bars
int displayedBoost = 200; //from 0 to 1500
int obdSpeed = 0;

int  calculatedGear = 0;

int obdRPM_tmp = 800;
int obdOilTemp_tmp = 82;
int obdCoolantTemp_tmp = 95; 
int obdAirTemp_tmp = 35;
int obdBoost_tmp = 200;
int obdSpeed_tmp = 0;

int obdOilTemp_prev = 82;
int obdCoolantTemp_prev = 95; 
int obdAirTemp_prev = 35;
int obdRPM_prev = 400;
int obdBoost_prev = 0.1;
int obdBoostBars_prev = 0.1;
int displayedBoost_prev = 700;
int obdSpeed_prev = 0;

int obdParameter = 1;

double accelY = 0;
double accelZ = 0;


String textWrapper = "String";

void tftPrintCoolantTemp ();
void tftPrintOilTemp ();
void tftPrintAirTemp ();
void tftPrintRPM ();
void tftPrintBoost();
void tftPrintAccel();
void tftPrintGear();
void processPid(int pid);

TaskHandle_t Task1_buzzer;
TaskHandle_t Task2_obdReader;
TaskHandle_t Task3_brightness;



void my_disp_flush( lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p ) {
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );
    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();
    lv_disp_flush_ready( disp_drv );
}

//read from OBD2 and filter
void obdRead () {
  //Serial.println(obdParameter);
  if (obdParameter == 1) {
    obdRPM_tmp = OBD2.pidRead(ENGINE_RPM);
    if (obdRPM_tmp < 10000) {obdRPM = obdRPM_tmp;};
  } else if (obdParameter == 2) {
    obdBoost_tmp = OBD2.pidRead(INTAKE_MANIFOLD_ABSOLUTE_PRESSURE);
    if (obdBoost_tmp < 500) {obdBoost = obdBoost_tmp;};
    obdBoostBars = (obdBoost/100)-1;
    displayedBoost= obdBoostBars*100;
    // Serial.print("obdBoost_tmp:");
    // Serial.println(obdBoost_tmp);
    // Serial.print("obdBoostBars:");
    // Serial.println(obdBoostBars);
    // Serial.print("displayedBoost:");
    // Serial.println(displayedBoost);
  } else if (obdParameter == 3) {
    obdOilTemp_tmp = OBD2.pidRead(ENGINE_OIL_TEMPERATURE);
    if (obdOilTemp_tmp < 300) {obdOilTemp = obdOilTemp_tmp;};
  } else if (obdParameter == 4) {
    obdRPM_tmp = OBD2.pidRead(ENGINE_RPM);
    if (obdRPM_tmp < 10000) {obdRPM = obdRPM_tmp;};
  } else if (obdParameter == 5) {
    obdBoost_tmp = OBD2.pidRead(INTAKE_MANIFOLD_ABSOLUTE_PRESSURE);
    if (obdBoost_tmp < 500) {obdBoost = obdBoost_tmp;};
    obdBoostBars = (obdBoost/100)-1;
    displayedBoost= obdBoostBars*100;
  } else if (obdParameter == 6) {
    obdCoolantTemp_tmp = OBD2.pidRead(ENGINE_COOLANT_TEMPERATURE);
    if (obdCoolantTemp_tmp < 300) {obdCoolantTemp = obdCoolantTemp_tmp;};
  } else if (obdParameter == 7) {
    obdRPM_tmp = OBD2.pidRead(ENGINE_RPM);
    if (obdRPM_tmp < 10000) {obdRPM = obdRPM_tmp;};
  } else if (obdParameter == 8) {
    obdBoost_tmp = OBD2.pidRead(INTAKE_MANIFOLD_ABSOLUTE_PRESSURE);
    if (obdBoost_tmp < 500) {obdBoost = obdBoost_tmp;};
    obdBoostBars = (obdBoost/100)-1;
    displayedBoost= obdBoostBars*100;
  } else if (obdParameter == 9) {
    obdAirTemp_tmp = OBD2.pidRead(AIR_INTAKE_TEMPERATURE);
    if (obdAirTemp_tmp < 300) {obdAirTemp = obdAirTemp_tmp;};
    obdParameter = 0;
  }; 
  obdParameter = obdParameter + 1;
  Serial.print("OBD PID read");
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

void obdClioRPMSpeedToSerial () {
  obdRPM_tmp = OBD2.pidRead(ENGINE_RPM);
  delay(30);
  obdSpeed_tmp = OBD2.pidRead(VEHICLE_SPEED);
  Serial.println("- - - - -");
  Serial.print(obdRPM_tmp);
  Serial.print(" / ");
  Serial.println(obdSpeed_tmp);
  delay(200);
}

void obdCanInit () {
  strip.setPixelColor(0, 0, 0, 200);
  strip.show();
  CAN0.setCANPins((gpio_num_t)CAN_RX_PIN, (gpio_num_t)CAN_TX_PIN);
  while (true) {
    Serial.print(F("Attempting to connect to OBD2 CAN bus ... "));
    if (!OBD2.begin()) {
      Serial.println(F("failed!"));
      delay(1000);
    } else {
      Serial.println(F("success"));
      break;
    }
  }
  strip.setPixelColor(0, 0, 255, 200);
  strip.show();
};

void buzzerTask (void * parameter) {
  for (;;) {
  delay(10);
  if (buzzerStatus == 0) {
    delay(100);
  } else if (buzzerStatus == 1) {
    //coolant warn
    if (buzzerStatus != buzzerStatus_prev) {
      buzzerLoopCount = 0;
      buzzerStatus_prev = buzzerStatus;
    };
    if (buzzerLoopCount <= 3) {
      ledcWriteTone(TONE_PWM_CHANNEL, 1000);
      delay(70);
      ledcWrite(TONE_PWM_CHANNEL, 0);
      delay(100);
      buzzerLoopCount = buzzerLoopCount + 1;
    } else {
      delay(100);
    };
  } else if (buzzerStatus == 2) {
    //oil warn
   if (buzzerStatus != buzzerStatus_prev) {
      buzzerLoopCount = 0;
      buzzerStatus_prev = buzzerStatus;
    };
    if (buzzerLoopCount <= 3) {
      ledcWriteTone(TONE_PWM_CHANNEL, 1900);
      delay(70);
      ledcWrite(TONE_PWM_CHANNEL, 0);
      delay(100);
      buzzerLoopCount = buzzerLoopCount + 1;
    } else {
      delay(100);
    };
  } else if (buzzerStatus == 3) {
    //coolant alarm
    buzzerLoopCount = 0;
    buzzerStatus_prev = buzzerStatus;
    ledcWriteTone(TONE_PWM_CHANNEL, 1000);
    delay(100);
    ledcWrite(TONE_PWM_CHANNEL, 0);
    delay(100);
    ledcWriteTone(TONE_PWM_CHANNEL, 1000);
    delay(100);
    ledcWrite(TONE_PWM_CHANNEL, 0);
    delay(100);
    ledcWriteTone(TONE_PWM_CHANNEL, 1000);
    delay(100);
    ledcWrite(TONE_PWM_CHANNEL, 0);
    delay(1000);
  } else if (buzzerStatus == 4) {
    //oil alarm
    buzzerLoopCount = 0;
    buzzerStatus_prev = buzzerStatus;
    ledcWriteTone(TONE_PWM_CHANNEL, 1900);
    delay(100);
    ledcWrite(TONE_PWM_CHANNEL, 0);
    delay(100);
    ledcWriteTone(TONE_PWM_CHANNEL, 1900);
    delay(100);
    ledcWrite(TONE_PWM_CHANNEL, 0);
    delay(100);
    ledcWriteTone(TONE_PWM_CHANNEL, 1900);
    delay(100);
    ledcWrite(TONE_PWM_CHANNEL, 0);
    delay(1000);
  };
  // ledcWriteTone(TONE_PWM_CHANNEL, 1000);
  // delay(100);    
  // ledcWrite(TONE_PWM_CHANNEL, 0);
  // delay(500);
  } //end loop
}; //end buzzer task

void obdReaderTask (void * parameter) {
  for (;;) {
  delay(10);
  //obdRead(); //read one OBD PID
  //obdClioRead(); //read one OBD PID  
  } //end loop
}; //end OBD Reader task

void brightnessTask (void * parameter) {
  for (;;) {
  Serial.print("Brightness: ");
  Serial.println(analogRead(1));
  delay(5000);
  } //end loop
}; //end Brightness Task





void setup() {
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  Serial.begin(115200);
  //while (!Serial);
  
  strip.begin(); 
  strip.setBrightness(10);
  strip.setPixelColor(0, 0, 255, 0);
  strip.show();

  pinMode(redLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(button_1_Pin, INPUT_PULLUP);
  pinMode(button_2_Pin, INPUT_PULLUP);

  ledcAttachPin(TONE_OUTPUT_PIN, TONE_PWM_CHANNEL);

  Serial.println(F("OBD2 data printer"));

  tft.init();
  digitalWrite(TFT_BL, LOW);
  //lvgl example
  String LVGL_Arduino = "Hello Arduino! ";
  LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
  Serial.println( LVGL_Arduino );
  Serial.println( "I am LVGL_Arduino" );
  lv_init();
  #if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print ); /* register print function for debugging */
  #endif
  lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init( &disp_drv );
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  disp_drv.sw_rotate = 1;
  lv_disp_drv_register( &disp_drv );
  lv_obj_t *label = lv_label_create( lv_scr_act() );
  lv_label_set_text( label, "Hello Ardino and LVGL!");
  lv_obj_align( label, LV_ALIGN_CENTER, 0, 0 );
  //lvgl example end
  lv_disp_set_rotation(NULL, LV_DISP_ROT_90);
  ui_init();

  xTaskCreatePinnedToCore(buzzerTask, "Buzzer Task",    8000,      NULL,    2,    &Task1_buzzer,    0);
  xTaskCreatePinnedToCore(obdReaderTask, "OBD Reader Task",    8000,      NULL,    2,    &Task2_obdReader,    0);
  xTaskCreatePinnedToCore(brightnessTask, "Brightness Task",    8000,      NULL,    2,    &Task3_brightness,    1);

  Wire.setPins(sda_pin, scl_pin); // Set the I2C pins before begin
  Wire.begin(); // join i2c bus (address optional for master)

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(500);
  digitalWrite(TFT_BL, HIGH);
  //obdCanInit();
};
//end setup

void loop() {

  button_1_State = digitalRead(button_1_Pin);
  if (button_1_State != lastbutton_1_State) {
    lastbutton_1_State = button_1_State;
    if (button_1_State == LOW) {
      Serial.println(F("Button 1 click"));
      if (screenSelected == 1) {
        screenSelected = 2;
        lv_scr_load(ui_Screen2);
      } else if (screenSelected == 2) {
        screenSelected = 3;
        lv_scr_load(ui_Screen3);
      } else if (screenSelected == 3) {
        screenSelected = 1;
        lv_scr_load(ui_Screen1);
      }
    }
  } //end button 1 block


  button_2_State = digitalRead(button_2_Pin);
  if (button_2_State != lastbutton_2_State) {
    lastbutton_2_State = button_2_State;
    if (button_2_State == LOW) {
      Serial.println(F("Button 2 click"));

      // Serial.println(shiftLightState);
      if (obdOilTemp == 135) {obdOilTemp = 98;};
      obdOilTemp = obdOilTemp +1;
      if (obdCoolantTemp == 135) {obdCoolantTemp= 105;};
      obdCoolantTemp = obdCoolantTemp +1;
      obdAirTemp = obdAirTemp +1;
      obdBoostBars = obdBoostBars +0.1;
      if (obdBoostBars == 1.3) {
        obdBoostBars = 0;
      };
      displayedBoost = displayedBoost + 100;
      if (displayedBoost == 1300) {
        displayedBoost = 0;
      };
      obdRPM = obdRPM + 80;
      // bench test end

    }
  } //end button 1 block
 

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  accelY = (a.acceleration.y+0.48)/9.8;
  accelZ = (a.acceleration.z-0.94)/9.8;
  
  if (screenSelected == 1) {
    tftPrintCoolantTemp();
    tftPrintOilTemp();
    tftPrintAirTemp();
    tftPrintBoost();
    tftPrintRPM();
  } else if (screenSelected == 2) {
    tftPrintAccel();
    delay(100);
  } else if (screenSelected == 3) {
    tftPrintGear();
  };
  

  //shiftlight
  if (obdRPM < 2000) {
    shiftLightState = 0;
  } else if (7000 <= obdRPM && obdRPM < 7500) {
    shiftLightState = 1;
  } else if (7500 <= obdRPM && obdRPM < 8000) {
    shiftLightState = 2;
  } else if (8000 <= obdRPM && obdRPM < 9000) {
    shiftLightState = 3;
  };

  if (shiftLightState == 1) {
    digitalWrite(redLedPin, LOW);
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(greenLedPin, HIGH);
  } else if (shiftLightState == 2) {
    digitalWrite(redLedPin, LOW);
    digitalWrite(yellowLedPin, HIGH);
    digitalWrite(greenLedPin, HIGH);
  } else if (shiftLightState == 3) {
    digitalWrite(redLedPin, HIGH);
    digitalWrite(yellowLedPin, HIGH);
    digitalWrite(greenLedPin, HIGH);
  } else {
    digitalWrite(redLedPin, LOW);
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(greenLedPin, LOW);
  } //shiftlight-end
  
  


  lv_timer_handler(); /* let the GUI do its work */
  
  // delay(300);
  
   //Serial.println("Loop finished");

  
};
//end loop



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
      if (buzzerStatus == 0) {buzzerStatus = 1;};
      // if (buzzerStatus == 2) {buzzerStatus = 1;};
      if (buzzerStatus == 3) {buzzerStatus = 0;};
      lv_obj_add_state(ui_coolantTemp, LV_STATE_USER_2);
      lv_obj_add_state(ui_coolantLabel, LV_STATE_USER_2);
      lv_obj_add_state(ui_coolantRed, LV_STATE_USER_2);
      lv_obj_clear_state(ui_coolantTemp, LV_STATE_USER_3);
      lv_obj_clear_state(ui_coolantLabel, LV_STATE_USER_3);
      lv_obj_clear_state(ui_coolantRed, LV_STATE_USER_3);
    } else if (130 <= obdCoolantTemp) {
      if (buzzerStatus != 4) {buzzerStatus = 3;};
      lv_obj_add_state(ui_coolantTemp, LV_STATE_USER_3);
      lv_obj_add_state(ui_coolantLabel, LV_STATE_USER_3);
      lv_obj_add_state(ui_coolantRed, LV_STATE_USER_3);
    };
    lv_label_set_text_fmt(ui_coolantTemp, "%d °C", obdCoolantTemp);
}

void tftPrintOilTemp () {
    if (obdOilTemp < 110) {
      if (buzzerStatus == 4) {buzzerStatus = 0;};
      lv_obj_clear_state(ui_oilTemp, LV_STATE_USER_2);
      lv_obj_clear_state(ui_oilLabel, LV_STATE_USER_2);
      lv_obj_clear_state(ui_oilRed, LV_STATE_USER_2);
      lv_obj_clear_state(ui_oilTemp, LV_STATE_USER_3);
      lv_obj_clear_state(ui_oilLabel, LV_STATE_USER_3);
      lv_obj_clear_state(ui_oilRed, LV_STATE_USER_3);
    } else if (110 <= obdOilTemp && obdOilTemp  < 120) {
      if (buzzerStatus == 0) {buzzerStatus = 2;};
      if (buzzerStatus == 1) {buzzerStatus = 2;};
      if (buzzerStatus == 4) {buzzerStatus = 0;};
      lv_obj_add_state(ui_oilTemp, LV_STATE_USER_2);
      lv_obj_add_state(ui_oilLabel, LV_STATE_USER_2);
      lv_obj_add_state(ui_oilRed, LV_STATE_USER_2);
      lv_obj_clear_state(ui_oilTemp, LV_STATE_USER_3);
      lv_obj_clear_state(ui_oilLabel, LV_STATE_USER_3);
      lv_obj_clear_state(ui_oilRed, LV_STATE_USER_3);
    } else if (120 <= obdOilTemp) {
      buzzerStatus = 4;
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
  lv_label_set_text_fmt(ui_boost, "%1.2f", obdBoostBars);
  // Serial.println(obdBoostBars);
  lv_bar_set_value(ui_boostBar, displayedBoost, LV_ANIM_OFF);
}

void tftPrintRPM () {
  lv_label_set_text_fmt(ui_rpm, "%d", obdRPM);
  //lv_arc_set_value(ui_rpmArc, obdRPM);
};

void tftPrintAccel () {
  lv_label_set_text_fmt(ui_accelX, "%1.1f G", accelY);
  lv_label_set_text_fmt(ui_accelY, "%1.1f G", accelZ);
  lv_obj_set_x(ui_accelPointer, (accelY*35)+50);
  lv_obj_set_y(ui_accelPointer, (accelZ*35)+0);
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
  // Serial.print(obdRPM);
  // Serial.print(" / ");
  // Serial.print(obdSpeed);
  // Serial.print(" / ");
  // if (obdSpeed != 0) {
  //   Serial.print(obdRPM/obdSpeed);
  // } else {
  //   Serial.print("zero");
  // };
  
  if (obdSpeed > 0) {
    if (100 < (obdRPM/obdSpeed) && (obdRPM/obdSpeed) < 140) {
      lv_label_set_text( ui_gearDisp, "1");
    } else if (65 < (obdRPM/obdSpeed) && (obdRPM/obdSpeed) < 85) {
      lv_label_set_text( ui_gearDisp, "2");
    } else if (53 < (obdRPM/obdSpeed) && (obdRPM/obdSpeed) < 59) {
      lv_label_set_text( ui_gearDisp, "3");
    } else if (44 < (obdRPM/obdSpeed) && (obdRPM/obdSpeed) < 48) {
      lv_label_set_text( ui_gearDisp, "4");
    } else if (37 < (obdRPM/obdSpeed) && (obdRPM/obdSpeed) < 41) {
      lv_label_set_text( ui_gearDisp, "5");
    } else if (32 < (obdRPM/obdSpeed) && (obdRPM/obdSpeed) < 35) {
      lv_label_set_text( ui_gearDisp, "6");
    } else {
      lv_label_set_text( ui_gearDisp, "N");
      //Serial.println("else");
    };
  }
  

}



//Вынести считывание обд в отдельную таску
//Протестировать ускоренное считывание ОБД
//Починить запуск без консоли
//на экране РПМ сделать анимацию арка
//Буст - протестировать выдачу данных с обд

//Вывод данных в Рейсхроно по BLE

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
#define CAN_RX_PIN  4
#define CAN_TX_PIN  5
//work fine with rx (orange) = 4, tx (red) = 5
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
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

const int button_1_Pin = 12;
const int button_2_Pin = 13;
const int redLedPin = 9;
const int yellowLedPin = 11;
const int greenLedPin = 10;

int button_1_State = 0;
int button_2_State = 0; 
int shiftLightState = 0; 
byte lastbutton_1_State = HIGH;
byte lastbutton_2_State = HIGH;

int screenSelected = 1;

int obdRPM = 800;
int obdOilTemp = 82;
int obdCoolantTemp = 95; 
int obdAirTemp = 35;
int obdBoost = 200; //kPa
double obdBoostBars = 0.2; //to Bars
int displayedBoost = 200; //from 0 to 1500

int obdRPM_tmp = 800;
int obdOilTemp_tmp = 82;
int obdCoolantTemp_tmp = 95; 
int obdAirTemp_tmp = 35;
int obdBoost_tmp = 200;

int obdOilTemp_prev = 82;
int obdCoolantTemp_prev = 95; 
int obdAirTemp_prev = 35;
int obdRPM_prev = 400;
int obdBoost_prev = 0.1;
int obdBoostBars_prev = 0.1;
int displayedBoost_prev = 700;

int obdParameter = 1;

String textWrapper = "String";

void tftPrintCoolantTemp ();
void tftPrintOilTemp ();
void tftPrintAirTemp ();
void tftPrintRPM ();
void tftPrintBoost();
void processPid(int pid);


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
  Serial.println(obdParameter);
  if (obdParameter == 1) {
    obdRPM_tmp = OBD2.pidRead(ENGINE_RPM);
    if (obdRPM_tmp < 10000) {obdRPM = obdRPM_tmp;};
  } else if (obdParameter == 2) {
    obdBoost_tmp = OBD2.pidRead(INTAKE_MANIFOLD_ABSOLUTE_PRESSURE);
    if (obdBoost_tmp < 500) {obdBoost = obdBoost_tmp;};
    obdBoostBars = (obdBoost/100)-1;
    displayedBoost= obdBoostBars*100;
    Serial.print("obdBoost_tmp:");
    Serial.println(obdBoost_tmp);
    Serial.print("obdBoostBars:");
    Serial.println(obdBoostBars);
    Serial.print("displayedBoost:");
    Serial.println(displayedBoost);
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




void setup() {
  Serial.begin(115200);
  while (!Serial);
  strip.begin(); 
  strip.setBrightness(10);
  strip.setPixelColor(0, 0, 255, 0);
  strip.show();

  pinMode(redLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(button_1_Pin, INPUT_PULLUP);
  pinMode(button_2_Pin, INPUT_PULLUP);

  Serial.println(F("OBD2 data printer"));

  tft.init();
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
  lv_disp_drv_register( &disp_drv );
  lv_obj_t *label = lv_label_create( lv_scr_act() );
  lv_label_set_text( label, "Hello Ardino and LVGL!");
  lv_obj_align( label, LV_ALIGN_CENTER, 0, 0 );
  //lvgl example end

  ui_init();

  }
  //obdCanInit();
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
      obdOilTemp = obdOilTemp +1;
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


  //obdRead(); //read one OBD PID  

  
  if (screenSelected == 1) {
    tftPrintCoolantTemp();
    tftPrintOilTemp();
    tftPrintAirTemp();
  } else if (screenSelected == 2) {
    tftPrintBoost();
  } else if (screenSelected == 3) {
    tftPrintRPM();
  };
  

  //shiftlight
  if (obdRPM < 2000) {
    shiftLightState = 0;
  } else if (2000 <= obdRPM && obdRPM < 2500) {
    shiftLightState = 1;
  } else if (2500 <= obdRPM && obdRPM < 3000) {
    shiftLightState = 2;
  } else if (3000 <= obdRPM && obdRPM < 5000) {
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
  
  // Serial.println("Loop finished");

  
}
//end loop



void tftPrintCoolantTemp () {
    if (obdCoolantTemp < 100) {
      lv_obj_clear_state(ui_coolantTemp, LV_STATE_USER_2);
      lv_obj_clear_state(ui_coolantLabel, LV_STATE_USER_2);
      lv_obj_clear_state(ui_coolantRed, LV_STATE_USER_2);
      lv_obj_clear_state(ui_coolantTemp, LV_STATE_USER_3);
      lv_obj_clear_state(ui_coolantLabel, LV_STATE_USER_3);
      lv_obj_clear_state(ui_coolantRed, LV_STATE_USER_3);
    } else if (100 <= obdCoolantTemp && obdCoolantTemp  < 115) {
      lv_obj_add_state(ui_coolantTemp, LV_STATE_USER_2);
      lv_obj_add_state(ui_coolantLabel, LV_STATE_USER_2);
      lv_obj_add_state(ui_coolantRed, LV_STATE_USER_2);
      lv_obj_clear_state(ui_coolantTemp, LV_STATE_USER_3);
      lv_obj_clear_state(ui_coolantLabel, LV_STATE_USER_3);
      lv_obj_clear_state(ui_coolantRed, LV_STATE_USER_3);
    } else if (115 <= obdCoolantTemp) {
      lv_obj_add_state(ui_coolantTemp, LV_STATE_USER_3);
      lv_obj_add_state(ui_coolantLabel, LV_STATE_USER_3);
      lv_obj_add_state(ui_coolantRed, LV_STATE_USER_3);
    };
    lv_label_set_text_fmt(ui_coolantTemp, "%d °C", obdCoolantTemp);
}

void tftPrintOilTemp () {
    if (obdOilTemp < 100) {
      lv_obj_clear_state(ui_oilTemp, LV_STATE_USER_2);
      lv_obj_clear_state(ui_oilLabel, LV_STATE_USER_2);
      lv_obj_clear_state(ui_oilRed, LV_STATE_USER_2);
      lv_obj_clear_state(ui_oilTemp, LV_STATE_USER_3);
      lv_obj_clear_state(ui_oilLabel, LV_STATE_USER_3);
      lv_obj_clear_state(ui_oilRed, LV_STATE_USER_3);
    } else if (100 <= obdOilTemp && obdOilTemp  < 110) {
      lv_obj_add_state(ui_oilTemp, LV_STATE_USER_2);
      lv_obj_add_state(ui_oilLabel, LV_STATE_USER_2);
      lv_obj_add_state(ui_oilRed, LV_STATE_USER_2);
      lv_obj_clear_state(ui_oilTemp, LV_STATE_USER_3);
      lv_obj_clear_state(ui_oilLabel, LV_STATE_USER_3);
      lv_obj_clear_state(ui_oilRed, LV_STATE_USER_3);
    } else if (110 <= obdOilTemp) {
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
  lv_arc_set_value(ui_rpmArc, obdRPM);
};





//Сделать частое обновление экрана и отдельно медленную функцию ОБД. Через 2 ядра и мультитаск.
//на экране РПМ сделать анимацию арка
//Буст - протестировать выдачу данных с обд
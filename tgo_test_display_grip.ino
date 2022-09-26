

#include <pgmspace.h>
#include "BluetoothSerial.h"
#include <TFT_eSPI.h>
#include <SPI.h>
//#include "esp_adc_cal.h"
#include <driver/adc.h>
#include <TJpg_Decoder.h>
#include <esp_wifi.h>
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

#include "battery1_5.h"
#include "battery2_5.h"
#include "battery3_5.h"
#include "battery4_5.h"
#include "battery5_5.h"
#include "bluetooth1.h"
#include "logo48.h"
float voltsArray[100]= {
  3.250,  3.300,  3.350,  3.400,  3.450,
  3.500,  3.550,  3.600,  3.650,  3.700,
  3.703,  3.706,  3.710,  3.713,  3.716,
  3.719,  3.723,  3.726,  3.729,  3.732,
  3.735,  3.739,  3.742,  3.745,  3.748,
  3.752,  3.755,  3.758,  3.761,  3.765,
  3.768,  3.771,  3.774,  3.777,  3.781,
  3.784,  3.787,  3.790,  3.794,  3.797,
  3.800,  3.805,  3.811,  3.816,  3.821,
  3.826,  3.832,  3.837,  3.842,  3.847,
  3.853,  3.858,  3.863,  3.868,  3.874,
  3.879,  3.884,  3.889,  3.895,  3.900,
  3.906,  3.911,  3.917,  3.922,  3.928,
  3.933,  3.939,  3.944,  3.950,  3.956,
  3.961,  3.967,  3.972,  3.978,  3.983,
  3.989,  3.994,  4.000,  4.008,  4.015,
  4.023,  4.031,  4.038,  4.046,  4.054,
  4.062,  4.069,  4.077,  4.085,  4.092,
  4.100,  4.111,  4.122,  4.133,  4.144,
  4.156,  4.167,  4.178,  4.189,  4.20 };
int currentBatteryState = 5;
int lastBatteryState = 0;

#define battIndicatorWidth 70
#define battIndicatorHeight 36
#define battIndicatorPosX (tft.width() - battIndicatorWidth)
#define bluetoothIndicatorWidth 41
#define bluetoothIndicatorHeight 70
#define bluetoothIndicatorPosX (tft.width() - bluetoothIndicatorWidth)
#define headerHeight battIndicatorHeight

bool toLightSleep = false;
bool toDeepSleep = false;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

int screenDelay = 100;

BluetoothSerial SerialBT;
TFT_eSPI tft = TFT_eSPI(135, 240); //res
#define tft_backlightPin  4

//device buttons
#define leftBtn      0
#define rightBtn     35
//macroni buttons
#define indexBtn     15
#define middleBtn    2
#define pinkyBtn     13
#define ringBtn      12
//modes
int buttonMode = 1;

#define rLed      25
#define gLed      33
#define bLed      32
#define rChan     0
#define gChan     1
#define bChan     2
#define vChan     3

int buttonDelay = 250;

long batteryLife = 1500/6; //battery capacity / energy consumption
long previousMillis = 0;

float oldBatteryPercentage = 100;
int oldMacroMode = 0;
bool oldConnected = true; //false. not the real value. just refreshes the screen on a fresh boot

bool deviceConnected = false;
int macroMode = 0; //0,1,2,3,4
int ann = 0;
bool requestColors = true;
uint16_t modeColors[5] = {TFT_WHITE, TFT_RED, TFT_RED, TFT_RED, TFT_RED};
                    //no mode, mode 1 - index, mode 2 - middle, mode 3 - ring, mode 4 - pinky
                    //these update themselves later from the app
int ledValues[3] = { 0,0,0 };
uint16_t lastColor = 0;
const uint16_t colorPalette[] = {
  TFT_BLACK,
  TFT_NAVY,
  TFT_DARKGREEN,
  TFT_DARKCYAN,
  TFT_MAROON,
  TFT_PURPLE,
  TFT_OLIVE,
  TFT_LIGHTGREY,
  TFT_DARKGREY,
  TFT_BLUE,
  TFT_GREEN,
  TFT_CYAN,
  TFT_RED,
  TFT_MAGENTA,
  TFT_YELLOW,
  TFT_WHITE,
  TFT_ORANGE,
  TFT_GREENYELLOW,
  TFT_PINK,
  TFT_BROWN,
  TFT_GOLD,
  TFT_SILVER,
  TFT_SKYBLUE,
  TFT_VIOLET
};

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT){
    Serial.println("Client Connected");
    deviceConnected = true;
  }
  if(event == ESP_SPP_CLOSE_EVT ){
    Serial.println("Client disconnected");
    deviceConnected = false;
    requestColors = true;
    modeColors[1] = TFT_RED;
    modeColors[2] = TFT_RED;
    modeColors[3] = TFT_RED;
    modeColors[4] = TFT_RED;
    modeColors[5] = TFT_RED;
  }
}

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
  if ( y >= tft.height() ) return 0;
  //tft.fillRect(x, y, w, h,TFT_BLACK);
  tft.pushImage(x, y, w, h, bitmap);
  return 1;
}


void setup() {
  Serial.begin(115200);
  /*
    ADC detection enable port
    If the USB port is used for power supply, it is turned on by default.
    If it is powered by battery, it needs to be set to high level
    */
  pinMode(14, OUTPUT);

  pinMode(leftBtn, INPUT_PULLUP);
  pinMode(rightBtn, INPUT_PULLUP);
  pinMode(indexBtn, INPUT_PULLUP);
  pinMode(middleBtn, INPUT_PULLUP);
  pinMode(ringBtn, INPUT_PULLUP);
  pinMode(pinkyBtn, INPUT_PULLUP);

  ledcSetup(0, 10000, 8);
  ledcAttachPin(rLed, rChan);
  ledcSetup(1, 10000, 8);
  ledcAttachPin(gLed, gChan);
  ledcSetup(2, 10000, 8);
  ledcAttachPin(bLed, bChan);

  adc2_config_channel_atten( ADC2_CHANNEL_7, ADC_ATTEN_11db );
  //adc1_config_width(ADC_WIDTH_BIT_12);
  //adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_11db);

  tft.init();
  tft.setRotation(3);
  tft.setCursor(0, 0);
  tft.setSwapBytes(true);
  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(true);
  TJpgDec.setCallback(tft_output);

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(3);
  tft.setTextDatum(MC_DATUM);
  fadeLed(TFT_PURPLE, 1);
  drawRadar();
  tft.fillScreen(TFT_BLACK);
  delay(100);
  SerialBT.begin("Macroni"); //Bluetooth device name
  SerialBT.register_callback(callback);
  xTaskCreate(batteryInfo, "batteryInfo", 2048, NULL, 1, NULL);
  xTaskCreate(buttonChecker, "buttonChecker", 2048, NULL, 1, NULL);
}

void espDelay(int ms)
{
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_light_sleep_start();
}

void fadeLed(uint16_t color, int msRate) {
  if ( color == lastColor ) {
    return;
  }
  int oldLedValues[3] = {ledValues[0], ledValues[1], ledValues[2]};
  if ( color == TFT_BLACK ) {
    ledValues[0] = 0;
    ledValues[1] = 0;
    ledValues[2] = 0;
    lastColor = color;  
  } else if ( color == TFT_NAVY ) {
    ledValues[0] = 0;
    ledValues[1] = 0;
    ledValues[2] = 128;  
    lastColor = color;
  } else if ( color == TFT_DARKGREEN ) {
    ledValues[0] = 0;
    ledValues[1] = 128;
    ledValues[2] = 0;  
    lastColor = color;
  } else if ( color == TFT_DARKCYAN ) {
    ledValues[0] = 0;
    ledValues[1] = 128;
    ledValues[2] = 128;  
    lastColor = color;
  }  else if ( color == TFT_MAROON ) {
    ledValues[0] = 128;
    ledValues[1] = 0;
    ledValues[2] = 0;  
    lastColor = color;
  } else if ( color == TFT_PURPLE ) {
    ledValues[0] = 128;
    ledValues[1] = 0;
    ledValues[2] = 128;  
    lastColor = color;
  } else if ( color == TFT_OLIVE ) {
    ledValues[0] = 128;
    ledValues[1] = 128;
    ledValues[2] = 0;  
    lastColor = color;
  } else if ( color == TFT_LIGHTGREY ) {
    ledValues[0] = 211;
    ledValues[1] = 211;
    ledValues[2] = 211;  
    lastColor = color;
  } else if ( color == TFT_DARKGREY ) {
    ledValues[0] = 128;
    ledValues[1] = 128;
    ledValues[2] = 128;  
    lastColor = color;
  } else if ( color == TFT_BLUE ) {
    ledValues[0] = 0;
    ledValues[1] = 0;
    ledValues[2] = 255;  
    lastColor = color;
  } else if ( color == TFT_GREEN ) {
    ledValues[0] = 0;
    ledValues[1] = 255;
    ledValues[2] = 0;  
    lastColor = color;
  } else if ( color == TFT_CYAN ) {
    ledValues[0] = 0;
    ledValues[1] = 255;
    ledValues[2] = 255;  
    lastColor = color;
  } else if ( color == TFT_RED ) {
    ledValues[0] = 255;
    ledValues[1] = 0;
    ledValues[2] = 0;  
    lastColor = color;
  } else if ( color == TFT_MAGENTA ) {
    ledValues[0] = 255;
    ledValues[1] = 0;
    ledValues[2] = 255;  
    lastColor = color;
  } else if ( color == TFT_YELLOW ) {
    ledValues[0] = 255;
    ledValues[1] = 255;
    ledValues[2] = 0;  
    lastColor = color;
  } else if ( color == TFT_WHITE ) {
    ledValues[0] = 255;
    ledValues[1] = 255;
    ledValues[2] = 255;  
    lastColor = color;
  } else if ( color == TFT_ORANGE ) {
    ledValues[0] = 255;
    ledValues[1] = 180;
    ledValues[2] = 0;  
    lastColor = color;
  } else if ( color == TFT_GREENYELLOW ) {
    ledValues[0] = 180;
    ledValues[1] = 255;
    ledValues[2] = 0;  
    lastColor = color;
  } else if ( color == TFT_PINK ) {
    ledValues[0] = 255;
    ledValues[1] = 192;
    ledValues[2] = 203;  
    lastColor = color;
  } else if ( color == TFT_BROWN ) {
    ledValues[0] = 150;
    ledValues[1] = 75;
    ledValues[2] = 255;  
    lastColor = color;
  } else if ( color == TFT_GOLD ) {
    ledValues[0] = 255;
    ledValues[1] = 215;
    ledValues[2] = 0;  
    lastColor = color;
  } else if ( color == TFT_SILVER ) {
    ledValues[0] = 192;
    ledValues[1] = 192;
    ledValues[2] = 192;  
    lastColor = color;
  } else if ( color == TFT_SKYBLUE ) {
    ledValues[0] = 135;
    ledValues[1] = 206;
    ledValues[2] = 235;  
    lastColor = color;
  } else if ( color == TFT_VIOLET ) {
    ledValues[0] = 180;
    ledValues[1] = 46;
    ledValues[2] = 226;  
    lastColor = color;
  }
  //r
  while( oldLedValues[0] > ledValues[0] ) {
    ledcWrite(rChan, oldLedValues[0]);
    oldLedValues[0] = oldLedValues[0] - 1;
    delay(msRate);
  }
  while( oldLedValues[0] < ledValues[0] ) {
    ledcWrite(rChan, oldLedValues[0]);
    oldLedValues[0] = oldLedValues[0] + 1;
    delay(msRate);
  }
  //g
  while( oldLedValues[1] > ledValues[1] ) {
    ledcWrite(gChan, oldLedValues[1]);
    oldLedValues[1] = oldLedValues[1] - 1;
    delay(msRate);
  }
  while( oldLedValues[1] < ledValues[1] ) {
    ledcWrite(gChan, oldLedValues[1]);
    oldLedValues[1] = oldLedValues[1] + 1;
    delay(msRate);
  }
  //b
  while( oldLedValues[2] > ledValues[2] ) {
    ledcWrite(bChan, oldLedValues[2]);
    oldLedValues[2] = oldLedValues[2] - 1;
    delay(msRate);
  }
  while( oldLedValues[2] < ledValues[2] ) {
    ledcWrite(bChan, oldLedValues[2]);
    oldLedValues[2] = oldLedValues[2] + 1;
    delay(msRate);
  }
}

void drawRadar(){
  int bootColors[6] = {TFT_YELLOW, TFT_MAGENTA, TFT_ORANGE, TFT_BLUE, TFT_CYAN};
  tft.pushImage(tft.width() / 2 - (48/2), tft.height() / 2 - (63/2), 48, 63, logo48);
  for (int y=0; y<3; y++) {
    for (int x=6; x<16; x++) {
      tft.drawCircle( tft.width() / 2, tft.height() / 2, x*x+10, bootColors[ random( 0, sizeof(bootColors)-1 ) ] );
      delay(50);
    }
    //tft.drawString("CourierKraft", tft.width() / 2, tft.height() / 2);      
    delay(500);
    //
    for (int x=6; x<16; x++) {
      tft.drawCircle(tft.width() / 2, tft.height() / 2, x*x+10, TFT_BLACK);
      delay(50);
    }    
  }
}

void sleepyByTime() {
  screenBacklight(false);
  //
  //esp_bluedroid_disable();
  //esp_bt_controller_disable();
  //  
  espDelay(6000);
  tft.writecommand(TFT_DISPOFF);
  tft.writecommand(TFT_SLPIN);
  //After using light sleep, you need to disable timer wake, because here use external IO port to wake up
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0); 
  delay(200);
  esp_deep_sleep_start();
}

String clickType(int btn) {
  // return true if button param is greater than 1 second
  // return false for short press
  // return null for neither long nor short
  float timePassed = 0;
  while(!digitalRead(btn)) {
    timePassed += 0.10;
    if ( timePassed > 30 ) {
      break;
    } else {
      delay(10);
    }
  }
  if ( timePassed > 0.2 && timePassed < 2.69 ) {
    Serial.println("short press");
    return "short";
  } else if ( timePassed > 2.69 && timePassed < 30 ) {
    Serial.println("long press");
    return "long";
  } else {
    Serial.println("press error");
    return "none";
  }
}

bool buttonCheckMomentary(){
  bool actionTaken = false;
    /// --- macroni buttons
    //
    if (!digitalRead(indexBtn)) {
      String clickAction = "";
      clickAction = clickType(indexBtn);
      if ( clickAction == "long") {
        Serial.println("long click - 1");
        if ( macroMode != 1 ) {
          Serial.println("mode: 1");
          macroMode = 1;
          actionTaken = true;
          delay(buttonDelay);
        } else {
          macroMode = 0;
          actionTaken = true;
          delay(buttonDelay);
        } 
      } else if ( clickAction == "short") {
        Serial.println("short click - 1");
        switch ( macroMode ) {
            case 0:
              SerialBT.write( 1 );
              delay(buttonDelay);
              actionTaken = true;
              break;
            case 1:
              //mode button
              delay(buttonDelay);
              actionTaken = true;
              break;
            case 2:
              SerialBT.write( 8 );
              delay(buttonDelay);
              actionTaken = true;
              break;
            case 3:
              SerialBT.write( 11 );
              delay(buttonDelay);
              actionTaken = true;
              break;
            case 4:
              SerialBT.write( 14 );
              delay(buttonDelay);
              actionTaken = true;
              break;
        }
      }
    } else if (!digitalRead(middleBtn)) {
      String clickAction = "";
      clickAction = clickType(middleBtn);
      if (clickAction == "long") {
        Serial.println("long click - 2");
        if ( macroMode != 2 ) {
          Serial.println("mode: 2");
          macroMode = 2;
          actionTaken = true;
          delay(buttonDelay);        
        } else {
          Serial.println("mode: 0");
          macroMode = 0;
          actionTaken = true;
          delay(buttonDelay);
        }
      } else if (clickAction == "short"){
        Serial.println("short click - 2");
        switch ( macroMode ) {
            case 0:
              SerialBT.write( 2 );
              actionTaken = true;
              delay(buttonDelay);
              break;
            case 1:
              SerialBT.write( 5 );
              actionTaken = true;
              delay(buttonDelay);
              break;
            case 2:
              //mode button
              actionTaken = true;
              delay(buttonDelay);
              break;
            case 3:
              SerialBT.write( 12 );
              actionTaken = true;
              delay(buttonDelay);
              break;
            case 4:
              SerialBT.write( 15 );
              actionTaken = true;
              delay(buttonDelay);
              break;
          }
      }
    } else if (!digitalRead(ringBtn)) {
      String clickAction = "";
      clickAction = clickType(ringBtn);
      if (clickAction == "long") {
        Serial.println("long click - 3");
        if ( macroMode != 3 ) {
          Serial.println("mode: 3");
          macroMode = 3;
          actionTaken = true;
          delay(buttonDelay);
        } else {
          Serial.println("mode: 0");
          macroMode = 0;
          actionTaken = true;
          delay(buttonDelay);
        }
      } else if (clickAction == "short") {
        Serial.println("short click - 3");
        switch ( macroMode ) {
            case 0:
              SerialBT.write( 3 );
              actionTaken = true;
              delay(buttonDelay);
              break;
            case 1:
              SerialBT.write( 6 );
              actionTaken = true;
              delay(buttonDelay);
              break;
            case 2:
              SerialBT.write( 9 );
              actionTaken = true;
              delay(buttonDelay);
              break;
            case 3:
              //mode button
              delay(buttonDelay);
              break;
            case 4:
              SerialBT.write( 16 );
              actionTaken = true;
              delay(buttonDelay);
              break;
          }
      }
    } else if (!digitalRead(pinkyBtn)) {
      String clickAction = "";
      clickAction = clickType(pinkyBtn);
      if (clickAction == "long") {
        Serial.println("long click - 4");
        if ( macroMode != 4 ) {
          Serial.println("mode: 4");
          macroMode = 4;
          actionTaken = true;
          delay(buttonDelay);
        } else {
          Serial.println("mode: 0");
          macroMode = 0;
          actionTaken = true;
          delay(buttonDelay);
        }
      } else if (clickAction == "short") {
        Serial.println("short click - 4");
        switch ( macroMode ) {
            case 0:
              SerialBT.write( 4 );
              actionTaken = true;
              delay(buttonDelay);
              break;
            case 1:              
              SerialBT.write( 7 );
              actionTaken = true;
              delay(buttonDelay);
              break;
            case 2:
              SerialBT.write( 10 );
              actionTaken = true;
              delay(buttonDelay);
              break;
            case 3:
              SerialBT.write( 13 );
              actionTaken = true;
              delay(buttonDelay);
              break;
            case 4:
              //mode button
              actionTaken = true;
              delay(buttonDelay);
              break;
          }
      }
    }  
}

bool buttonCheckTemporary(){
  bool actionTaken = false;
  float duration = 0;
  while(!digitalRead( leftBtn )) { //held
    actionTaken = true;
    if ( duration > 2.69 && duration < 69.69) {
      toDeepSleep = true;
      delay(6000);
    } else {
      duration += 0.10;
      delay(10); 
    }    
  }
  duration = 0;
  while(!digitalRead( rightBtn )) { //held
    actionTaken = true;
    if ( duration > 2.69 && duration < 69.69) {
      delay(3000);
     } else {
      duration += 0.10;
      delay(10); 
    }    
  }
  if ( ! deviceConnected ) {
    return actionTaken;
  }
  duration = 0;
  while (!digitalRead( indexBtn )) { //held
    actionTaken = true;
    if ( duration > 2.69 && duration < 69.69) {
      //mode 1 activated. check buttons
      if ( macroMode != 1 ) {
        macroMode = 1;
      }
      //
      if (!digitalRead( middleBtn )) {
        SerialBT.write( 5 );
        delay(buttonDelay);
      } else if (!digitalRead( ringBtn )) {
        SerialBT.write( 6 );
        delay(buttonDelay);
      } else if (!digitalRead( pinkyBtn )) {
        SerialBT.write( 7 );
        delay(buttonDelay);
      }
    } else {
      if ( macroMode != 0 ) {
        macroMode = 0;
      }
    }
    duration += 0.10;
    delay(10);
  }
  //
  if ( duration > 0 && duration < 2.69 ) { //short press index
    macroMode = 0;
    SerialBT.write( 1 );
    delay(buttonDelay); 
  }
  //
  duration = 0;
  while (!digitalRead( middleBtn )) { //held
    actionTaken = true;
    if ( duration > 2.69 && duration < 69.69) {
      //mode 1 activated. check buttons
      if ( macroMode != 2 ) {
        macroMode = 2;
      }
      //
      if (!digitalRead( indexBtn )) {
        SerialBT.write( 8 );
        delay(buttonDelay);
      } else if (!digitalRead( ringBtn )) {
        SerialBT.write( 9 );
        delay(buttonDelay);
      } else if (!digitalRead( pinkyBtn )) {
        SerialBT.write( 10 );
        delay(buttonDelay);
      }
    } else {
      if ( macroMode != 0 ) {
        macroMode = 0;
      }
    }
    duration += 0.10;
    delay(10);
  }
  //
  if ( duration > 0 && duration < 2.69 ) { //short press middle
    macroMode = 0;
    SerialBT.write( 2 );
    delay(1000);
  }
  //
  duration = 0;
  while (!digitalRead( ringBtn )) { //held
    actionTaken = true;
    if ( duration > 2.69 && duration < 69.69) {
      //mode 1 activated. check buttons
      if ( macroMode != 3 ) {
        macroMode = 3;
      }
      //
      if (!digitalRead( indexBtn )) {
        SerialBT.write( 11 );
        delay(buttonDelay);
      } else if (!digitalRead( middleBtn )) {
        SerialBT.write( 12 );
        delay(buttonDelay);
      } else if (!digitalRead( pinkyBtn )) {
        SerialBT.write( 13 );
        delay(buttonDelay);
      }
    } else {
      if ( macroMode != 0 ) {
        macroMode = 0;
      }
    }
    duration += 0.10;
    delay(10);
  }
  //
  if ( duration > 0 && duration < 2.69 ) { //short press ring
    macroMode = 0;
    SerialBT.write( 3 );
    delay(buttonDelay);
  }
  //
  duration = 0;
  while (!digitalRead( pinkyBtn )) { //held
    actionTaken = true;
    if ( duration > 2.69 && duration < 69.69) {
      //mode 1 activated. check buttons
      if ( macroMode != 4 ) {
        macroMode = 4;
      }
      //
      if (!digitalRead( indexBtn )) {
        SerialBT.write( 14 );
        delay(buttonDelay);
      } else if (!digitalRead( middleBtn )) {
        SerialBT.write( 15 );
        delay(buttonDelay);
      } else if (!digitalRead( ringBtn )) {
        SerialBT.write( 16 );
        delay(buttonDelay);
      }
    } else {
      if ( macroMode != 0 ) {
        macroMode = 0;
      }
    }
    duration += 0.10;
    delay(10);
  }
  //
  if ( duration > 0 && duration < 2.69 ) { //short press pinky
    macroMode = 0;
    SerialBT.write( 4 );
    delay(buttonDelay);
  }
  //
  //reset
  if ( macroMode != 0 ) {
    macroMode = 0;
  }
  return actionTaken;
}

void buttonStates(){
    Serial.println("---------------");
    Serial.print("#l-btn - ");
    Serial.print( digitalRead(leftBtn) ); //0
    Serial.println("");
    //
    Serial.print("#r-btn - ");
    Serial.print( digitalRead(rightBtn) );//35
    Serial.println("");
    //
    Serial.print("#1 - ");
    Serial.print( digitalRead(indexBtn) ); //15
    Serial.println("");
    //
    Serial.print("#2 - ");
    Serial.print( digitalRead(middleBtn) );//2
    Serial.println("");
    //
    Serial.print("#3 - ");
    Serial.print( digitalRead(ringBtn) );//12
    Serial.println("");
    //
    Serial.print("#4 - ");
    Serial.print( digitalRead(pinkyBtn) );//13
    Serial.println("");
}

void screenBacklight(bool state) {
  if ( state ) {
    if ( !digitalRead(tft_backlightPin) ) {
      Serial.println("screenBacklight: ON");
      digitalWrite(tft_backlightPin, HIGH); 
    }
  } else {
    if ( digitalRead(tft_backlightPin) ) {
      Serial.println("screenBacklight: OFF");
      digitalWrite(tft_backlightPin, LOW); 
    }
  }
}

int readVPin() {
  int v;
  adc2_get_raw( ADC2_CHANNEL_7, ADC_WIDTH_12Bit, &v);
  return v;
}

//void drawingBatteryIcon(const uint8_t* image){

void showVoltage() {
  int batteryPercentage = 0;
  int vAvg = 0;
  digitalWrite(14, HIGH); delay(1);
  for (int x=1; x<20; x++) {
    int v;
    esp_err_t err = adc2_get_raw( ADC2_CHANNEL_7, ADC_WIDTH_12Bit, &v);
    if ( err == ESP_OK ) { // else: ADC2 used by Wi-Fi
      vAvg += v;
    } else {
      delay(10);
    }
  }
  digitalWrite(14, LOW);
  vAvg = vAvg / 20;
  float batteryVoltage = ((float)vAvg / 4095.0) * 2.0 * 3.3 * (1100 / 1000.0);
  String voltage = String(batteryVoltage) + "v";
  if ( batteryVoltage > 4.1 ) {
    currentBatteryState = 5;
    TJpgDec.drawJpg(battIndicatorPosX, 0, battery5_5, sizeof(battery5_5));
  } else {
    for ( int x=0; x < sizeof( voltsArray )-1; x++ ){
      if ( batteryVoltage >= voltsArray[x] && batteryVoltage <= voltsArray[x+1] ) {
        batteryPercentage = x;
        if ( abs(oldBatteryPercentage - x) > 10 ) {
          //going from usb power to battery
          batteryPercentage = x;
        } else if ( abs(oldBatteryPercentage - x) > 3 ) {
          //rattle
          batteryPercentage = oldBatteryPercentage;
        }
        break;
      }
    }
    //Serial.println(batteryPercentage);
    if ( batteryPercentage > 75 ) {
      currentBatteryState = 4;
      TJpgDec.drawJpg(battIndicatorPosX, 0, battery4_5, sizeof(battery4_5));
    } else if ( batteryPercentage < 75 && batteryPercentage > 50 ) {
      currentBatteryState = 3;
      TJpgDec.drawJpg(battIndicatorPosX, 0, battery3_5, sizeof(battery3_5));
    } else if ( batteryPercentage < 50 && batteryPercentage > 25 ) {
      currentBatteryState = 2;
      TJpgDec.drawJpg(battIndicatorPosX, 0, battery2_5, sizeof(battery2_5));
    } else if ( batteryPercentage < 25 ) {
      currentBatteryState = 1;
      TJpgDec.drawJpg(battIndicatorPosX, 0, battery1_5, sizeof(battery1_5));
      currentBatteryState = 0;
      if ( lastBatteryState != currentBatteryState ) {
        ledcWrite(rChan, 0); ledcWrite(gChan, 0); ledcWrite(bChan, 0);
        for ( int x=0; x<3; x++ ) {
          ledcWrite(rChan, 255); delay(50); ledcWrite(rChan, 0); delay(10);
        }
        lastBatteryState = currentBatteryState;
      } else if ( batteryPercentage < 5 ) {
        currentBatteryState = 0;
        tft.setTextSize(2);
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("Battery Proctection!",  tft.width() / 2, tft.height() / 2 );
        fadeLed(TFT_BLACK, 1);
        delay(5000);
        toDeepSleep = true;
        for (int x=30; x>0; x-- ) {
          if ( ! readVPin() > 2000 ) {
            tft.fillScreen(TFT_BLACK);
            tft.drawString("Sleeping in: " + String(x),  tft.width() / 2, tft.height() / 2 );
            delay(1000);
            tft.fillScreen(TFT_BLACK);
            tft.drawString("Find a charger!" + String(batteryPercentage) + "%",  tft.width() / 2, tft.height() / 2 );
            delay(1000);            
          } else {
            toDeepSleep = false;
            break;
          }
        }
      }
    }
    //
  }
  if ( abs(oldBatteryPercentage - batteryPercentage) > 1 || oldMacroMode != macroMode || oldConnected != deviceConnected ) {
    oldBatteryPercentage = batteryPercentage;
    oldMacroMode = macroMode;
    oldConnected = deviceConnected;
    //
    if ( macroMode == 0 ) {
      tft.setTextColor(TFT_WHITE);
    } else {
      tft.setTextColor(modeColors[ macroMode ]);
    }
    tft.setTextSize(2);
    tft.fillRect(0, 0, battIndicatorPosX, headerHeight,TFT_BLACK);
    if ( currentBatteryState < 5 ) {
      tft.setTextDatum(5);
      tft.drawString(String(batteryPercentage)+"%", battIndicatorPosX-2, battIndicatorHeight/2, 1);    
    } else {
      tft.setTextDatum(5);
      tft.drawString("USB", battIndicatorPosX-2, battIndicatorHeight/2, 1);
    }
    tft.fillRect(0, headerHeight, tft.width(), tft.height(),TFT_BLACK);
    tft.setCursor(4, 4);
    tft.print("Macroni");
    tft.setCursor (4, headerHeight+4);
    tft.print("Mode:");
    tft.println(macroMode);
    tft.println("");
    tft.print("Volts:");
    tft.println(voltage);  
    tft.println("");
    if (deviceConnected) {
      tft.setTextColor(TFT_CYAN);
      tft.println("Connected");
      TJpgDec.drawJpg(bluetoothIndicatorPosX-16, headerHeight+16, bluetooth1, sizeof(bluetooth1));
    } else {
      tft.setTextColor(TFT_ORANGE);
      tft.println("Disconnected");
      tft.fillRect(bluetoothIndicatorPosX-16, headerHeight+16, tft.width(), tft.height(),TFT_BLACK);
      tft.pushImage(bluetoothIndicatorPosX - 24, headerHeight + 16, 48, 63, logo48);
    } 
  }
}

void showMessage(String msg, int d) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(msg,  tft.width() / 2, tft.height() / 2 );  
  delay(d);
  tft.fillScreen(TFT_BLACK);
}

void batteryInfo(void *arg) {
  while ( 1 ) {
    while( ! toDeepSleep ) {
      if (deviceConnected) {
        if ( requestColors ) {
          SerialBT.write(69);
          delay(1000);
        }
        if (SerialBT.available()) {
          int incoming = SerialBT.read();
          Serial.print("incoming:"); Serial.println(incoming);
          if ( modeColors[1] == TFT_RED ) {
            modeColors[1] = colorPalette[ incoming ];
          //fadeLed( colorPalette[ incoming ], 1 );
          } else if ( modeColors[2] == TFT_RED ) {
            modeColors[2] = colorPalette[ incoming ];
            //fadeLed( colorPalette[ incoming ], 1 );
          } else if ( modeColors[3] == TFT_RED ) {
            modeColors[3] = colorPalette[ incoming ];
            //fadeLed( colorPalette[ incoming ], 1 );
          } else if ( modeColors[4] == TFT_RED ) {
            modeColors[4] = colorPalette[ incoming ];
            //fadeLed( colorPalette[ incoming ], 1 );
            requestColors = false;
          }
        }
        if (ann > 50001) { //ping
          SerialBT.write(420);
          ann = 0;
          Serial.println("ping sent");
          delay(1000);
        } else {
          ann++;
        }
      }
      //
      if ( toLightSleep ) {
        Serial.println("batteryInfo: to light sleep - " + String(toLightSleep));
        showMessage("Light z_z Sleep!", 3000);        
      } else {
        showVoltage();
      }
      if ( macroMode == 0 ) {
        fadeLed( TFT_BLACK, 1 );
      } else {
        fadeLed( modeColors[ macroMode ], 1 );
      }
      delay(screenDelay);
    }
    //
    Serial.println("batteryInfo: to deep sleep - " + String(toDeepSleep));
    fadeLed( TFT_BLACK, 1 );
    showMessage("Deep Z_Z Sleep!", 6000);
    sleepyByTime();
  }
}

void buttonChecker(void *arg)
{
  if ( !digitalRead( indexBtn ) || !digitalRead( middleBtn ) || !digitalRead( ringBtn ) || !digitalRead( pinkyBtn )  ) {
      Serial.println("mode 2");
      buttonMode = 2;
  } else {
    Serial.println("mode 1");
    buttonMode = 1;    
  }
  while ( 1 ) {
    while( ! toDeepSleep ) {
      if ( buttonMode == 1 ) {
        if ( buttonCheckTemporary() ) {
          screenDelay = 1;
          if ( toLightSleep ) {
            toLightSleep = false;
            screenBacklight(true);            
          }
        } else {
          screenDelay = 10;
        }        
      } else {
        if ( buttonCheckMomentary() ) {
          screenDelay = 1;
        } else {
          screenDelay = 10;
        }
      }
      delay(1);      
    }
    delay(10);
  }
}

void loop() { }


// EPaper setup
/*#include <GxEPD.h>
#include <GxGDEH0154D67/GxGDEH0154D67.h>

#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>*/
#define USE_HSPI_FOR_EPD
#define ENABLE_GxEPD2_GFX 0

#include <GxEPD2.h>
#include <GxEPD2_BW.h>
//#include <GxGDEH0154D67/GxGDEH0154D67.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>

#include <SPI.h>


#if defined ARDUINO_ARCH_STM32F1
  #if defined _BOARD_MAPLE_MINI_H_
    #define EPD_RST_PIN  26      
    #define EPD_BUSY_PIN 27  
    #define EPD_DC_PIN   25
    #define EPD_CS_PIN   31 
  #else
    #define EPD_RST_PIN  PB3  // PA9      
    #define EPD_BUSY_PIN PB4  // PA10  
    #define EPD_DC_PIN   PA15 // PA8
    #define EPD_CS_PIN   PB12
  #endif
#else
  #if defined ARDUINO_ARCH_RP2040
    #define EPD_RST_PIN  10       
    #define EPD_BUSY_PIN  9   
    #define EPD_DC_PIN   11
    #define EPD_CS_PIN   13   
  #else
    #define EPD_RST_PIN   9       
    #define EPD_BUSY_PIN 10   
    #define EPD_DC_PIN   11
    #define EPD_CS_PIN   13 
  #endif
#endif

GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display(GxEPD2_154_D67(/*CS=D8*/ EPD_CS_PIN, /*DC=D3*/ EPD_DC_PIN, /*RST=D4*/ EPD_RST_PIN, /*BUSY=D2*/ EPD_BUSY_PIN)); // GDEH0154D67

//GxIO_Class io(SPI1, EPD_CS_PIN, EPD_DC_PIN, EPD_RST_PIN);
//GxEPD_Class display(io, EPD_RST_PIN, EPD_BUSY_PIN);

typedef struct {
  uint16_t co2 = 0;
  float temperature = 0;
  uint8_t humidity = 0;
  bool lowbatt = false;
} DisplayDataType;

DisplayDataType DisplayData;


void MeasurementsDisplay(const void* parameters)
// uint16_t co2, float temperature, uint8_t humidity
{
  int16_t tbx, tby; 
  uint16_t tbw, tbh, x, y;
  uint8_t humidity;
  
  String strco2  = String(DisplayData.co2, DEC);
  String strtemp = String(DisplayData.temperature, 1);
  String strhum  =  String(DisplayData.humidity, DEC);  
  String strstatic = "";   
  
  display.setRotation(3);
  display.setTextColor(GxEPD_BLACK);
  display.fillScreen(GxEPD_WHITE);
/*
  display.fillScreen(GxEPD_BLACK);
  display.fillRect(2,2,196,196,GxEPD_WHITE);
  display.fillRect(0,99,200,2,GxEPD_BLACK);   
  display.fillRect(99,100,2,100,GxEPD_BLACK);  
*/
  strstatic = "ppm CO2";
  display.setFont(&FreeMonoBold12pt7b);   
  display.getTextBounds(strstatic, 0, 0, &tbx, &tby, &tbw, &tbh);  
  //DPRINT("TEXTBOX: x="); DPRINT(tbx); DPRINT(" y="); DPRINT(tby); DPRINT(" w="); DPRINT(tbw); DPRINT(" h="); DPRINTLN(tbh);
  x = display.width() - tbw - tbx - 10 ;
  y = display.height() / 2 - tbh - tby - 10;  
  display.setCursor(x, y); 
  strstatic = "ppm CO";
  display.print(strstatic); 
  x = display.getCursorX();
  y = display.getCursorY() + 5;
  display.setCursor(x, y);   
  display.setFont(&FreeMonoBold9pt7b);   
  display.print("2");       

  humidity = int(DisplayData.humidity);

  strstatic = "oC";
  display.setFont(&FreeMonoBold12pt7b);   
  display.getTextBounds(strstatic, 0, 0, &tbx, &tby, &tbw, &tbh);  
  if (humidity > 0 ) {
    x = display.width() / 2 - tbw - tbx - 10 ;
  } else {
    x = display.width() - tbw - tbx - 10 ;
  }
  y = display.height() / 2 - tbh - tby - 10 + display.height() / 2;  
  display.setCursor(x, y - 5); 
  display.setFont(&FreeMonoBold9pt7b);     
  display.print("o"); 
  x = display.getCursorX();  
  display.setCursor(x, y);  
  display.setFont(&FreeMonoBold12pt7b);      
  display.print("C");   

  if (humidity > 0 ) {
  strstatic = "%rH";
  display.setFont(&FreeMonoBold12pt7b);   
  display.getTextBounds(strstatic, 0, 0, &tbx, &tby, &tbw, &tbh);  
  x = display.width() / 2 - tbw - tbx - 10 + display.width() / 2;
  display.setCursor(x, y); 
  display.print(strstatic);   
  }
  
  display.setTextSize(2);  //
  display.setFont(&FreeMonoBold18pt7b);  
  display.getTextBounds(strco2, 0, 0, &tbx, &tby, &tbw, &tbh);
  // center bounding box by transposition of origin:
  x = ((display.width() - tbw) / 2) - tbx;
  y = ((display.height() / 2 - tbh) / 2) - tby - display.height() / 20;
  display.setCursor(x, y);
  display.print(strco2);
  
  if (humidity > 0 ) {
   display.setTextSize(1);  //
   display.setFont(&FreeMonoBold18pt7b);
  } else {
   display.setFont(&FreeMonoBold18pt7b);
  }
  display.getTextBounds(strtemp, 0, 0, &tbx, &tby, &tbw, &tbh);
  // center bounding box by transposition of origin:
  if (humidity > 0 ) {
    x = ((display.width() / 2 - tbw) / 2) - tbx;
  } else {
    x = ((display.width() - tbw) / 2) - tbx;
  }
  y = ((display.height() / 2 - tbh) / 2) - tby + display.height() / 2 - display.height() / 40;
  display.setCursor(x, y);
  display.print(strtemp);

  if (humidity > 0 ) {
  display.getTextBounds(strhum, 0, 0, &tbx, &tby, &tbw, &tbh);
  // center bounding box by transposition of origin:
  x = ((display.width() / 2 - tbw) / 2) - tbx + display.width() / 2;
  //y = ((display.height() / 2 - tbh) / 2) - tby + display.height() / 2;  
  display.setCursor(x, y);
  display.print(strhum);
  }  
  display.setTextSize(1);  //

  if (DisplayData.lowbatt)
  {
      display.drawRect(15,25,11,5,GxEPD_BLACK);       
      display.drawRect(10,30,20,30,GxEPD_BLACK);         
      display.fillRect(10,60,20,10,GxEPD_BLACK);    
  }
}


void EmptyBattDisplay(const void* parameters)
{
  int16_t tbx, tby; 
  uint16_t tbw, tbh, x, y;
  
  String strstatic = "";   

  DPRINTLN("Empty batt display");
  display.setRotation(1);
  display.setTextColor(GxEPD_BLACK);
  display.fillScreen(GxEPD_BLACK);
  display.fillRect(2,2,196,196,GxEPD_WHITE);


  display.drawRect(50,30,90,30,GxEPD_BLACK);       
  display.drawRect(140,35,10,20,GxEPD_BLACK);         
  display.drawLine(70,80,120,10,GxEPD_BLACK);    

  display.setFont(&FreeMonoBold12pt7b);  
  strstatic = "Connect"; 
  display.getTextBounds(strstatic, 0, 0, &tbx, &tby, &tbw, &tbh);
  // center bounding box by transposition of origin:
  x = ((display.width() - tbw) / 2) - tbx;
  y = ((display.height() - tbh) / 2) - tby + display.height() / 4 - 12;
  display.setCursor(x, y);
  display.print(strstatic);
  strstatic = "USB Charger!";
  display.getTextBounds(strstatic, 0, 0, &tbx, &tby, &tbw, &tbh);
  // center bounding box by transposition of origin:
  x = ((display.width() - tbw) / 2) - tbx;
  y = ((display.height() - tbh) / 2) - tby + display.height() / 4 + 12;
  display.setCursor(x, y);
  display.print(strstatic);  
}



// all library classes are placed in the namespace 'as'
using namespace as;


class ePaperType : public Alarm {

private:
  bool                 mUpdateDisplay;
  bool                 shMeasurementsDisplay;
  bool                 shEmptyBattDisplay;

public:
  ePaperType () :  Alarm(0), mUpdateDisplay(false), shEmptyBattDisplay(false) {}
  virtual ~ePaperType () {}

  bool showEmptyBattDisplay() {
    return shEmptyBattDisplay;
  }

  void showEmptyBattDisplay(bool s) {
    shEmptyBattDisplay = s;
  }

  bool mustUpdateDisplay() {
    return mUpdateDisplay;
  }

  void mustUpdateDisplay(bool m) {
    mUpdateDisplay = m;
  }

  void init() {
    /*
    u8g2Fonts.begin(display);
    u8g2Fonts.setFontMode(1);
    u8g2Fonts.setFontDirection(0);
    */
  }

 
  void setRefreshAlarm (uint32_t t) {
    //isWaiting(true);
    sysclock.cancel(*this);
    Alarm::set(millis2ticks(t));
    sysclock.add(*this);
  }
  
  virtual void trigger (__attribute__((unused)) AlarmClock& clock) {
    DPRINTLN("ePaper refresh triggered");
    if (this->mustUpdateDisplay()) {
      this->mustUpdateDisplay(false);
      display.init();
      if (this->showEmptyBattDisplay() == true ) {
          display.drawPaged(EmptyBattDisplay,0);
          this->showEmptyBattDisplay(false);
      } 
      else {
        display.drawPaged(MeasurementsDisplay,0);
      }
//      display.update();
      delay(50);
      pinMode(EPD_RST_PIN,INPUT);
      digitalWrite(EPD_RST_PIN, LOW);
//      delay(1);
      //pinMode(EPD_RST_PIN,INPUT); //fix current into reset pin in v2 of Waveshare display
    }
  }
  
};

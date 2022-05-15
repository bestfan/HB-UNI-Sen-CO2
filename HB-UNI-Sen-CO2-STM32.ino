//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2019-02-28 jp112sdl (Creative Commons)
//- -----------------------------------------------------------------------------------------------------------------------
// 2020-11-07 CO2 Sensor, HMSteve (cc)
//- -----------------------------------------------------------------------------------------------------------------------
// 2022-04-25 CO2 Sensor on STM32 and other features, bestfan (cc)

//#define NDEBUG   // disable all serial debug messages  

//#define EI_NOTEXTERNAL

//#define SERIAL_OUTPUT  // data in JSONPATH format on serial line, use NDEBUG!
#define RGBLED

//#define useMHZ19  //use MHZ19 CO2 sensor
#define useSCD30  //use SCD30 CO2 sensor
#define useBME280 //use pressure sensor for compensation, in case MH-Z19 use also temperature and humidity

#define BAT_VOLT_LOW        22  // 2.2V for 2x Eneloop 
#define BAT_VOLT_CRITICAL   20  // 2.0V for 2x Eneloop

//#define USE_CC1101_ALT_FREQ

#if defined ARDUINO_ARCH_STM32F1
  #define STORAGEDRIVER at24cX<0x50,128,32>
//#define STORAGEDRIVER InternalEprom   // Alternative

  #define TICKS_PER_SECOND 500UL
  #define USE_HW_SERIAL
  #include <SPI.h>    // when we include SPI.h - we can use LibSPI class
  #include <Wire.h>   // for I2C access to EEPROM
  #include <EEPROM.h> // the EEPROM library contains Flash Access Methods
  #define BATT_SENSOR BatterySensor
  #if defined _BOARD_MAPLE_MINI_H_
    #define SPI1_NSS_PIN 7    //SPI_1 Chip Select pin is 7.
    #define SPI2_NSS_PIN 31   //SPI_2 Chip Select pin is 31.
  #else // Blue Pill
    #define SPI1_NSS_PIN PA4    //SPI_1 Chip Select pin is PA4.
    #define SPI2_NSS_PIN PB12   //SPI_2 Chip Select pin is PB12.
  #endif
  SPIClass SPI_2(2); //Create an instance of the SPI Class called SPI_2 that uses the 2nd SPI Port
#else
  #include <EnableInterrupt.h>
  #include <LowPower.h>
#endif

#include <AskSinPP.h>
#include <Register.h>
#include <MultiChannelDevice.h>
#ifndef ARDUINO_ARCH_STM32F1
  #include "sensors/tmBattery.h"  //SG: added from Tom's UniSensor project
#endif
#if defined useMHZ19
  #if defined ARDUINO_ARCH_STM32F1
    #define co2Serial Serial3  // serial3
  #else
    #define co2Serial (RX, TX)
  #endif
  #include "sensors/Sens_MHZ19.h"
#endif
#if defined useSCD30
  #include "sensors/Sens_SCD30.h"
#endif
#if defined useBME280
  #include "sensors/Sens_BME280.h"
#endif
#include "EPDisplay.h"


#if defined ARDUINO_ARCH_STM32F1 && defined RGBLED
  #include "bluepill_ws2812.h"
  bluepill_neopixel PIX;       // a string of pixels
  #define NUM_PIXELS 1     //   number of pixels in the string
  pixel string[NUM_PIXELS]; //   rgb data buffer
  #define RGBLED_PORT GPIOA //   pin string is connected to
  #if defined _BOARD_MAPLE_MINI_H_
    #define RGBLED_PIN  2
  #else
    #define RGBLED_PIN  8
  #endif
  #include "RgbLed.h"
#else
  #include "DuoLed.h"
#endif
#include "UserButton.h"

#if defined ARDUINO_ARCH_STM32F1
  #if defined _BOARD_MAPLE_MINI_H_
    #define CC1101_GDO0_PIN     3 // 
    #define CC1101_CS_PIN       7 // original PA4
    // CC1101 communication uses HW-SPI
    #define LED_PIN             11 // alternative 11 or LED_BUILTIN
    #define LED2_PIN            10
    #define DUOLED_GN_PIN       18 // suggested, not connected on PCB
    #define DUOLED_RT_PIN       19 // suggested, not connected on PCB
    #define CONFIG_BUTTON_PIN   8
    #define USER_BUTTON_PIN     17 // alternative 17 or 32 (build in)
    //  #define CC1101_PWR_SW_PIN   PA8
  #else
    // we use a BluePill
    #define CC1101_GDO0_PIN     PB0
    #define CC1101_CS_PIN       PA4 // original PA4
    // CC1101 communication uses HW-SPI
    #define LED_PIN             PB1  // aternative PB1 or LED_BUILTIN
    #define LED2_PIN            PA3
    #define DUOLED_GN_PIN       PB8 // suggested, not connected on PCB
    #define DUOLED_RT_PIN       PB9 // suggested, not connected on PCB
    #define CONFIG_BUTTON_PIN   PA1
    #define USER_BUTTON_PIN     PB5 
    //  #define CC1101_PWR_SW_PIN   PA8
  #endif
#else
  // Pin definitions Stephan's HB-UNI-Sen-CO2 Board v1.0
  #define CC1101_CS_PIN        4
  #define CC1101_GDO0_PIN      2
  #define CC1101_SCK_PIN       7 
  #define CC1101_MOSI_PIN      5 
  #define CC1101_MISO_PIN      6 
  #define LED_PIN             12              
  #define LED2_PIN            15
  #define DUOLED_GN_PIN       18
  #define DUOLED_RT_PIN       19
  #define CONFIG_BUTTON_PIN   14
  #define USER_BUTTON_PIN     13 
  #define CC1101_PWR_SW_PIN   27
#endif

#define PEERS_PER_CHANNEL 6
//tmBatteryLoad: sense pin A6, activation pin 26, Faktor = Rges/Rlow*1000, z.B. 10/30 Ohm, Faktor 40/10*1000 = 4000, 200ms Belastung vor Messung
// 1248p has 2.56V ARef, 328p has 1.1V ARef, so output needs to be scaled up
#ifndef ARDUINO_ARCH_STM32F1
  #define BATT_SENSOR tmBatteryLoad<A6, 26, (uint16_t)(4000.0*2.56/1.1), 200>
#endif


// all library classes are placed in the namespace 'as'
using namespace as;

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
  {0xf8, 0x22, 0x01},     // Device ID
  "CO2-000000",           // Device Serial
  {0xf8, 0x22},           // Device Model Indoor //orig 0xf1d1
  0x10,                   // Firmware Version
  as::DeviceType::THSensor, // Device Type
  {0x00, 0x01}            // Info Bytes
};




//Configure the used hardware
#if defined ARDUINO_ARCH_STM32F1
  typedef LibSPI<CC1101_CS_PIN> RadioSPI;
  typedef Radio<RadioSPI, CC1101_GDO0_PIN> RadioType;
#else
  typedef AvrSPI<CC1101_CS_PIN, CC1101_MOSI_PIN, CC1101_MISO_PIN, CC1101_SCK_PIN> SPIType;
  typedef Radio<SPIType, CC1101_GDO0_PIN> RadioType;
#endif
typedef StatusLed<LED_PIN> LedType;
typedef AskSin<LedType, BATT_SENSOR, RadioType> BaseHal;
typedef StatusLed<LED2_PIN> Led2Type;
#if defined RGBLED
  typedef RgbLed<RGBLED_PORT, RGBLED_PIN> RgbLedType;
#else
  typedef DuoLed<DUOLED_GN_PIN, DUOLED_RT_PIN> DuoLedType;
#endif


class Hal : public BaseHal {
  public:
    void init (const HMID& id) {
      BaseHal::init(id);
#ifdef USE_CC1101_ALT_FREQ
      radio.initReg(CC1101_FREQ2, 0x21);
      radio.initReg(CC1101_FREQ1, 0x65);
      radio.initReg(CC1101_FREQ0, 0xE2);
      //0x216572 868.303 MHz
#endif
// measure battery every a*b*c seconds
      battery.init(seconds2ticks(60UL * 30 * 1), sysclock);  
      battery.low(BAT_VOLT_LOW);
      battery.critical(BAT_VOLT_CRITICAL);
    }

    bool runready () {
      return sysclock.runready() || BaseHal::runready();
    }
} hal;


DEFREGISTER(Reg0, MASTERID_REGS, DREG_LEDMODE, DREG_TRANSMITTRYMAX, 0x20, 0x21, 0x22, 0x23, 0x24)
class SensorList0 : public RegList0<Reg0> {
  public:
    SensorList0(uint16_t addr) : RegList0<Reg0>(addr) {}

    bool updIntervall (uint16_t value) const {
      return this->writeRegister(0x20, (value >> 8) & 0xff) && this->writeRegister(0x21, value & 0xff);
    }
    uint16_t updIntervall () const {
      return (this->readRegister(0x20, 0) << 8) + this->readRegister(0x21, 0);
    }
    
    bool altitude (uint16_t value) const {
      return this->writeRegister(0x22, (value >> 8) & 0xff) && this->writeRegister(0x23, value & 0xff);
    }
    uint16_t altitude () const {
      return (this->readRegister(0x22, 0) << 8) + this->readRegister(0x23, 0);
    }

    bool tempOffset10 (uint8_t value) const {
      return this->writeRegister(0x24, value & 0xff);
    }
    uint16_t tempOffset10 () const {
      return this->readRegister(0x24, 0);
    }    


    void defaults () {
      clear();
      ledMode(1);
      transmitDevTryMax(3);     
      updIntervall(60); //seconds
      altitude(94); //meters
      tempOffset10(0); //temperature offset for SCD30 calib: 15 means 1.5K
    }
};



DEFREGISTER(Reg1, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07)
class SensorList1 : public RegList1<Reg1> {
  public:
    SensorList1 (uint16_t addr) : RegList1<Reg1>(addr) {}

    bool co2CalibRef (uint16_t value) const {
      return this->writeRegister(0x01, (value >> 8) & 0xff) && this->writeRegister(0x02, value & 0xff);
    }
    
    uint16_t co2CalibRef () const {
      return (this->readRegister(0x01, 0) << 8) + this->readRegister(0x02, 0);
    }

    bool co2StateAmber (uint16_t value) const {
      return this->writeRegister(0x03, (value >> 8) & 0xff) && this->writeRegister(0x04, value & 0xff);
    }
    
    uint16_t co2StateAmber () const {
      return (this->readRegister(0x03, 0) << 8) + this->readRegister(0x04, 0);
    }

    bool co2StateRed (uint16_t value) const {
      return this->writeRegister(0x05, (value >> 8) & 0xff) && this->writeRegister(0x06, value & 0xff);
    }
    
    uint16_t co2StateRed () const {
      return (this->readRegister(0x05, 0) << 8) + this->readRegister(0x06, 0);
    }

    bool co2NoGreen (uint16_t value) const {
      return this->writeRegister(0x07, value & 0xff);
    }
    
    uint16_t co2NoGreen () const {
      return this->readRegister(0x07, 0);
    }    

    void defaults () {
      clear();
      co2CalibRef(410); //410ppm in fresh air
      co2StateAmber(1000);     
      co2StateRed(2000); 
      co2NoGreen(0);       
    }
};



class WeatherEventMsg : public Message {
  public:
    void init(uint8_t msgcnt, int16_t temp, uint8_t humidity, uint16_t pressureNN, uint16_t co2, uint16_t volt, bool batlow) {
      uint8_t t1 = (temp >> 8) & 0x7f;
      uint8_t t2 = temp & 0xff;
      if ( batlow == true ) {
        t1 |= 0x80; // set bat low bit
      }
      // BIDI|WKMEUP every 20th msg
      uint8_t flags = BCAST;
      if ((msgcnt % 20) == 1) {
          flags = BIDI | WKMEUP;
      }      
      // Message Length (first byte param.): 11 + payload. Max. payload: 17 Bytes (https://www.youtube.com/watch?v=uAyzimU60jw)
      Message::init(18, msgcnt, 0x70, flags, t1, t2);
      pload[0] = humidity & 0xff;
      pload[1] = (pressureNN >> 8) & 0xff;           
      pload[2] = pressureNN & 0xff;          
      pload[3] = (co2 >> 8) & 0xff;           
      pload[4] = co2 & 0xff;         
      pload[5] = (volt >>8) & 0xff;
      pload[6] = volt & 0xff;      
    }
};




class WeatherChannel : public Channel<Hal, SensorList1, EmptyList, List4, PEERS_PER_CHANNEL, SensorList0>, public Alarm {
    WeatherEventMsg     msg;
    bool                firstMsg = true;
    uint16_t            millis;
    uint16_t            pressureAmb = 1013; //mean pressure at sea level in hPa
    uint16_t            pressureNN = 0; //dummy value to be returned if no sensor measurement
    uint16_t            co2 = 0;
    uint16_t            temp10 = 0;
    uint8_t             humidity = 0;
    RgbLedType          trafficLight; 
    bool                trafficLightEnabled = true;
    ePaperType          ePaper;
#if defined useSCD30
    Sens_SCD30          scd30;
#endif               
#if defined useMHZ19
    Sens_MHZ19          mhz19;
#endif               
#if defined useBME280
    Sens_BME280         bme280;
#endif       
       

  public:
    WeatherChannel () : Channel(), Alarm(10), millis(0) {}
    
    virtual ~WeatherChannel () {}

    virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
      // reactivate for next measure
      tick = delay();
      clock.add(*this);

      // measure
      #if defined useBME280
        bme280.measure(this->device().getList0().altitude());
        pressureAmb = bme280.pressure()/10;
        pressureNN = bme280.pressureNN();
        #if defined useSCD30
          temp10 = scd30.temperature() - this->device().getList0().tempOffset10();
        #else
          temp10 = bme280.temperature()- this->device().getList0().tempOffset10();
          humidity = bme280.humidity();
        #endif
      #endif
      #if defined useSCD30
         scd30.measure(pressureAmb);
      #endif
      #if defined useMHZ19
         mhz19.measure();
      #endif
     
      // suppress potentially erranous first CO2 measurements after reset due to connecting charger
      if (!firstMsg) {
        #if defined useSCD30
          co2 = scd30.carbondioxide();
          temp10 = scd30.temperature() - this->device().getList0().tempOffset10();
          humidity = scd30.humidity();        
        #endif
        #if defined useMHZ19
          co2 = mhz19.carbondioxide();
          #if !defined useBME280
            temp10 = mhz19.temperature() - this->device().getList0().tempOffset10();
            humidity = 0;
          #endif        
        #endif          
        DPRINT("Temp x10 / Hum / PressureNN x10 / PressureAmb / Batt x10 / CO2 = ");
        DDEC(temp10 / 10);DPRINT(" / ");
        DDEC(humidity);DPRINT(" / ");
        DDEC(pressureNN);DPRINT(" / ");
        DDEC(pressureAmb);DPRINT(" / ");     
        DDEC(device().battery().current() / 10);DPRINT(" / ");
        DDECLN(co2);

        uint8_t msgcnt = device().nextcount();
        msg.init( msgcnt, temp10, humidity, pressureNN, co2, device().battery().current() / 10, device().battery().low());
        if (msg.flags() & Message::BCAST) 
        {
          device().broadcastEvent(msg, *this);
        }
        else
        {
          device().sendPeerEvent(msg, *this);
        }
        
        DisplayData.co2 = co2;
        DisplayData.temperature = temp10 / 10.0;
        DisplayData.humidity = humidity;   
        DisplayData.lowbatt = device().battery().low();    
        ePaper.mustUpdateDisplay(true);
        ePaper.setRefreshAlarm(500);
#if defined SERIAL_OUTPUT
        Serial.println("{\"sensor\":{\"temp\":" + String(temp10 / 10.0) + "\,\"pressureAmb\":" + String(pressureAmb) + "\,\"pressureNN\":" + String(pressureNN)  + "\,\"humidity\":" + String(humidity) + "\,\"CO2\":" + String(co2) + "}}" );
#endif        
        setTrafficLight();
      };
      firstMsg = false;
    }


    uint32_t delay () {
      return seconds2ticks(this->device().getList0().updIntervall());
    }
    
    void setup(Device<Hal, SensorList0>* dev, uint8_t number, uint16_t addr) {
      Channel::setup(dev, number, addr);
      
      #if defined useSCD30
        uint16_t scd30SamplingInterval = this->device().getList0().updIntervall() / 5;
        if (scd30SamplingInterval < 1) {scd30SamplingInterval = 1;}       
        if (scd30SamplingInterval > 30) {scd30SamplingInterval = 30;}
        DPRINT("SCD30 sampling interval : ");DDECLN(scd30SamplingInterval);
        scd30.init(device().getList0().altitude(), this->device().getList0().tempOffset10(), scd30SamplingInterval, false);
      #endif
      #if defined useMHZ19
        mhz19.init(true);
      #endif
      
      #if defined useBME280
        bme280.init();
      #endif
      
      trafficLight.init();
      
      sysclock.add(*this);
    }

    uint8_t status () const {
      return 0;
    }

    uint8_t flags () const {
      return 0;
    }

    #if defined useSCD30
    bool forceCalibSCD30()
    {
      return(scd30.setForcedRecalibrationFactor(this->getList1().co2CalibRef()));
    }


    bool setSCD30SamplingInterval()
    {
      uint16_t scd30SamplingInterval = this->device().getList0().updIntervall() / 5;
      if (scd30SamplingInterval < 1) {scd30SamplingInterval = 1;}      
      if (scd30SamplingInterval > 30) {scd30SamplingInterval = 30;}
      return(scd30.setSamplingInterval(scd30SamplingInterval));
    }


    bool setMHZ19SamplingInterval()
    {
      uint16_t scd30SamplingInterval = this->device().getList0().updIntervall() / 5;
      if (scd30SamplingInterval < 1) {scd30SamplingInterval = 1;}      
      if (scd30SamplingInterval > 30) {scd30SamplingInterval = 30;}
      return(scd30.setSamplingInterval(scd30SamplingInterval));
    }


    void stopSCD30()
    {
      scd30.stop_measurements();   
    }
    #endif

    
    #if defined useMHZ19
    bool forceCalibMHZ19()
    {
      return(mhz19.setForcedRecalibrationFactor(this->getList1().co2CalibRef()));
    }


/*    bool setMHZ19SamplingInterval()
    {
      uint16_t mhz19SamplingInterval = this->device().getList0().updIntervall() / 5;
      if (mhz19SamplingInterval < 1) {mhz19SamplingInterval = 1;}      
      if (mhz19SamplingInterval > 30) {mhz19SamplingInterval = 30;}
      return(mhz19.setSamplingInterval(mhz19SamplingInterval));
    }*/


    void stopMHZ19()
    {
      mhz19.stop_measurements();   
    }
    #endif


    void setTrafficLight()
    {
      DPRINT("setTrafficLight");
      if (trafficLightEnabled)
      {
        if ((co2 <= this->getList1().co2StateAmber()) and (this->getList1().co2NoGreen())) trafficLight.setOff(&string[0]);
        if ((co2 <= this->getList1().co2StateAmber()) and !(this->getList1().co2NoGreen())) trafficLight.setGreen(&string[0]);
        if ((co2 > this->getList1().co2StateAmber()) and (co2 <= this->getList1().co2StateRed())) trafficLight.setAmber(&string[0]);
        if (co2 > this->getList1().co2StateRed()) trafficLight.setRed(&string[0]);
      }
      else
      {
        trafficLight.setOff(&string[0]);
      }
      PIX.paint( string[0].bytes, NUM_PIXELS, RGBLED_PORT, RGBLED_PIN );
    }

    bool toggleTrafficLight()
    {
      trafficLightEnabled = !trafficLightEnabled;
      setTrafficLight();
      return(trafficLightEnabled);
    }
};




class SensChannelDevice : public MultiChannelDevice<Hal, WeatherChannel, 1, SensorList0> {
  public:
    typedef MultiChannelDevice<Hal, WeatherChannel, 1, SensorList0> TSDevice;
    Led2Type    led2;
    
    SensChannelDevice(const DeviceInfo& info, uint16_t addr) : TSDevice(info, addr) 
    {
      led2.init();
    }
    
    virtual ~SensChannelDevice () {}

    virtual void configChanged () {
      TSDevice::configChanged();
    #if defined useSCD30
      this->channel(0).setSCD30SamplingInterval();
    #endif
    #if defined useMHZ19
//      this->channel(0).setMHZ19SamplingInterval();
    #endif
      DPRINTLN("* Config Changed       : List0");
      DPRINT(F("* LED Mode             : ")); DDECLN(this->getList0().ledMode());    
      DPRINT(F("* Sendeversuche        : ")); DDECLN(this->getList0().transmitDevTryMax());                   
      DPRINT(F("* SENDEINTERVALL       : ")); DDECLN(this->getList0().updIntervall());
      DPRINT(F("* Hoehe ueber NN       : ")); DDECLN(this->getList0().altitude());
      DPRINT(F("* Temp Offset x10      : ")); DDECLN(this->getList0().tempOffset10());   
         
    }
};



SensChannelDevice sdev(devinfo, 0x20);
//ConfigButton<SensChannelDevice> cfgBtn(sdev);
myConfigButton<SensChannelDevice> cfgBtn(sdev);   // for use of TTP223 without using solder pad
UserButton<SensChannelDevice> usrBtn(sdev);


void setup () {
  #if defined useMHZ19
    co2Serial.begin(9600);
    delay(1000);
  #endif
  //SG: switch on MOSFET to power CC1101 and sensors
  #ifndef ARDUINO_ARCH_STM32F1
    pinMode(CC1101_PWR_SW_PIN, OUTPUT);
    digitalWrite (CC1101_PWR_SW_PIN, LOW);
  #endif
  //init ePaper display
  //display.init(); //not done here because v2 of Waveshare display draws about continuous 3mA via Reset pin
  #if defined NDEBUG && defined SERIAL_OUTPUT
    Serial.begin(57600);
  #else
    DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  #endif
  #if defined ARDUINO_ARCH_STM32F1
    Wire.begin();
  #endif
  DDEVINFO(sdev);
  sdev.init(hal);
  buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
  buttonISR(usrBtn, USER_BUTTON_PIN);  
  sdev.initDone();
  DPRINT("List0 dump: "); sdev.getList0().dump();
//  display.init(57600);
//  display.init();
}


void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if ( worked == false && poll == false ) {
    if (hal.battery.critical()) {
      // this call will never return
      DPRINTLN("!!!Shutting down due to critical battery voltage!!!");
      #if defined useSCD30
        sdev.channel(0).stopSCD30();
      #endif
      #if defined useMHZ19
        sdev.channel(0).stopMHZ19();           
      #endif
      //display.init();
      //display.drawPaged(EmptyBattDisplay);
      //wait a bit to let paged epd refresh finish
      //delay(5000);
#ifndef ARDUINO_ARCH_STM32F1
      pinMode(CC1101_PWR_SW_PIN, INPUT);
#endif
      hal.activity.sleepForever(hal);      
    }    
    // if nothing to do - go to sleep
#ifndef ARDUINO_ARCH_STM32F1
    hal.activity.savePower<Sleep<>>(hal)
#endif
  }
}

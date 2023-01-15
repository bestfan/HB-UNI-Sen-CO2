//---------------------------------------------------------
// 2022-04-27 bestfan (cc)
// Sens_MHZ19 based on Sens_SCD30 by.
// 2019-07-14 Tom Major (Creative Commons)
// https://creativecommons.org/licenses/by-nc-sa/4.0/
// You are free to Share & Adapt under the following terms:
// Give Credit, NonCommercial, ShareAlike
// +++
// AskSin++ 2016-10-31 papa Creative Commons
//---------------------------------------------------------

#ifndef _SENS_MHZ19_H_
#define _SENS_MHZ19_H_

#include <Sensors.h>
//#include <Wire.h>
#include "MHZ19.h"


/* stuff not implemeted for MH-Z19
#define COMMAND_STOP_MEASUREMENT        0x0104 // supplemented in order to stop the measurements in case of an empty accumulator battery
#define COMMAND_SOFT_RESET              0xD304 // supplemented to use before begin in case low battery caused undefined state
#define COMMAND_READ_FIRMWARE_VERSION   0xD100
*/

namespace as {

class Sens_MHZ19 : public Sensor {

    int16_t  _temperature;
    uint32_t _carbondioxide;
    MHZ19    _mhz19;

    void measureRaw()
    {
        DPRINTLN("");
        while (!_mhz19.available())
        {
            delay(1000);
            DPRINT(".");
           DPRINTLN("Error: no Winsen MHZ19 CO2 sensor found");
           if (_mhz19.available())
            {
              break;
            }
        }
        if (_mhz19.available())
        {
              _carbondioxide    = (uint16_t)(_mhz19.getCO2(true, true));
              _temperature      = (int16_t)(_mhz19.getTemperature(true, false)); 

              DPRINTLN("");
              delay(1000);
        }
    }

public:
    Sens_MHZ19()
        : _temperature(0)
        , _carbondioxide(0)
    {
    }


    void init(bool auto_self_calib)
    {
        //Wire.begin();
        //_scd30.sendCommand(COMMAND_SOFT_RESET);
        //delay(200);
        _mhz19.begin(co2Serial);
        if (_mhz19.available() == false)
        {
          DPRINTLN("Error: no Winsen MHZ19 CO2 sensor found");
        }
        else
        {
          DPRINTLN(F("MHZ19 found"));
          _present = true;
          _mhz19.autoCalibration(auto_self_calib);                    // enable/disable auto self calibration, sensor needs to see fresh air regularly!
        }
    }


    bool setForcedRecalibrationFactor(uint16_t concentration)
    {
        //concentration not used for MH-Z19
        _mhz19.calibrate();
        return 0;
    }


    float getTemperatureOffset(void)
    {
//      return _mhz19.getTemperatureOffset();
        return 0.0;
    }


    void stop_measurements()
    {
//        _mhz19.sendCommand(COMMAND_STOP_MEASUREMENT);
        DPRINTLN("Stop continuous measurements of MHZ19");
    }


    void measure()
    {
        _temperature = _carbondioxide = 0;
        if (_present == true) {
            measureRaw();
            DPRINT(F("MHZ19    Temperature   : "));
            DDECLN(_temperature);
            DPRINT(F("MHZ19    CO2           : "));
            DDECLN(_carbondioxide);
        }
    }

    void read_firmware_version()
    {
      DPRINTLN("bla");
    }

    int16_t  temperature() { return _temperature; }
    uint32_t carbondioxide() { return _carbondioxide; }
};

}

#endif


/*   Version: 1.5.3  |  License: LGPLv3  |  Author: JDWifWaf@gmail.com   */
/*   same as verify fuction in MH-Z19 library but with boolean return    */
/*   by bestfan 2022-04-27                                               */

bool MHZ19::available()
{
    unsigned long timeStamp = millis();

    /* construct common command (133) */
    constructCommand(CO2UNLIM);

    write(this->storage.constructedCommand);

    while (read(this->storage.responses.CO2UNLIM, CO2UNLIM) != RESULT_OK)
    {
        if (millis() - timeStamp >= TIMEOUT_PERIOD)
        {
           #if defined (ESP32) && (MHZ19_ERRORS)
            ESP_LOGE(TAG_MHZ19, "Failed to verify connection(1) to sensor.");   
            #elif MHZ19_ERRORS
            Serial.println("!ERROR: Failed to verify connection(1) to sensor.");
            #endif   

            return 0;
        }
    }

    /* construct & write last response command (162) */
    constructCommand(GETLASTRESP);
    write(this->storage.constructedCommand);
    
    /* update timeStamp  for next comms iteration */ 
    timeStamp = millis();

    while (read(this->storage.responses.STAT, GETLASTRESP) != RESULT_OK)
    {
        if (millis() - timeStamp >= TIMEOUT_PERIOD)
        {
            #if defined (ESP32) && (MHZ19_ERRORS)
            ESP_LOGE(TAG_MHZ19, "Failed to verify connection(2) to sensor.");   
            #elif MHZ19_ERRORS
            Serial.println("!ERROR: Failed to verify connection(2) to sensor.");
            #endif
            
            return 0;
        }
    }      

    /* compare CO2 & temp bytes, command(133), against last response bytes, command (162)*/
    for (byte i = 2; i < 6; i++)
    {
        if (this->storage.responses.CO2UNLIM[i] != this->storage.responses.STAT[i])
        {
            #if defined (ESP32) && (MHZ19_ERRORS)
            ESP_LOGE(TAG_MHZ19, "Last response is not as expected, verification failed.");   
            #elif MHZ19_ERRORS
            Serial.println("!ERROR: Last response is not as expected, verification failed.");
            #endif

            return 0;
        }
    }
    return 1;
}

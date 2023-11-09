//---------------------------------------------------------
// 2022-10-24 bestfan (cc)
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
        _mhz19.verify();
        while (_mhz19.errorCode != RESULT_OK)
        {
            delay(1000);
            DPRINT(".");
            DPRINTLN("Error: no Winsen MHZ19 CO2 sensor found");
//           if (_mhz19.available())
            _mhz19.verify();
            if (_mhz19.errorCode == RESULT_OK)
	    {
	      break;
	    }
        }
        _mhz19.verify();
        if (_mhz19.errorCode == RESULT_OK)
        {
              _carbondioxide    = (uint16_t)(_mhz19.getCO2(true));
              _temperature      = (int16_t)(_mhz19.getTemperature(false)); 

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
//       if (_mhz19.available() == false)
        _mhz19.verify();
        if (_mhz19.errorCode != RESULT_OK) 
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
//      return(_mhz19.setForcedRecalibrationFactor(concentration));
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


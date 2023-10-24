//---------------------------------------------------------
// Sens_DS18X20
// Version 1.01
// (C) 2018-2020 Tom Major (Creative Commons)
// https://creativecommons.org/licenses/by-nc-sa/4.0/
// You are free to Share & Adapt under the following terms:
// Give Credit, NonCommercial, ShareAlike
// +++
// AskSin++ 2016-10-31 papa Creative Commons
//---------------------------------------------------------

#ifndef _SENS_DS18X20_H_
#define _SENS_DS18X20_H_

#include <Sensors.h>
#include <OneWire.h>

namespace as {

class Sens_DS18X20 : public Sensor {

public:
    Sens_DS18X20()
        : _oneWire(0)
        , _temperature(-990)
    {
    }

    static uint8_t init(OneWire& ow, Sens_DS18X20* devices, uint8_t maxSearchCount)
    {
        uint8_t count = 0, a[8];
        DPRINTLN(F("TEST: DS18x20 init"));        
//        while ((count < maxSearchCount)) {
        while ((ow.search(a) == 1) && (count < maxSearchCount)) {
         DPRINTLN(F("here i am 1"));
           if ((OneWire::crc8(a, 7) == a[7]) && (Sens_DS18X20::valid(a) == true)) {
                devices->init(ow, a);
         DPRINTLN(F("here i am 2"));
             DDECLN(count);
               count++;
                devices++;
            }
        }
        ow.reset_search();
        if (count) {
            DPRINT(F("DS18x20 count: "));
            DDECLN(count);
        } else {
            DPRINTLN(F("ERROR: no DS18x20 found"));
        }
        return count;
    }

    static bool valid(uint8_t* addr) { return *addr == 0x10 || *addr == 0x22 || *addr == 0x28; }

    static void measure(Sens_DS18X20* devices, uint8_t count)
    {
        if (count > 0) {
            devices->convert(true);    // this will start all DS18x20 on the same 1-wire bus
            devices->wait();
            for (uint8_t i = 0; i < count; i++, devices++) {
                devices->read();
            }
        }
    }

    void measure(__attribute__((unused)) bool async = false)
    {
        _temperature = -990;
        if (_present == true) {
            convert(false);
            wait();
            read();
        }
    }

    int16_t temperature() { return _temperature; }

private:
    ::OneWire* _oneWire;
    int16_t    _temperature;
    uint8_t    _addr[8];

    void init(OneWire& oneWire, uint8_t* addr)
    {
        _oneWire = &oneWire;
        _present = true;
        DPRINT(F("DS18x20 found: "));
        for (uint8_t i = 0; i < 8; i++) {
            _addr[i] = addr[i];
            DHEX(addr[i]);
        }
        DPRINTLN("");
    }

    void convert(bool multipleDevices)
    {
        _oneWire->reset();
        if (multipleDevices) {
            _oneWire->skip();
        } else {
            _oneWire->select(_addr);
        }
        _oneWire->write(0x44);    // start conversion, use ds.write(0x44,1) with parasite power on at the end
    }

    void wait() { delay(1050); }

    void read()
    {
        _temperature = -990;
        _oneWire->reset();
        _oneWire->select(_addr);
        _oneWire->write(0xBE);    // Read Scratchpad

        uint8_t data[9];
        for (uint8_t i = 0; i < 9; i++) {    // we need 9 bytes
            data[i] = _oneWire->read();
        }

        if (OneWire::crc8(data, 8) == data[8]) {
            int16_t raw = (data[1] << 8) | data[0];
            if (_addr[0] == 0x10) {
                raw = raw << 3;    // 9 bit resolution default
                if (data[7] == 0x10) {
                    // "count remain" gives full 12 bit resolution
                    raw = (raw & 0xFFF0) + 12 - data[6];
                }
            } else {
                byte cfg = (data[4] & 0x60);
                // at lower res, the low bits are undefined, so let's zero them
                if (cfg == 0x00) {
                    raw = raw & ~7;    // 9 bit resolution, 93.75 ms
                } else if (cfg == 0x20) {
                    raw = raw & ~3;    // 10 bit res, 187.5 ms
                } else if (cfg == 0x40) {
                    raw = raw & ~1;    // 11 bit res, 375 ms
                }
                // default is 12 bit resolution, 750 ms conversion time
            }
            _temperature = (raw * 10) / 16;
            DPRINT(F("DS18x20  Temperature   : "));
            DDECLN(_temperature);
        } else {
            DPRINTLN(F("ERROR: DS18x20 crc"));
        }
    }
};

}

#endif

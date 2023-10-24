namespace as {

//template <uint8_t LEDRGB_PIN, class PINTYPE=ArduinoPins>
class RgbLed : LedStates {
    
  public:
    RgbLed() {};
    
    virtual ~RgbLed() {};

    void init() 
    {
      DPRINTLN("TrafficLightInit");
      pixels.begin();
    }

    void setBlue() // not use but today, maybe in future updates
    {
      pixels.setPixelColor(0, pixels.Color(0, 0, 255));
      DPRINTLN("TrafficLightBlue");
    };

    void setGreen() 
    {
      pixels.setPixelColor(0, pixels.Color(255, 0, 40));
      DPRINTLN("TrafficLightGreen");
    };

    void setAmber() 
    {
      pixels.setPixelColor(0, pixels.Color(255, 225, 0));
      DPRINTLN("TrafficLightAmber");
    };

    void setRed() 
    {
      pixels.setPixelColor(0, pixels.Color(0, 255, 0));
      DPRINTLN("TrafficLightRed");
    };

    void setOff() 
    {
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      DPRINTLN("TrafficLightOff");
    };

};

}

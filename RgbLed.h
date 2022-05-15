namespace as {

template <uint32_t LEDRGB_PORT, uint8_t LEDRGB_PIN, class PINTYPE=ArduinoPins>
class RgbLed : LedStates {
  private:
    Led<PINTYPE> ledrgb;
    
  public:
    RgbLed() {};
    
    virtual ~RgbLed() {};

    void init() 
    {
      DPRINTLN("TrafficLightInit");
      PIX.begin(LEDRGB_PORT, LEDRGB_PIN); // set pin to output
    }

    void setBlue(pixel *p) // not use but today, maybe in future updates
    {
      p->rgb.r = 0;
      p->rgb.g = 0;
      p->rgb.b = 255;
      DPRINTLN("TrafficLightBlue");
    };

    void setGreen(pixel *p) 
    {
      p->rgb.r = 0;
      p->rgb.g = 255;
      p->rgb.b = 40;
      DPRINTLN("TrafficLightGreen");
    };

    void setAmber(pixel *p) 
    {
      p->rgb.r = 255;
      p->rgb.g = 225;
      p->rgb.b = 0;
      DPRINTLN("TrafficLightAmber");
    };

    void setRed(pixel *p) 
    {
      p->rgb.r = 255;
      p->rgb.g = 0;
      p->rgb.b = 0;
      DPRINTLN("TrafficLightRed");
    };

    void setOff(pixel *p) 
    {
      p->rgb.r = 0;
      p->rgb.g = 0;
      p->rgb.b = 0;
      DPRINTLN("TrafficLightOff");
    };

};

}

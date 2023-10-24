namespace as {

template <class DEVTYPE>
class UserButton : public Button {
  DEVTYPE& device;
  
public:
  UserButton (DEVTYPE& dev, uint8_t longpresstime=3) : device(dev) {
    this->setLongPressTime(seconds2ticks(longpresstime));
  }
 
  virtual ~UserButton () {}
 
  virtual void state (uint8_t s) {
    uint8_t old = Button::state();
    Button::state(s);
    if( s == Button::released ) {
      bool tlOn = device.channel(1).toggleTrafficLight();
/*      if (tlOn) {
        device.led2.set(LedStates::welcome);  
      }
      else
      {
        device.led2.set(LedStates::send);
      } */
    }    
    else if( s == Button::longpressed ) {
    #if defined useSCD30
      bool fc = device.channel(1).forceCalibSCD30();
      if (fc==true) {
        DPRINTLN("SCD30: forced calibration done");       
        device.led2.set(LedStates::key_long);
      }
      else {
        DPRINTLN("SCD30: forced calibration FAILED");          
        device.led2.set(LedStates::failure);
      }
    #endif           
    #if defined useMHZ19
      bool fc = device.channel(1).forceCalibMHZ19();
      if (fc==true) {
        DPRINTLN("MHZ19: forced calibration done");       
        device.led2.set(LedStates::key_long);
      }
      else {
        DPRINTLN("MHZ19: forced calibration FAILED");          
        device.led2.set(LedStates::failure);
      }    
    #endif      
    }
  }
};

// define standard button switches to GND

// Redifinition of ConfigButton to comply with TTP223 touch pad. (HIGH at no touch)
// Can be avoided by using tiny soldering bridge on the TTP223 (then LOW at no touch)
// short press -> pairing, long press -> toogle traffic light

template <class DEVTYPE,uint8_t OFFSTATE=LOW,uint8_t ONSTATE=HIGH,WiringPinMode MODE=INPUT>
class myConfigButton : public StateButton<OFFSTATE,ONSTATE,MODE> {
  DEVTYPE& device;
public:
  typedef StateButton<OFFSTATE,ONSTATE,MODE> ButtonType;
  myConfigButton (DEVTYPE& dev,uint8_t longpresstime=3) : device(dev) {
    this->setLongPressTime(seconds2ticks(longpresstime));
  }
  virtual ~myConfigButton () {}
  virtual void state (uint8_t s) {
    uint8_t old = ButtonType::state();
    ButtonType::state(s);
    if( s == ButtonType::released ) {
      device.startPairing();
//      DPRINTLN("MyConfigButton: start pairing");       
    }
    else if( s == ButtonType::longpressed ) {
//      if( old == ButtonType::longpressed ) {
//        if( device.getList0().localResetDisable() == false ) {
      bool tlOn = device.channel(1).toggleTrafficLight();
//          device.reset(); // long pressed again - reset
//        }
//      }
//      else {
//       device.led().set(LedStates::key_long);
//      }
    }
  }
};

}

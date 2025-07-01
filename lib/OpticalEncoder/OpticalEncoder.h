class OpticalEncoder {

public:
  void setup(uint8_t pinNr, void (*ISR_callback)(void), int value,
             int ticksPerRotation);
  void handleInterrupt(void);
  long getSpeed(void);
  uint8_t getPin(void);
  int32_t getPosition(void);
  void resetPosition(void);
  bool getDirection(void);
  void setDirection(bool forward);

private:
  volatile long _encoder0Pos = 0;
  volatile bool _movingForward = true;
  uint8_t tach1Pin;

  long newposition = 0;
  long oldposition = 0;
  unsigned long newtime = 0;
  unsigned long oldtime = 0;
  long speed = 0;
  int nrTicks = 20;
  unsigned long prevmillis = 0;

  void (*ISR_callback)();
};

void OpticalEncoder::setup(uint8_t pinNr, void (*ISR_callback)(void), int value,
                           int ticksPerRotation) {
  tach1Pin = pinNr;
  nrTicks = ticksPerRotation;
  pinMode(tach1Pin, INPUT_PULLUP);

#ifdef digitalPinToInterrupt // Use macro defined in pins_arduino.h
  attachInterrupt(digitalPinToInterrupt(tach1Pin), ISR_callback, FALLING);
#elif defined(__AVR_ATmega32U4__) // Arduino Leonardo
  attachInterrupt(1, ISR_callback, FALLING); // pin 2
#else
  attachInterrupt(0, ISR_callback, FALLING); // pin 2
#endif
}

inline void OpticalEncoder::handleInterrupt(void) {
  unsigned long currmillis = millis();
  if (currmillis - prevmillis > 2) {
    if (_movingForward) {
      _encoder0Pos++;
    } else {
      _encoder0Pos--;
    }
  }
  prevmillis = currmillis;
}

uint8_t OpticalEncoder::getPin(void) { return tach1Pin; }

int32_t OpticalEncoder::getPosition() {
  // noInterrupts();
  newposition = _encoder0Pos;
  // interrupts();
  return newposition;
}

void OpticalEncoder::resetPosition() {
  noInterrupts();
  _encoder0Pos = 0;
  interrupts();
  newposition = 0;
  oldposition = 0;
}

bool OpticalEncoder::getDirection() { return _movingForward; }

void OpticalEncoder::setDirection(bool forward) { _movingForward = forward; }

long OpticalEncoder::getSpeed() {
  newtime = millis();
  speed = (abs(getPosition()) - abs(oldposition)) * 60000 /
          ((newtime - oldtime) *
           nrTicks); // 1000(to conver milli sec to sec)*60(to convert sec to
                     // minutes)/6(total number of pulses per rotation)=10000
                     // (pulses per milli sec to rpm)
  oldposition = newposition;
  oldtime = newtime;
  return speed;
}


#ifndef MyRotaryEncoder_h
#define MyRotaryEncoder_h

#include "Arduino.h"

class RotaryEncoder
{
public:

  // ----- Constructor -----
  RotaryEncoder(int cosPin, int sinPin, int zeroPin);

  // retrieve the current position
  long getPosition();

  void getPositionAndTime(long positionAndTime[2]);

  // simple retrieve of the direction the knob was rotated last time. 0 = No rotation, 1 = Clockwise, -1 = Counter Clockwise
  int getDirection();

  // adjust the current position
  void setPosition(long newPosition);

  // reset startime
  void resetClock();

// zero pin reader, to get the full round signal
  void fullRotationRead();

  // call this function on interupt to get the change in orientation
  void encoderReadStep();

  // Returns the time in milliseconds between the current observed
  unsigned long getMillisBetweenRotations() const;

  // Returns the RPM
  unsigned long getRPM();

private:
  int _cosPin, _sinPin, _zeroPin; // Arduino pins used for the encoder.

  volatile int _direction;
  volatile long _position;        // Internal position (4 times _positionExt)
  volatile long _positionPrev; // External position (used only for direction checking)

  unsigned long _positionTime;     // The time the last position change was detected.
  unsigned long _positionTimePrev; // The time the previous position change was detected.
  unsigned long _startTime; //The start of the measurments
};

#endif

// End

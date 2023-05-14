// -----
// RotaryEncoder.cpp - Library for using rotary encoders.
// This class is implemented for use with the Arduino environment.
//
// Copyright (c) by Matthias Hertel, http://www.mathertel.de
//
// This work is licensed under a BSD 3-Clause style license,
// https://www.mathertel.de/License.aspx.
//
// More information on: http://www.mathertel.de/Arduino
// -----
// Changelog: see RotaryEncoder.h
// -----

#include "MyRotaryEncoder.h"
#include <Arduino.h>
#include <SimplyAtomic.h>



// ----- Initialization and Default Values -----

RotaryEncoder::RotaryEncoder(int cosPin, int sinPin, int zeroPin)
{
  // Remember Hardware Setup
  _cosPin = cosPin;
  _sinPin = sinPin;
  _zeroPin = zeroPin;

  // Setup the input pins and turn on pullup resistor
  pinMode(_sinPin, INPUT);
  pinMode(_cosPin, INPUT_PULLUP);
  pinMode(_zeroPin, INPUT_PULLUP);

  // start with position 0;
  _position = 0;
  _positionPrev = 0;
  _direction = 0.0;
  _positionTime = 0.0;
  _startTime = 0.0;
} // RotaryEncoder()


long RotaryEncoder::getPosition()
{
  return _position;
} // getPosition()


void RotaryEncoder::getPositionAndTime(long positionAndTime[2])
{
  ATOMIC()
  {
    positionAndTime[0] = _position;
    positionAndTime[1] = _positionTime;
  }
  positionAndTime[1] -= _startTime;
} // getPositionAndTime()


int RotaryEncoder::getDirection()
{
  if(millis() - _positionTime>500){
    return 0;
  }else{
    return _direction;
  }
return -42;

}


void RotaryEncoder::setPosition(long newPosition)
{
    // only adjust the external part of the position.
    _position = newPosition;
    _positionPrev = newPosition;

} // setPosition()


 void RotaryEncoder::resetClock(){

    _startTime = millis();
    
 }


void RotaryEncoder::encoderReadStep(){

  if  (digitalRead(_sinPin) == LOW) {
    // clockwise rotation
    _positionTimePrev = _positionTime;
    _positionTime = millis();
    _direction = 1;
    _position++;

  } else {
    //counter-clockwise rotation
    _positionTimePrev = _positionTime;
    _positionTime = millis();
    _direction = -1;
    _position--;

  } 
}


void RotaryEncoder::fullRotationRead() {
  // detect pulse on zero channel
  _position = 0;
}

unsigned long RotaryEncoder::getMillisBetweenRotations() const
{
  return (_positionTime - _positionTimePrev);
}

unsigned long RotaryEncoder::getRPM()
{
  // calculate max of difference in time between last position changes or last change and now.
  unsigned long timeBetweenLastPositions = _positionTime - _positionTimePrev;
  unsigned long timeToLastPosition = millis() - _positionTime;
  unsigned long t = max(timeBetweenLastPositions, timeToLastPosition);
  //return 60000.0 / ((float)(t * 20));
  // encoder pulses/rev = 500
  //RPM = 60 000(ms/min) /(dt(ms)*500(pulse/rev));
  return 120.0/(float)(t);
}


// End

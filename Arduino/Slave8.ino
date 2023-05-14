#include <Wire.h>
#include <Arduino.h>
#include "myAccelStepper.h"
#include "MyRotaryEncoder.h"




// Global variables for stepper motor:
// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin 9
#define stepPin 11
#define motorInterfaceType 1

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

long moveToValue = 400000;
float acceleration = 0;
float MaxSpeed = 3000;

unsigned long startTime = 0;
//------------------------------------------------------------------------------



// Global variables for encoder:
const byte ZeroPin = 2;  // RS422 input for Z channel (zero detection)
const byte COSPin = 3;   // RS422 input for A channel (cos signal) 
const byte SINPin = 4;   // RS422 input for B channel (sin signal)  

int encoderState = 0;
int ENC_CLOCKWISE_ROTATION = 1;
int ENC_COUNTERCLOCKWISE_ROTATION = -1;
int ENC_STOP = 0;
bool hasStarted = false;
bool isAccelerating = false;

// Setup a RotaryEncoder with 2 steps per latch for the 2 signal input pins:
  RotaryEncoder encoder(COSPin, SINPin, ZeroPin);
//-----------------------------------------------------------------------------




void setup() {
  Serial.begin(9600);
  Wire.begin(0x08);
  Wire.onRequest(sendPositionAndTime);
  Wire.onReceive(recieveAccC0);
  // Set the maximum speed and acceleration:
  stepper.setMaxSpeed(MaxSpeed);
  stepper.setAcceleration(acceleration);
  stepper.moveTo(0);
  stepper.setCurrentPosition(0);
 
  // set 1st interrupt service routine to COSPin and 'RISING' edge 
  attachInterrupt(digitalPinToInterrupt(COSPin), onInterupt, RISING);

  // set 2nd interrupt service routine to ZeroPin and 'HIGH' level 
  // attachInterrupt(digitalPinToInterrupt(ZeroPin), onZeroInterupt, HIGH);
  
  Serial.print("Setup is done \n");
}





void loop() {
  
  if(isAccelerating){
    stepper.run();
  }else{
    stepper.runSpeedToPosition();
      //Serial.print("No acceleration \n");
  }

}




void sendPositionAndTime()
{
  long positionAndTime[2] = {0};
  char *char_ar = (char*)positionAndTime; //Create String
  
  encoder.getPositionAndTime(positionAndTime);// Get position and time of measurment
  Wire.write(char_ar,8); //Write two long to Pi.

}




void recieveAccC0(int numBytes){

//Set Up Vars
  float c0=0.0;
  char* c0Char = (char *)&c0;
  int count=0;

  //We'll recieve one byte at a time. Stop when none left
  while(Wire.available())
  {
    char c = Wire.read();    // receive a byte as character
    //Create float from the Byte Array
    *(c0Char+count) = c ;
    count++;
  }
  if(count != 4){
    Serial.print("Did not recieve 4 bytes\n");
  }else if(c0<0){
    stepper.moveToNoSpeedReCalc(-moveToValue);
    stepper.setC0(c0);
    isAccelerating = true;
  }else if(c0>0){
    stepper.moveToNoSpeedReCalc(moveToValue);
    stepper.setC0(c0);
    isAccelerating = true;
  }else{
    isAccelerating = false;
  }
  //Print the float out.
  //Serial.print("Received Number: "); 
  //Serial.println(c0);
  if(!hasStarted){
    encoder.resetClock();
    hasStarted = true;
  }
  //Serial.print("Accel was: ");
  //Serial.println(stepper.getAcceleration());
}




void onInterupt(){
  encoder.encoderReadStep();
}

//void onZeroInterupt(){
//  encoder.fullRotationRead();
//}

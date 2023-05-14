#include <AccelStepper.h>
#include <Stepper.h>
#include <Math.h>
#include "RotaryEncoder.h"

// Define constants for motor control
#define STEPS_PER_REVOLUTION 500
#define ACCELERATION 10
#define MAX_SPEED 2160

//Global variables for 
const int stepPin = 13;
const int dirPin = 12;
const int stepsPerRevolution = 500;  // change this to fit the number of steps per revolution
// for your motor
Stepper myStepper(stepsPerRevolution, 12,13);
volatile int step = 0;

// Pin definitions
const byte ZeroPin = 2;  // RS422 input for Z channel (zero detection)
const byte COSPin = 3;   // RS422 input for A channel (cos signal) 
const byte SINPin = 4;   // RS422 input for B channel (sin signal)   

// variables
enum {ENC_STOP, ENC_CLOCKWISE_ROTATION, ENC_COUNTERCLOCKWISE_ROTATION};   // encoder operation modes
volatile byte encoder_state = ENC_STOP;
volatile int encoder_position = 0; 
volatile int encoder_oldpos = 0;
volatile int runderP; 
volatile int runderN;
volatile float centimeterP;
volatile float centimeterN;


void   setup() {
  Serial.begin(115200); //Use serial monitor for debugging

  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  myStepper.setSpeed(2160); // sett hastigheten til RPM
  //accelerationstepper::setmaxspeed
  //attachInterrupt(digitalPinToInterrupt(2), stepMotor, CHANGE);

   // Define pins for input and output
  pinMode(SINPin, INPUT);

  // set   internal pullup resistor for interrupt pin
  pinMode(COSPin, INPUT_PULLUP);
  pinMode(ZeroPin, INPUT_PULLUP);
  
  
  // set 1st interrupt service   routine to COSPin and 'RISING' edge 
  attachInterrupt(digitalPinToInterrupt(COSPin), encoder_isr, RISING);

  // set 2nd interrupt service routine to ZeroPin and   'HIGH' level 
  attachInterrupt(digitalPinToInterrupt(ZeroPin), zero_detection_isr,   HIGH);
}


void loop() {
  //Prepare to take some steps
  digitalWrite(dirPin, HIGH);  // One direction
  //  Take 100 steps in one direction
  for (unsigned int i=0; i<40000; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(50);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(50);
    }
// Detect Encoder Stop
  if (encoder_oldpos == encoder_position) encoder_state = ENC_STOP;
  
  //Covert to length of movement
  centimeterP = ((runderP * 5.0)+(encoder_position/100.0))/10.0;
  
  // output encoder incremental and status
  Serial.print("Encoder position: ");
  Serial.print(centimeterP);
  Serial.print("cm");
  Serial.print(", Encoder state: ");
  
  if (encoder_state==ENC_CLOCKWISE_ROTATION)   {
    Serial.println("Clockwise Rotation");
        
  } else if (encoder_state==ENC_COUNTERCLOCKWISE_ROTATION)   {
    Serial.println("Counter-Clockwise Rotation");
    
  } else {
     Serial.println("Stop");  
  }
  runderP = 0;

  // Change direction
  digitalWrite(dirPin, LOW);  // The other direction
  //  Take 100 steps in the other direction
  for (unsigned int i=0; i<40000; i++) {    
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(50);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(50);
    }
  
  // Detect Encoder Stop
  if (encoder_oldpos == encoder_position) encoder_state= ENC_STOP;
  
  //Convert to length of movement
  centimeterN = ((runderN * 5.0)+(encoder_position/100.0))/10.0;
  
  // output encoder incremental   and status
  Serial.print("Encoder position: ");
  Serial.print(centimeterN);
  Serial.print("cm");
  Serial.print(", Encoder state: ");
  
  if (encoder_state==ENC_CLOCKWISE_ROTATION)   {
    Serial.println("Clockwise Rotation");
        
  } else if (encoder_state==ENC_COUNTERCLOCKWISE_ROTATION)   {
    Serial.println("Counter-Clockwise Rotation");
    
  } else {
     Serial.println("Stop");  
  }
  //To get the counter to count every round
  runderN = 0;
  
}

void encoder_isr() {
  
  if  (digitalRead(SINPin) == HIGH) {
    // clockwise rotation
    encoder_state=ENC_CLOCKWISE_ROTATION;
     encoder_position++;
     if(encoder_position == 499){
      runderP++;
     }
  } 
  else {
    //counter-clockwise rotation
    encoder_state=ENC_COUNTERCLOCKWISE_ROTATION;
     encoder_position++;
     if(encoder_position == 500){
      runderN++;
     } 
  }
}

void zero_detection_isr() {
   // detect pulse on zero channel
  encoder_position = 0;
}

void stepMotor() 
{
  myStepper.step(step);
  step++;
  if (step > stepsPerRevolution) {
    step = 0;
  }
}

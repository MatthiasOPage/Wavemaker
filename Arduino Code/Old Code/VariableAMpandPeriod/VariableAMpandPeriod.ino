// Bounce.pde
// -*- mode: C++ -*-
//
// Make a single stepper bounce from one limit to another
//
// Copyright (C) 2012 Mike McCauley
// $Id: Random.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $
#include <AccelStepper.h>

float scale = 13.5*10;
float Amp = 10;
float dualAmp = Amp*2;
float Period = 10;
float microSteps = 8;
float Position = round((scale*dualAmp)/microSteps);
float maxSpd = (Position/(Period/6))*0.975;
float maxAccel = maxSpd/(Period/6);
unsigned long startTime = millis();


// Define a stepper and the pins it will use
AccelStepper stepper(1, 3, 6); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
void setup()
{  
  Serial.begin(9600);
  Serial.println( Position );
  Serial.println( maxSpd );
  Serial.println( maxAccel );

  // Change these to suit your stepper if you want
  stepper.setMaxSpeed(maxSpd);
  stepper.setAcceleration(maxAccel);
  stepper.moveTo(Position);
}
void loop()
{
    
    // If at the end of travel go to the other end
    if (stepper.distanceToGo() == 0){
      stepper.moveTo(-stepper.currentPosition());
      Serial.println(millis());
    }
    stepper.run();

}

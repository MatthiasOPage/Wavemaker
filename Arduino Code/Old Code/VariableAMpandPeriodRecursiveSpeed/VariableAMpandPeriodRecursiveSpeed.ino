// Bounce.pde
// -*- mode: C++ -*-
//
// Make a single stepper bounce from one limit to another
//
// Copyright (C) 2012 Mike McCauley
// $Id: Random.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $
#include <AccelStepper.h>

double scale = 13.5*10;
double Amp = 5;
double dualAmp = Amp*2;
double Period = 1;
double microSteps = 8;
double Position = round((scale*dualAmp)/microSteps);
double timeSplit = Period/3;
double maxSpd = floor(dualAmp/timeSplit*scale);
double maxAccel = floor(Amp/(timeSplit*timeSplit)*scale);

double currentSpeed = maxSpd;
double currentAccel = maxAccel;
double timeScale = 0;
int i = 1;

double halfPeriodMillis = 0.5*(Period*1000);
unsigned long loopStartTime = 0;
unsigned long loopStopTime = 0;
unsigned long deltaTime = 0;


// Define a stepper and the pins it will use
AccelStepper stepper(1, 3, 6); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
void setup()
{  
  Serial.begin(9600);
  
  // Change these to suit your stepper if you want
  stepper.setMaxSpeed(maxSpd);
  stepper.setAcceleration(maxAccel);
  stepper.moveTo(Position);
}
void loop()
{
    // If at the end of travel go to the other end
    if (stepper.distanceToGo() == 0){
      loopStopTime = millis();
      deltaTime = loopStopTime-loopStartTime;
          if (i != 1 && abs(deltaTime-halfPeriodMillis) >= 2){
              timeScale = (deltaTime/halfPeriodMillis);
              currentSpeed = currentSpeed * timeScale;
              currentAccel = currentAccel * timeScale;
              stepper.setMaxSpeed(currentSpeed);
              stepper.setAcceleration(currentAccel);
              Serial.println("Calibrating...");
                if (abs(deltaTime-halfPeriodMillis) >= 2){
                  Serial.println("Calibration finished!");
                }
           }
       stepper.moveTo(-stepper.currentPosition());
       loopStartTime = millis();
    }
      stepper.run();

      i=2;

}

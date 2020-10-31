/* Encoder Library - TwoKnobs Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>
#include <AccelStepper.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder encoder(2, 3);
AccelStepper stepper(1, 5, 6);    
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.println("Knob Encoder Test:");
  
  stepper.setMaxSpeed(600.0);                                                                                             //Set Max Speed of Stepper (Slower to get better accuracy)
  stepper.setAcceleration(100.0); 
}

long positionRight = -999;

void loop() {
  long newRight;
  newRight = knobRight.read();
  if (newRight != positionRight) {
    Serial.print(", Right = ");
    Serial.print(newRight);
    Serial.println();
    positionRight = newRight;
  }
  if (stepper.currentPosition() != round(knobRight.read()/12)) {
    stepper.runToNewPosition((round(knobRight.read()/12)));
    Serial.println(stepper.currentPosition());
    Serial.println((round(knobRight.read()/12)));
  }
}

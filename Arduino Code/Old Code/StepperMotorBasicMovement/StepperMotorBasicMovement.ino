/*
  Stepper Motor Test
  stepper-test01.ino
  Uses MA860H or similar Stepper Driver Unit
  Has speed control & reverse switch
  
  DroneBot Workshop 2019
  https://dronebotworkshop.com
*/

// Defin pins
int driverPUL = 9;    // PUL- pin
int driverDIR = 3;    // DIR- pin
int spd = A0;     // Potentiometer

// Variables

int pd = 1000;       // Pulse Delay period
boolean setdir = HIGH; // Set Direction

// Interrupt Handler

void revmotor (){

  setdir = !setdir;
  
}


void setup() {

  pinMode (driverPUL, OUTPUT);
  pinMode (driverDIR, OUTPUT);
  digitalWrite(driverDIR,setdir);

}

void loop() {

    digitalWrite(driverPUL,HIGH);
   // delayMicroseconds(pd);
    digitalWrite(driverPUL,LOW);
    delayMicroseconds(pd);
//    if (pd >= 100); {
//      pd = 0.99999*pd;
//    }
//      Serial.print('pd');
//      Serial.println();
}

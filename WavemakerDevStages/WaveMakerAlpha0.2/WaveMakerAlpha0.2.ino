
#include <AccelStepper.h>
#include <Encoder.h>

#define top_home_switch 13                                                                                                     //Attach limit switch to pin 9
#define bottom_home_switch 12  
#define stepper_enable 7
#define run_switch 11

AccelStepper stepper(1, 5, 6);                                                                                            //Stepper 1 with 5-PUL and 6-DIR
Encoder rotary_encoder(2, 3);                                                                                             //Coder Defined on pins 2 and 3 (Hardware interupt pins)

long top_motorStep = 0;
long bottom_motorStep = 0;
long initial_homing=-1;                                                                                                   //Used to Home Stepper at startup
int top_position = 0;
int bottom_position = 0;
int current_position = 0;
int homing_delay = 1;
long ppc = 0;
long spc = 0;
long center_position = 0;
long center_step = 0;
long resolution = 50;

long limit_distance = 22.855;
long center_cm = 10;

void setup() {
  Serial.begin(9600);                                                                                                     //Open up the serial port with a 9600 baud rate
  pinMode(top_home_switch, INPUT_PULLUP);                                                                                     //Initilaizing the limit switch to port 9 as an input with pullup resistor
  pinMode(bottom_home_switch, INPUT_PULLUP);                                                                                     //Initilaizing the limit switch to port 9 as an input with pullup resistor
  pinMode(run_switch, INPUT_PULLUP);
  pinMode(stepper_enable, OUTPUT);
  delay(5);                                                                                                               //Wait a bit
  digitalWrite(stepper_enable, LOW);                                                                                      //Set Acceleration of Stepper
  Serial.print("Stepper is Homing . . . . . . . . . . . ");                                                               //Inform user that the homing process is being completed
  Serial.println(" ");                                                               //Inform user that the homing process is being completed


  stepper.setCurrentPosition(0);                                                                                          //Set the current position as zero for now
  stepper.setMaxSpeed(500.0);                                                                                             //Set Max Speed of Stepper (Slower to get better accuracy)
  stepper.setAcceleration(500.0);                                                                                         //Set Acceleration of Stepper

    while (digitalRead(top_home_switch)) {                                                                                    //Make the Stepper move CW until the switch is activated   
    stepper.moveTo(initial_homing); 
    stepper.runSpeed();                                                                                                        //Set the position to move to
    initial_homing++;                                                                                                     //Decrease by 1 for next move if needed                                                                                                        //Start moving the stepper
    delay(homing_delay);
}
  stepper.setCurrentPosition(0); //Set current position as origin
  rotary_encoder.write(0);
  top_motorStep = rotary_encoder.read();
  initial_homing=1;         

  
    while (digitalRead(bottom_home_switch)) {                                                                                    //Make the Stepper move CCW until the switch is activated   
    stepper.moveTo(initial_homing); 
    stepper.runSpeed();                                                                                                        //Set the position to move to
    initial_homing--;                                                                                                     //Decrease by 1 for next move if needed                                                                                                        //Start moving the stepper
    delay(homing_delay);
}
  bottom_motorStep = stepper.currentPosition();
  bottom_position = rotary_encoder.read();
  initial_homing=0; 
  
  ppc = bottom_position/limit_distance;
  spc = bottom_motorStep/limit_distance;
  center_position = top_position + ppc*center_cm;
  center_step = spc*center_cm;
  current_position = rotary_encoder.read();
      
Serial.print("Encoder Top Position: ");
Serial.println(top_position);
Serial.print("Encoder Bottom Position: ");
Serial.println(bottom_position);

Serial.print("Motor Top Position: ");
Serial.println(top_motorStep);
Serial.print("Motor Bottom Position: ");
Serial.println(bottom_motorStep);

Serial.print("Pulses/cm: ");
Serial.println(ppc);
Serial.print("Steps/cm: ");
Serial.println(spc);

    while (abs(current_position - center_position) > resolution) {                                                                                    //Make the Stepper move CW until the switch is activated   
    stepper.moveTo(initial_homing); 
    stepper.setSpeed(100.00);
    stepper.run();                                                                                                       //Set the position to move to
    initial_homing++;                                                                                                     //Decrease by 1 for next move if needed                                                                                                        //Start moving the stepper
    delay(homing_delay);
    current_position = rotary_encoder.read();
    }
    
Serial.print("Homing Complete.");

}
void loop() {
  if(digitalRead(run_switch)==LOW && digitalRead(top_home_switch) ==LOW && digitalRead(bottom_home_switch) ==LOW{
    
  }
{
}

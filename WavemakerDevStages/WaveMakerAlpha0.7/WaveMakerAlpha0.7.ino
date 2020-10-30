
#include <AccelStepper.h>
#include <Encoder.h>

#define top_home_switch 13                                                                                                     //Attach limit switch to pin 9
#define bottom_home_switch 12  
#define stepper_enable 7
#define run_switch 11

AccelStepper stepper(1, 5, 6);                                                                                            //Stepper 1 with 5-PUL and 6-DIR
Encoder rotary_encoder(2, 3);                                                                                             //Coder Defined on pins 2 and 3 (Hardware interupt pins)

double top_motorStep = 0;
double bottom_motorStep = 0;
double initial_homing=-1;                                                                                                   //Used to Home Stepper at startup
double top_position = 0;
double bottom_position = 0;
double current_position = 0;
double homing_delay = 1;
double ppc = 0;
double spc = 0;
double center_position = 0;
double center_step = 0;
double position_resolution = 25;
double step_resolution = 25;
double period_resolution = 25;
double amplitude_step = 0;
double stpR = 0;
double timeSplit = 0;                                                                                          //A ratio precomputed for efficiency
double maxSpd = 0;                                                //Setting stepper speed in steps/second
double maxAccel = 0;
double currentSpeed = 0;                                                                                             //Initializing a variable for recursive speed calibration
double currentAccel = 0;                                                                                           //Initializing a variable for recursive acceleration calibration
double timeScale = 0;                                                                                                     //Initializing timescale
int i = 1;
int iteration_max = 30;//Simple boolean integer to skip first oscillation
double halfPeriodMillis = 0;                                                                              //Target halfperiod in milliseconds
unsigned long loopStartTime = 0;                                                                                          //Variable used to store when the movement starts
unsigned long loopStopTime = 0;                                                                                           //Variable used to store when the movement ends
unsigned long deltaTime = 0;                                                                                              //Variable used to store time difference
bool Calibration = false;
double LRC = 1.3;

double limit_distance = 22.855;
double center_cm = 15;
double amplitude = 5;
double period = 0.65;
double ratio = 0.3333333;




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
  stepper.setMaxSpeed(100000.0);                                                                                             //Set Max Speed of Stepper (Slower to get better accuracy)
  stepper.setAcceleration(100000.0);                                                                                         //Set Acceleration of Stepper

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
  stpR = bottom_motorStep/bottom_position;
  
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
Serial.print("Steps to Pulse Ratio: ");
Serial.println(stpR);

    while (abs(current_position - center_position) > position_resolution) {                                                                                    //Make the Stepper move CW until the switch is activated   
    stepper.moveTo(initial_homing); 
    stepper.setSpeed(500.00);
    stepper.run();                                                                                                       //Set the position to move to
    initial_homing++;                                                                                                     //Decrease by 1 for next move if needed                                                                                                        //Start moving the stepper
    delay(homing_delay);
    current_position = rotary_encoder.read();
    }
    
  stepper.setCurrentPosition(0);
  rotary_encoder.write(0);
  amplitude_step = spc*amplitude;

//----------------------------------------------------------------CALIBRATION--------------------------------------------------------------------------------------\\
                                                                                              //A ratio precomputed for efficiency
currentSpeed = abs(ceil(LRC*((4*amplitude_step)/period)));                                                                                             //Initializing a variable for recursive speed calibration
currentAccel = abs(ceil(LRC*((9*amplitude_step)/(ratio*(period*period)))));  
halfPeriodMillis = 0.5*period*1000;//Setting stepper acceleration in steps/(second)^2
Serial.println("Half Period: ");
Serial.println(halfPeriodMillis);

i = 1;

Serial.print("Speed: ");
Serial.println(currentSpeed);
Serial.print("Accel: ");
Serial.println(currentAccel);

digitalWrite(stepper_enable, HIGH);
  stepper.setMaxSpeed(currentSpeed);                                                                                            //Setting the initial stepper target speed
  stepper.setAcceleration(currentAccel);                                                                                      //Setting the initial stepper acceleration
  stepper.moveTo(amplitude_step);                                                                                               //Start the moving process
Serial.print("Calibrating.... ");

  while(Calibration == false) {
      if (stepper.distanceToGo() == 0){                                                                                     //If at the end of the movement
        loopStopTime = millis();  
       //When the movement is stopping
        deltaTime = loopStopTime-loopStartTime;                                                                             //Calculating the time taken barring the first loop
          if (i != 1 && abs(deltaTime-halfPeriodMillis) > period_resolution && i<= iteration_max ){                                                            //If not on the first loop and not accurate period
              timeScale = (deltaTime/halfPeriodMillis);                                                                   //Calculating ratio of actual time to target time
              currentSpeed = (currentSpeed * timeScale);    
              Serial.print("Iteration: ");
              Serial.println(i);
              currentAccel = (currentAccel * timeScale);                                                                    //Proportionally scaling the steppers acceleration
              stepper.setMaxSpeed(currentSpeed);                                                                          //Changing the steppers speed
              stepper.setAcceleration(currentAccel);                                                                      //Changing the steppers accceleration
           }
           else if (i > iteration_max){
            Serial.print(" Calibration Failed ");
            Serial.print(" Please increase period and/or reduce amplitude." );
            Serial.print("Minimum Period: ");
            Serial.println(deltaTime*2);
           }
        loopStartTime = millis();                                                                                          //Store when the movement process starts                  
        stepper.moveTo(-stepper.currentPosition());   //Change target position to the opposite direction
        i++;     //Tell the program that the first period is completed
       
    }
    
      if (i != 1 && abs(deltaTime-halfPeriodMillis) <= period_resolution){
        Serial.print(" Calibration Complete ");
        Calibration = true;
      }
      
      stepper.run();                                                                                                      //Make the Stepper move
  }

  stepper.setMaxSpeed(currentSpeed);                                                                                            //Setting the calibrated stepper speed
  stepper.setAcceleration(currentAccel);                                                                                        //Setting the calibrated stepper acceleration
  stepper.runToNewPosition(0);
  digitalWrite(stepper_enable, LOW);
  delay(5);
  Serial.println("Push toggle switch to begin.");
}
void loop() {
  if(digitalRead(run_switch)==LOW && digitalRead(top_home_switch) && digitalRead(bottom_home_switch)){
    if(abs(amplitude_step - stepper.currentPosition())> step_resolution){
    stepper.runToNewPosition(amplitude_step);
    stepper.setCurrentPosition(stpR*rotary_encoder.read());
    } else{
      Serial.println( millis()/1000);
      Serial.println( rotary_encoder.read()/ppc);
      amplitude_step = -amplitude_step;
    }
  }
}

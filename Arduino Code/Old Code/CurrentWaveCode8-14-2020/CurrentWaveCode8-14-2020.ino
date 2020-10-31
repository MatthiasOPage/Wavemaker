//Wavemaker code heavily based on the accel stepper bounce example

#include <AccelStepper.h>

#define home_switch 13                                                                                                     //Attach limit switch to pin 9
#define stepper_enable 7
#define run_switch 11

int move_finished=1;                                                                                                      //Used to check if move is completed
long initial_homing=-1;                                                                                                   //Used to Home Stepper at startup

//List of input values necessary for wavemaker
double scale = 13.5*10;                                                                                                   //Ratio of steps to centimeters
double Amp = 8;                                                                                                          //Wavemaker amplitude in cm
double Period = 5;                                                                                                    //Wave period in seconds
double microSteps = 4;                                                                                                    //Microstep setting on stepper driver
double Center = 80;                                                                                                       //Center position in cm down from home

//Do the maths to integrate the stepper library and waveform math
double Position = round((scale*Amp)/microSteps);                                                                          //Translating target amplitude to stepper position
double centerPosition = -round((scale*Center)/microSteps);                                                                //Translating home center to stepper position
double timeSplit = Period/3;                                                                                              //A ratio precomputed for efficiency
double maxSpd = floor((2*Amp)/timeSplit*scale);                                                                           //Setting stepper speed in steps/second
double maxAccel = floor((2*Amp)/(timeSplit*timeSplit)*scale);                                                             //Setting stepper acceleration in steps/(second)^2

//Variable intilization
double currentSpeed = maxSpd;                                                                                             //Initializing a variable for recursive speed calibration
double currentAccel = maxAccel;                                                                                           //Initializing a variable for recursive acceleration calibration
double timeScale = 0;                                                                                                     //Initializing timescale
int i = 1;                                                                                                                //Simple boolean integer to skip first oscillation
double halfPeriodMillis = 0.5*(Period*1000);                                                                              //Target halfperiod in milliseconds
unsigned long loopStartTime = 0;                                                                                          //Variable used to store when the movement starts
unsigned long loopStopTime = 0;                                                                                           //Variable used to store when the movement ends
unsigned long deltaTime = 0;                                                                                              //Variable used to store time difference
bool Calibration = false;


//Defining the stepper
AccelStepper stepper(1, 5, 6);                                                                                            //Stepper 1 with 5-PUL and 6-DIR

//Setting up serial and stepper settings
void setup()
{  
  Serial.begin(9600);                                                                                                     //Open up the serial port with a 9600 baud rate
  pinMode(home_switch, INPUT_PULLUP);                                                                                     //Initilaizing the limit switch to port 9 as an input with pullup resistor
  pinMode(run_switch, INPUT_PULLUP);
  pinMode(stepper_enable, OUTPUT);
  delay(5);                                                                                                               //Wait a bit
  digitalWrite(stepper_enable, LOW);
  // Start Homing procedure of Stepper Motor at startup
  stepper.setMaxSpeed(1000.0);                                                                                             //Set Max Speed of Stepper (Slower to get better accuracy)
  stepper.setAcceleration(1000.0);                                                                                         //Set Acceleration of Stepper
  Serial.print("Stepper is Homing . . . . . . . . . . . ");                                                               //Inform user that the homing process is being completed

  while (digitalRead(home_switch)) {                                                                                      //Make the Stepper move CW until the switch is activated   
    stepper.moveTo(initial_homing);                                                                                       //Set the position to move to
    initial_homing++;                                                                                                     //Decrease by 1 for next move if needed
    stepper.run();                                                                                                        //Start moving the stepper
    delay(5);
}

  stepper.setCurrentPosition(0);                                                                                          //Set the current position as zero for now
  stepper.setMaxSpeed(100.0);                                                                                             //Set Max Speed of Stepper (Slower to get better accuracy)
  stepper.setAcceleration(100.0);                                                                                         //Set Acceleration of Stepper
  initial_homing=1;                                                                                                       //Reset intial homing variable

  while (!digitalRead(home_switch)) {                                                                                     //Make the Stepper move CCW until the switch is deactivated
    stepper.moveTo(initial_homing);                                                                                       //Set the position to move to
    stepper.run();                                                                                                        //Start moving the stepper
    initial_homing--;                                                                                                     //Decrease by 1 for next move if needed
    delay(5);
  }
  
  stepper.setCurrentPosition(0); //Set current position as origin
  delay(5);
  stepper.runToNewPosition(centerPosition);                                                                               //Move to the desired origin
  stepper.setCurrentPosition(0);                                                                                          //Set center as new origin
  
  Serial.println("Homing Completed");                                                                                     //Inform user that homing process is completed
  Serial.println("");
  
  digitalWrite(stepper_enable, HIGH);
  stepper.setMaxSpeed(maxSpd);                                                                                            //Setting the initial stepper target speed
  stepper.setAcceleration(maxAccel);                                                                                      //Setting the initial stepper acceleration
  stepper.moveTo(Position);                                                                                               //Start the moving process

  while(Calibration == false) {;
      if (stepper.distanceToGo() == 0){                                                                                     //If at the end of the movement
        loopStopTime = millis();                                                                                            //When the movement is stopping
        deltaTime = loopStopTime-loopStartTime;                                                                             //Calculating the time taken barring the first loop
          if (i != 1 && abs(deltaTime-halfPeriodMillis) > 2){                                                            //If not on the first loop and not accurate period
              timeScale = (deltaTime/halfPeriodMillis);                                                                   //Calculating ratio of actual time to target time
              currentSpeed = currentSpeed * timeScale;                                                                    //Proportionally scaling the steppers speed
              Serial.println( timeScale);
              currentAccel = currentAccel * timeScale;                                                                    //Proportionally scaling the steppers acceleration
              stepper.setMaxSpeed(currentSpeed);                                                                          //Changing the steppers speed
              stepper.setAcceleration(currentAccel);                                                                      //Changing the steppers accceleration
              Serial.println("Current Speed");                                                                           //Let the user know the movement is still being adjusted
              Serial.println(currentSpeed); 
              
           }
        loopStartTime = millis();                                                                                          //Store when the movement process starts                  
        stepper.moveTo(-stepper.currentPosition());                                                                        //Change target position to the opposite direction
        i=2;                                                                                                               //Tell the program that the first period is completed
       
    }
    
      if (i != 1 && abs(deltaTime-halfPeriodMillis) <= 2){
        Calibration = true;
      }
      
      stepper.run();                                                                                                      //Make the Stepper move
  }

  Serial.println( "Calibration Complete");
  stepper.setMaxSpeed(currentSpeed);                                                                                            //Setting the calibrated stepper speed
  stepper.setAcceleration(currentAccel);                                                                                        //Setting the calibrated stepper acceleration
  digitalWrite(stepper_enable, LOW);
  
}


void loop()
{
  if(digitalRead(run_switch)==LOW){
    if (stepper.distanceToGo() == 0)
      stepper.moveTo(-stepper.currentPosition());
    stepper.run();
  }
}

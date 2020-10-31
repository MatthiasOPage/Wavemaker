//------------------------------------------------------------------------------------Wavemaker Software Version 1.0----------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//


//---------------------------------------------------------------------------------------------------Initialization-----------------------------------------------------------------------------------//

////Wave Properties//// [Note: T_min = 0.15A]
  double center_cm = 18; //[cm from top limit to center of movement]
  double amplitude = 1;  //[cm]
  double period = 0.25;  //[s]

////Wavemaker Properties////
  double limit_distance = 22.855;

////Tolerance Properties////
  double position_resolution = 25;
  double step_resolution = 25;
  double period_resolution = 10;

////Calibration Properties////
  double homing_delay = 1;
  double homing_speed = 100000;
  double homing_accel = 100000;
  int iteration_max = 30;
  double ratio = 0.3333333;

////Pin Definitions////
  #define top_home_switch 13
  #define bottom_home_switch 12
  #define stepper_enable 7
  #define run_switch 11

////Library Initialization////
  #include <AccelStepper.h>
  #include <Encoder.h>

////Hardware Declerations////
  AccelStepper stepper(1, 5, 6);
  Encoder rotary_encoder(2, 3);

////Variable Pre-Initialization////
  double top_motorStep = 0;
  double bottom_motorStep = 0;
  double initial_homing = -1;
  double top_position = 0;
  double bottom_position = 0;
  double current_position = 0;
  double ppc = 0;
  double spc = 0;
  double center_position = 0;
  double center_step = 0;
  double amplitude_step = 0;
  double stpR = 0;
  double currentSpeed = 0;
  double currentAccel = 0;
  double timeScale = 0;
  int i = 1;
  double halfPeriodMillis = 0;
  unsigned long loopStartTime = 0;
  unsigned long loopStopTime = 0; 
  unsigned long deltaTime = 0;
  bool Calibration = false;

//---------------------------------------------------------------------------------------------------SETUP--------------------------------------------------------------------------------------------//
void setup() {

////Serial and Pin Setup////
  Serial.begin(9600);
  pinMode(top_home_switch, INPUT_PULLUP);
  pinMode(bottom_home_switch, INPUT_PULLUP); 
  pinMode(run_switch, INPUT_PULLUP);
  pinMode(stepper_enable, OUTPUT);
  delay(5);

//---------------------------------------------------------------------------------------------------HOMING-------------------------------------------------------------------------------------------//
////Enable Stepper and Begin Homing Sequence////
  digitalWrite(stepper_enable, LOW);
  Serial.print("Stepper is Homing . . .  ");
  Serial.println(" ");
  stepper.setCurrentPosition(0);
  stepper.setMaxSpeed(homing_speed);
  stepper.setAcceleration(homing_accel);

////Move to Top Limit and set Origin////
  while (digitalRead(top_home_switch)) {
    stepper.moveTo(initial_homing);
    stepper.runSpeed();
    initial_homing++;
    delay(homing_delay);
  }
  
  stepper.setCurrentPosition(0);
  rotary_encoder.write(0);
  top_motorStep = rotary_encoder.read();
  initial_homing = 1;

////Move to Bottom Limit////
  while (digitalRead(bottom_home_switch)){
    stepper.moveTo(initial_homing);
    stepper.runSpeed();
    initial_homing--;
    delay(homing_delay);
  }
  
  bottom_motorStep = stepper.currentPosition();
  bottom_position = rotary_encoder.read();
  initial_homing = 0;

////Calculate and Display Ratios and Magnitudes of Homing Properties////
  ppc = bottom_position / limit_distance;
  spc = bottom_motorStep / limit_distance;
  center_position = top_position + ppc * center_cm;
  center_step = spc * center_cm;
  current_position = rotary_encoder.read();
  stpR = bottom_motorStep / bottom_position;
  
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

////Move to Target Center and Reset Origin////
  while (abs(current_position - center_position) > position_resolution) {
    stepper.moveTo(initial_homing);
    stepper.setSpeed(500.00);
    stepper.run(); 
    initial_homing++;
    delay(homing_delay);
    current_position = rotary_encoder.read();
  }
  
  stepper.setCurrentPosition(0);
  rotary_encoder.write(0);
  amplitude_step = spc * amplitude;

//---------------------------------------------------------------------------------------------------Calibration----------------------------------------------------------------------------------------//
////Calculate Kinematic Velocity, Acceleration and Half-Period////
  currentSpeed = abs(ceil((6 * amplitude_step) / period));
  currentAccel = abs(ceil((18 * amplitude_step) / (ratio * (period * period))));
  halfPeriodMillis = 0.5 * period * 1000;
  
////Disable Stepper and Set Initial Parameters////
  digitalWrite(stepper_enable, HIGH);
  stepper.setMaxSpeed(currentSpeed);
  stepper.setAcceleration(currentAccel);
  stepper.moveTo(amplitude_step);
  Serial.print("Calibrating... ");

////Simulate Movement and Correct Period Error Until Within Threshold or Failure////
  while (Calibration == false) {
    if (stepper.distanceToGo() == 0){
      loopStopTime = millis();
      deltaTime = loopStopTime - loopStartTime;
      if (i != 1 && abs(deltaTime - halfPeriodMillis) > period_resolution && i <= iteration_max ) {
        timeScale = (deltaTime / halfPeriodMillis);
        currentSpeed = (currentSpeed * timeScale);
        currentAccel = (currentAccel * timeScale);
        Serial.print("Iteration: ");
        Serial.println(i);
        stepper.setMaxSpeed(currentSpeed);
        stepper.setAcceleration(currentAccel);
      }
      else if (i > iteration_max) {
        Serial.print(" Calibration Failed ");
        Serial.print(" Please increase period and/or reduce amplitude." );
        Serial.print("Minimum Period: ");
        Serial.println(deltaTime * 2 - 0.01);
      }
      loopStartTime = millis();
      stepper.moveTo(-stepper.currentPosition());
      i++;
    }
    if (i != 1 && abs(deltaTime - halfPeriodMillis) <= period_resolution) {
      Serial.print(" Calibration Complete ");
      Calibration = true;
    }
    stepper.run();
  }

////Set Calibrated Stepper Velocity and Acceleration and Prepare to Execute////
  stepper.setMaxSpeed(currentSpeed);
  stepper.setAcceleration(currentAccel);
  stepper.runToNewPosition(0);
  digitalWrite(stepper_enable, LOW);
  delay(5);
  Serial.println("Push toggle switch to begin.");
  Serial.println(currentSpeed);
}

//---------------------------------------------------------------------------------------------------EXCECUTE-----------------------------------------------------------------------------------------//
void loop() {

////Check for Edgecases////
  if (digitalRead(run_switch) == LOW && digitalRead(top_home_switch) && digitalRead(bottom_home_switch)) {

////Complete Motion Repeatedly While Compensating for Skipped Steps////
  if (abs(amplitude_step - stepper.currentPosition()) > step_resolution) {
      stepper.runToNewPosition(amplitude_step);
      stepper.setCurrentPosition(stpR * rotary_encoder.read());
  } 
  else {
      Serial.println( millis());
      Serial.println( rotary_encoder.read() / ppc);
      amplitude_step = -amplitude_step;
  }
 }
}

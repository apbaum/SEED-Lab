// Demo 1 Robot Localization and Control V4
// ================
// Authors: Madeleine Houghton and Quinn Hejmanowski
// Date: 3/8/2024
// ================
// The purpose of this code is to design a controller
// to move a robot to a desired distance and angle.
// The code uses a given distance and given angle
// (altered directly in the program) to accurately 
// maneuver the robot.

// DISCLAIMER:
// This program assumes that the assembly of a robot
// structure, and wiring for the motors to the Arduino and
// motor driver shield have been completed. Refer to Resources
// for additional info on the motor and motor driver shield
// wiring configurations. The hardware configuration mentioned
// here are the minimum requirements to use the program
// as intended. 

// Hardware Required
// ----------------
//    x1 Arduino Uno Board
//    x1 Motor Driver Shield (compatible with Arduino)
//    x2 Motors with mounted wheels
//    x1 Mounted wheel (no motor) for turning
//    x1 Voltage Regulator
//    x1 Battery Pack (~7.8 V) with fuse connector

// Hardware Configurations
// ----------------
//    Ensure the battery pack is connected to the corresponding
//    - and + terminals on the input to the voltage regulator.
//    Ensure that the output terminals of the voltage regulator
//    correspond to the correct - and + terminals on the motor
//    power inputs of the motor driver shield.
//    Install the motor driver shield onto the Arduino.
//    Make the necessary connections of the motor wires to the
//    motor driver shield (refer to driver shield website).

// How to Use Code
// ----------------
//    Download and open code in Arduino sketch in Arduino IDE.
//    Upload the code from the IDE to the Arduino board.

// Resources
// ----------------
//  Motor Wire Connections: https://www.pololu.com/docs/0J55
//  Motor Driver Shield: https://www.pololu.com/product/2824

// ================
// CODE BEGINS HERE
// ================

// Assigns timing variables for sampling the motor velocities.
unsigned long desired_Ts_ms = 10;
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time;
bool startMove = false;

// These variables keep track of the actual speed on the motors.
float actual_speed[2] = {0, 0};

// Assigns variables to keep track of the previous positions on
// the encoders.
float prev_pos[2] = {0, 0};

// This holds calculated values for the controller to aim towards
// a desired velocity for the angle and position.
float desiredAngleVel = 0;
float desiredDistVel = 0;

// These variables are the user input of the target distance
// and velocity of the robot. These variables should not change
// or be altered during program execution.
const float TARGET_DISTANCE = 3;
const float TARGET_ANGLE_DEG = 0;

// These parameters are used for the desired distance and angle.
// The variables change since the input is simulated as a ramp
// function.
float desiredDistance = 0;
float desiredAngle = 0;

// Sets the target speed of the motors.
float integralAngleError = 0;
float integralDistError = 0;

// Sets the distance and angle error of the robot. Also
// sets the robot velocity and rotational velocuty errors.
float distanceError = 0;
float angleError = 0;
float angleVelError = 0;
float distanceVelError = 0;

// Keeps track of the position of both motors in radians.
long pos_counts[2] = {0, 0};
float actual_pos[2] = {0, 0};

// Sets motor pins and defines them in an array. Also determines
// the voltage signs, motor voltage, and motor channels. This is
// done to monitor the direction and speed on each motor.
#define NUM_MOTOR_PINS 5
int MotorPinArray[NUM_MOTOR_PINS] = {4, 7, 8, 9, 10};
int Motor1Chan[2] = {2, 5};
int Motor2Chan[2] = {3, 6};
int VoltageSign[2] = {7,8};
int MotorVoltage[2] = {9,10};

// Initializes counting variables on the respective encoders.
long counts[2] = {0, 0};

// Initializes global variables for debouncing in the ISRs.
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 100;

// Rotation and forward movement variables
float robotAngle = 0;
float prevRobotAngle = 0;
float robotDistance = 0;
float prevRobotDistance = 0;
float instantRho = 0;
float instantPhi = 0;

// Variables to store change in distance and angle of robot.
float currentdl[2] = {0,0};
float lastdl[2] = {0,0};

// Width of the robot and radius of the wheel. Both are in feet.
float width = 0.9479;
float radius = 0.26365;

// Converts encoder counts to feet.
double encClicksToFeet = 2*PI*radius/3200;

// Defines variables for average voltage, difference voltage, voltage
// to store in the motors, and PWM to write to the motors.
float Voltage[2] = {0, 0};
unsigned int PWM[2] = {0, 0};
float avgVolt = 0;
float difVolt = 0;

// Defines the Interrupt Service Routine for changes on channel A and 
// determining the rotation on the encoder. This ISR is for motor 1.
void countEncoder1() {
  // This condition decreases the likelihood of the ISR activating 
  // during a bounce in the encoder channel transitions.
  if (micros() - lastDebounceTime > debounceDelay) {

    // Counts up twice for a clockwise rotation on the encoder.
    if (digitalRead(Motor1Chan[0]) == digitalRead(Motor1Chan[1])){
      counts[0] += 2;
    }
    // Counts down twice for a counter-clockwise rotation on encoder.
    else {
      counts[0] -= 2;
    }

    // Updates the last time this ISR was called.
    lastDebounceTime = micros();
  }
}

// Defines the Interrupt Service Routine for changes on channel A and 
// determining the rotation on the encoder. This ISR is for motor 2.
void countEncoder2() {
  // This condition decreases the likelihood of the ISR activating 
  // during a bounce in the encoder channel transitions.
  if (micros() - lastDebounceTime > debounceDelay) {

    // Counts up twice for a clockwise rotation on the encoder.
    if (digitalRead(Motor2Chan[0]) == digitalRead(Motor2Chan[1])){
      counts[1] += 2;
    }
    // Counts down twice for a counter-clockwise rotation on encoder.
    else {
      counts[1] -= 2;
    }

    // Updates the last time the ISR was called.
    lastDebounceTime = micros();
  }
}

// Initializes pins, ISRs, and gives variables starting values before
// running the motors and collecting data.
void setup() {
  // Initializes motor pins on the motor drive shield.
  for (int i = 0; i < NUM_MOTOR_PINS; i++) {
    pinMode(MotorPinArray[i], OUTPUT);
  }

  // Initializes the encoder pins on motors 1 and 2.
  for (int j = 0; j < 2; j++) {
    pinMode(Motor1Chan[j], INPUT_PULLUP);
    pinMode(Motor2Chan[j], INPUT_PULLUP);
  }

  // Initializes ISRs for motors 1 and 2. The ISR will
  // activate when a change on channel A occurs on their
  // respective motor.
  attachInterrupt(digitalPinToInterrupt(Motor1Chan[0]), countEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Motor2Chan[0]), countEncoder2, CHANGE);

  // Enables set up pin for motor drive shield.
  digitalWrite(4, HIGH);

  // Sets up monitoring the data collected from the program.
  Serial.begin(115200);

  // Initializes the times for the program.
  last_time_ms = millis();
  start_time_ms = last_time_ms;
  desiredAngle = 0;
  desiredDistance = 0;
}

// Runs the robot motor controller to have the robot move a desired distance and angle.
void loop() {  
  // These variables are the gains for the angle controller.
  float Kp_angle = 15;
  float Ki_angle= 11;
  float angleVelKp = 4.5;
  float maxAngleVel = 4;

  // These variables are the gains for the distance controller.
  float Kp_distance = 18.5;
  float Ki_distance = 4;
  float distanceVelKp = 4;
  float maxDistVel = 2.4;

  float Battery_Voltage = 7.8;

  // Sets the input for how much time to spend on ramp rise.
  float riseTimeAngle = 2.5;
  float riseTimeDist = 5;

  // Corrects angle rise time if the angle is more than 180 degrees.
  if (abs(TARGET_ANGLE_DEG) > 180) {
    riseTimeAngle = 7;
  }

  // Corrects distance rise time if the distance is more than 5 feet.
  /*if (abs(TARGET_DISTANCE > 5)) {
    riseTimeDist = 7;
  }*/

  // Variables to calculate the ramp function input rises.
  float distanceRise = TARGET_DISTANCE/(riseTimeDist * 1000) * desired_Ts_ms;
  float angleRise = TARGET_ANGLE_DEG*PI/180/(riseTimeAngle * 1000) * desired_Ts_ms;

  // Converts the encoder counts to feet.
  currentdl[0] = (double)myEnc(1) * encClicksToFeet;
  currentdl[1] = (double)myEnc(2) * encClicksToFeet;

  // Updates last time program ran.
  last_time_ms = millis();

  // Calculates the current time of the program.
  current_time = (float)(last_time_ms - start_time_ms)/1000;
  
  // Calculates the robot distance and angle.
  // Distance is in feet and angle is in radians.
  robotDistance = prevRobotDistance + (currentdl[0] - lastdl[0] + currentdl[1] - lastdl[1])/2;
  robotAngle = prevRobotAngle - (currentdl[0] - lastdl[0] - currentdl[1] + lastdl[1])/width;
  
  // Updates previous variables for distance and angle.
  prevRobotDistance = robotDistance;
  prevRobotAngle = robotAngle;

  // Updates last changes in encoder.
  lastdl[0] = currentdl[0];
  lastdl[1] = currentdl[1];

  // Simulates a ramp input for the desired distance. For the distance, the input starts after
  // 4 seconds and rises to the target distance at 6 seconds.
  if (startMove == true) {
    // Rise of ramp function.
    desiredDistance = desiredDistance + distanceRise;
    // End of ramp function.
    if (desiredDistance >= TARGET_DISTANCE) {
      desiredDistance = TARGET_DISTANCE;
    }
  }

  // Simulates a ramp input for the desired angle. For the angle, the input starts after 0.25
  // seconds and rises to the target angle at 1.5 seconds.
  // Rise of ramp function.
  if (current_time < riseTimeAngle) {
    desiredAngle = desiredAngle + angleRise;
  }
  // End of ramp function.
  else {
    desiredAngle = TARGET_ANGLE_DEG*PI/180;
  }

  // Calculates the phi error (angle) and rho error (distance).
  distanceError = desiredDistance - robotDistance;
  angleError = desiredAngle - robotAngle;

  // Calculates the desired angular velocity.
  integralAngleError = integralAngleError + angleError*((float)desired_Ts_ms / 1000);
  desiredAngleVel = Kp_angle * angleError + Ki_angle * integralAngleError;

  // Ensures the rotational velocity is not above a large amount.
  if (abs(desiredAngleVel) >= maxAngleVel) {
    desiredAngleVel = sgn(desiredAngleVel) * maxAngleVel;
  } 
  angleVelError = desiredAngleVel - instantPhi;

  // Calculates the desired position velocity.
  integralDistError = integralDistError + distanceError*((float)desired_Ts_ms / 1000);
  desiredDistVel = Kp_distance * distanceError + Ki_distance * integralDistError;

  // Ensures the forward velocity is not above a large amount.
  if (abs(desiredDistVel) >= maxDistVel) {
    desiredDistVel = sgn(desiredDistVel) * maxDistVel;
  }
  distanceVelError = desiredDistVel - instantRho;

  // Calculates the average and difference voltages.
  difVolt = angleVelKp * angleVelError;
  avgVolt = distanceVelKp * distanceVelError;

  // Starts moving robot forward when the angle is set.
  if (abs(angleVelError) <= 0.01) {
    startMove = true;
  }

  // Sets the voltage on the motors based on the average and difference voltages.
  // Inverted because motor 1 is on the left and motor 2 is on the right.
  Voltage[0] = (avgVolt - difVolt)/2;   // Motor 1
  Voltage[1] = (avgVolt + difVolt)/2;   // Motor 2

  // Sets speed and direction on motors.
  for (int k = 0; k < 2; k++) {
    if (Voltage[k] > 0) {
      // Rotates the motors forward.
      digitalWrite(VoltageSign[k], HIGH);  
    }
    else {
      // Rotates the motors backward.
      digitalWrite(VoltageSign[k], LOW);  
    }

    // Saturates the voltage to be within the batter voltage range.
    Voltage[k] = min(abs(Voltage[k]), Battery_Voltage);

    // Applies the saturated requested voltage and models as PWM signal.
    PWM[k] = 255 * Voltage[k] / Battery_Voltage;

    // Sets motor speed
    analogWrite(MotorVoltage[k], PWM[k]);
  }

  // Calculates the current positions on the motors in radians.
  for (int i = 0; i < 2; i++) {
    pos_counts[i] = myEnc(i + 1);
    actual_pos[i] = 2*PI*(float)pos_counts[i]/3200;

    // Calculates the actual velocities in rad/sec on the motors.
    actual_speed[i] = (actual_pos[i] - prev_pos[i])/((float)desired_Ts_ms/1000);
    prev_pos[i] = actual_pos[i];
  }

  // Calculates instantaneous forward velocity and rotational velocity.
  instantRho = 0.25*(actual_speed[1] + actual_speed[0])/2;
  instantPhi = radius*(actual_speed[1] - actual_speed[0])/width; 

  // Samples every desired sample rate.
  while (millis() < last_time_ms + desired_Ts_ms) {
    // Waits until desired time passes.
  }
}

// ================
// FUNCTIONS
// ================

// Adjusts the count on the channels to be more precise for both
// motors 1 and 2. 
long myEnc(int motor) {
  // Activates to adjust the count on the motor 1 encoder.
  if (motor == 1) {
    if (digitalRead(Motor1Chan[0]) != digitalRead(Motor1Chan[1])) {
      return (counts[0] + 1);
    }
    else {
      return counts[0];
    }
  }

  // Activates to adjust the count on the motor 2 encoder.
  else if (motor == 2) {
    if (digitalRead(Motor2Chan[0]) != digitalRead(Motor2Chan[1])) {
      return (counts[1] + 1);
    }
    else {
      return counts[1];
    }  
  }
}

 // Returns a sign of the value.
 int sgn(float value) {
  if (value > 0) {
    return 1;
  }
  else if (value < 0) {
    return -1;
  }
  else {
    return 0;
  }
 }

// ================
// CODE ENDS HERE
// ================

// Demo 2 Robot Localization and Control V3
// ================
// Authors: Madeleine Houghton and Quinn Hejmanowski
// Date: 3/30/2024
// ================
// The purpose of this code is to design a controller
// to move a robot based on data collected by a web 
// camera (from a Raspberry Pi) that is detecting an
// Aruco marker. If no data is collected, the robot will
// rotate a set angle at defined intervals. When data
// is read by the web camera, the robot will move to that angle
// and, based on the camera data, move to within 1 ft of
// the marker. If doing task 2, another piece of code
// in the program will run to have the robot move in a circle.

// DISCLAIMER:
// This program assumes that the assembly of a robot structure,
// Raspberry Pi, and wiring for the motors to the Arduino and
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
//    x1 Raspberry Pi
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
//    Make the necessary connections between the Raspberry Pi
//    and Arduino.

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

// Global variables to be used for I2C communication.
#include <Wire.h>
#define MY_ADDR 8
volatile uint8_t offset = 0;
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;
char stringInput[32] = {};
volatile uint8_t reply = 0;
bool detected = false;
bool start = true;

// Assigns timing variables for sampling the motor velocities.
unsigned long desired_Ts_ms = 10;
unsigned long last_time_ms;
unsigned long start_time_ms;
bool startMove = false;
bool task2 = true;

// These variables keep track of the actual speed on the motors.
float actual_speed[2] = {0, 0};

// Assigns variables to keep track of the previous positions on
// the encoders.
float prev_pos[2] = {0, 0};

// These variables are read from the Raspberry Pi to tell the robot
// its current position.
float currentAngle = 0;
float currentPos = 0;
float startPos = 0;

// This holds calculated values for the controller to aim towards
// a desired velocity for the angle and position.
float desiredAngleVel = 0;
float desiredDistVel = 0;

// These variables are the user input of the target distance
// and velocity of the robot.
float TARGET_DISTANCE;
float TARGET_ANGLE_DEG;

// These parameters are used for the desired distance and angle.
// The variables change since the input is simulated as a ramp
// function.
float desiredDistance = 0;
float desiredAngle = 0;
float riseTimeDist = 10;
float riseTimeAngle = 10;
float distanceRise = 0;
float angleRise = 0;

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
int counter;
int turn;

// Defines the states and variables for a FSM. This model was
// used to simplify how the code will run.
int state;
const int IDLE_STATE = 0;
const int LOCATE_STATE = 1;
const int DETECTED_STATE = 2;
const int TURN_90_STATE = 3;
const int CIRCLE_STATE = 4;

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
  state = IDLE_STATE;
  counter = 0;
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
  float Ki_distance = 6;
  float distanceVelKp = 4;
  float maxDistVel = 3;

  // Defines max battery voltage
  float Battery_Voltage = 7.8;

  // IDLE STATE
  // In this state, it moves straight to the next state after resetting variables.
  if (state == IDLE_STATE) {
    // Activates when task is ready.
    if (start == true) {
      state = LOCATE_STATE;
      startMove = false;
      counter = 0;
      TARGET_DISTANCE = 0;
      TARGET_ANGLE_DEG = 0;
      desiredAngle = 0;
      desiredDistance = 0;
    }
  }

  // LOCATE STATE
  // Turns until a marker is detected. If a marker is detected, goes to the next state.
  else if (state == LOCATE_STATE) {
    // Sets turning amount until marker is detected.
    TARGET_ANGLE_DEG = 45 + 45*turn;
    TARGET_DISTANCE = robotDistance;
    riseTimeAngle = 1;
    angleRise = TARGET_ANGLE_DEG*PI/180/(riseTimeAngle * 1000) * desired_Ts_ms;

    // Simulates a ramp input for the desired angle. The robot will keep spinning
    // until a marker is detected.
    desiredAngle = desiredAngle + angleRise;

    // If there is data on the buffer from the PI, read it.
    if (msgLength > 0) {
      if (offset == 1) {
        digitalWrite(LED_BUILTIN,instruction[0]);
      }
      printReceived();
      msgLength = 0;

      // Tells robot it detected marker.
      detected = true;
    }

    // If the marker is detected, move to the next state.
    if (detected == true) {
      counter = 0;
      state = DETECTED_STATE;
      TARGET_ANGLE_DEG = currentAngle + robotAngle;
      TARGET_DISTANCE = currentPos;
      startPos = currentPos;
      desiredAngle = 0;
      desiredDistance = 0;
    }

    // Otherwise, continues turning. If the first turn is met, has
    // the robot turn again.
    else {
      if (startMove == true) {
        turn = turn + 1;
        startMove = false;
        counter = 0;
      }
    }
  }

  // DETECTED STATE
  // Once the web camera detects a marker, it will use the data read from the
  // web camera (angle and distance) to move to the target
  else if (state == DETECTED_STATE) {
    // If there is data on the buffer from the PI, read it.
    if (msgLength > 0) {
      if (offset == 1) {
        digitalWrite(LED_BUILTIN,instruction[0]);
      }
      printReceived();
      msgLength = 0;

      // Tells the robot the marker has been detected
      detected = true;
    } 
    else {
      // Resets the communication if webcamera no longer reads anything.
      detected = false;
    }

    // If the camera no longer detects anything, goes back to previous state.
    if (detected == false) {
      state = LOCATE_STATE;
      counter = 0;
    }

    // If the camera reads that the robot is within 1 ft. Goes to
    // next state if task 2. Otherwise 
    if (currentPos <= 0.8) {
      // Goes to 90 degree turn if task 2.
      if (task2 == true) {
        state = TURN_90_STATE;

        // Delays response so that the next state is not activated right away.
        while (millis() < last_time_ms + (desired_Ts_ms*100)) {
          // Waits until desired time passes.
        }
      }

      // Otherwise goes back to idle state.
      else {
        start = false;
        state = IDLE_STATE;
      }
      // Resets the variables for current position and angle for the next state.
      counter = 0;
      desiredAngle = 0;
      desiredDistance = 0;
      startMove = false;
      counts[0] = 0;
      counts[1] = 0;
      lastdl[0] = 0;
      lastdl[1] = 0;
      prevRobotDistance = 0;
      prevRobotAngle = 0;
      TARGET_DISTANCE = 0;
      TARGET_ANGLE_DEG = 0;
    }
  }

  // TURN 90 DEG STATE
  // Once the robot is within 1 ft of the marker, turns the robot 90 deg
  // clockwise to prepare for circle.
  else if (state == TURN_90_STATE) {
    // Sets angle parameters.
    TARGET_ANGLE_DEG = -90;
    TARGET_DISTANCE = 0;
    riseTimeAngle = 1;
    angleRise = TARGET_ANGLE_DEG*PI/180/(riseTimeAngle * 1000) * desired_Ts_ms;
    
    // Simulates a ramp input for the desired angle. 
    if (abs(desiredAngle) >= abs(TARGET_ANGLE_DEG*PI/180)) {
      desiredAngle = TARGET_ANGLE_DEG*PI/180;
    }
    // End of ramp function.
    else {
      desiredAngle = desiredAngle + angleRise;
    }

    // Once the robot turns 90 degrees clockwise,
    // goes to the next state. 
    if ((startMove == true) && (task2 == true)) {
      desiredAngle = 0;
      desiredDistance = 0;
      state = CIRCLE_STATE;

      // To perform circle correctly, reset location.
      counts[0] = 0;
      counts[1] = 0;
      lastdl[0] = 0;
      lastdl[1] = 0;
      prevRobotDistance = 0;
      prevRobotAngle = 0;
    }
  }

  // CIRCLE STATE
  // Makes the robot move in a circle at some radius.
  else if (state == CIRCLE_STATE) {
    float turnRadius = 1;
    TARGET_DISTANCE = 2*PI*turnRadius;
    TARGET_ANGLE_DEG = 360 + 40;

    // Sets up ramp function rises for the distance and angle.
    riseTimeDist = 6;
    riseTimeAngle = 6;
    distanceRise = TARGET_DISTANCE/(riseTimeDist * 1000) * desired_Ts_ms;
    angleRise = TARGET_ANGLE_DEG*PI/180/(riseTimeAngle * 1000) * desired_Ts_ms;

    // Simulates a ramp input for the desired angle. 
    if (abs(desiredAngle) >= abs(TARGET_ANGLE_DEG*PI/180)) {
      desiredAngle = TARGET_ANGLE_DEG*PI/180;
    }
    else {
      desiredAngle = desiredAngle + angleRise;
    }

    // Simulates a ramp input for the desired distance.
    if (abs(desiredDistance) >= abs(TARGET_DISTANCE)) {
      desiredDistance = TARGET_DISTANCE;
    }
    else {
      desiredDistance = desiredDistance + distanceRise;
    }
  }

  // By default goes to idle state.
  else {
    state = IDLE_STATE;
  }

  //-------------------
  // CODE THAT RUNS FOR ALL STATES
  // The code below is the feedback controller and can be applied to all states in the FSM.
  //-------------------

  // Converts the encoder counts to feet.
  currentdl[0] = (double)myEnc(1) * encClicksToFeet;
  currentdl[1] = (double)myEnc(2) * encClicksToFeet;

  // Updates last time program ran.
  last_time_ms = millis();
  
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

  // Calculates the phi error (angle) and rho error (distance).
  distanceError = desiredDistance - robotDistance;
  angleError = desiredAngle - robotAngle;

  // Updates current robot distance from marker using camera. Only used when camera detects marker.
  if (state == DETECTED_STATE) {
    distanceError = currentPos;
    angleError = currentAngle;
  }

  // Calculates the desired angular velocity.
  integralAngleError = integralAngleError + angleError*((float)desired_Ts_ms / 1000);
  desiredAngleVel = Kp_angle * angleError + Ki_angle * integralAngleError;
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
  // Sets a counter to ensure system is stable.
  if (abs(TARGET_ANGLE_DEG*PI/180 - robotAngle) < 0.01) {
    counter = counter + 1;
  } 
  else {
    counter = 0;
  }

  // If the counter is hit, will start moving the robot forward.
  if (counter > 30){
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
  instantRho = radius*(actual_speed[1] + actual_speed[0])/2;
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

// Sets what data will be received by Pi for data communication.
void printReceived() {
  for (int i = 0; i < msgLength; i++) {
    stringInput[i] = instruction[i];
    if (i%2 == 1){
      currentPos = instruction[i];
    } 
    else {
      currentAngle = instruction[i];
    }
  }
  Serial.println(currentPos);
  Serial.println(currentAngle);
}

// Requests data to fix timing.
void request() {
  // According to the Wire source code, we must call write() within the requesting ISR
  // and nowhere else. Otherwise, the timing does not work out. See line 238:
  // https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/Wire/src/Wire.cpp
  Wire.write(reply);
  reply = 0;
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

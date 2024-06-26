// Mini Project Finalized Controller (Computer Vision Version)
// ================
// Authors: Madeleine Houghton and Quinn Hejmanowski
// Date: 2/21/2024
// ================
// The purpose of this code is to design a PI controller
// for two motors to move to a desired position.
// This code is the final updated code before computer vision
// integration and serves to verify remaining tuning of the
// controller.

// DISCLAIMER:
// This program assumes that the assembly of a robot
// structure, wiring and communication requirements to a
// Raspberry Pi, and wiring for the motors to the Arduino and
// motor driver shield have been completed. Refer to Resources
// for additional info on the motor and motor driver shield
// wiring configurations. Refer to Computer Vision SEED lab
// tutorials on Pi to Arduino communication for additional
// info. The hardware configuration mentioned
// here are the minimum requirements to use the program
// as intended. 

// Hardware Required
// ----------------
//    x1 Arduino Uno Board
//    x1 Motor Driver Shield (compatible with Arduino)
//    x2 Motors with mounted wheels
//    x1 Voltage Regulator
//    x1 Battery Pack (~7.8 V) with fuse connector
//    x1 Raspberry Pi

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
//    Make sure necessary wiring between the Raspberry Pi and
//    Arduino is completed and data communication is working
//    as expected.

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
bool stop = false;

// These variables keep track of the actual speed on the motors.
float actual_speed[2] = {0, 0};

// Assigns variables to keep track of the previous positions on
// the encoders.
float prev_pos[2] = {0, 0};

// This should be given from Raspberry Pi.
float desired_pos[2] = {0, 0};

// Sets the target speed of the motors.
float desired_speed[2] = {0, 0};
float integral_error[2] = {0, 0};
float pos_error[2] = {0, 0};
float error[2] = {0, 0};

// Keeps track of the position of both motors in radians.
long pos_counts[2] = {0, 0};
float actual_pos[2] = {0, 0};

// Sets motor pins and defines them in an array. Also determines
// the voltage signs, motor voltage, and motor channels. This is
// done to monitor the direction and speed on each motor.
// Index 0 is for motor 1 and index 1 is for motor 2.
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

// Global variables to be used for I2C communication.
#include <Wire.h>
#define MY_ADDR 8
volatile uint8_t offset = 0;
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;
char stringInput[32] = {};
volatile uint8_t reply = 0;
int quadrant = 0;

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

  // Sets up for recieving from the Pi.
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize I2C.
  Wire.begin(MY_ADDR);
  // Set callbacks for I2C interrupts.
  Wire.onReceive(receive);
  Wire.onRequest(request);

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
}

// Runs the motors and collects data on the velocity of the motors.
void loop() {  
  // These variables are used for the control system and are important
  // for the feedback control loop in the system.
  float Kp_vel = 2.5;
  float Kp_pos = 18.7;
  float Ki_pos = 4.5;
  float maxVelocity = 8.5;
  float Battery_Voltage = 7.8;
  float Voltage[2] = {0, 0};
  unsigned int PWM[2] = {0, 0};

  // Updates last time program ran.
  last_time_ms = millis();

  // If there is data on the buffer from the PI, read it.
  if (msgLength > 0) {
    if (offset == 1) {
      digitalWrite(LED_BUILTIN,instruction[0]);
    }
    printReceived();
    msgLength = 0;
  }

  // Calculates the current time of the program.
  current_time = (float)(last_time_ms - start_time_ms)/1000;

  // Sets desired position for moving the wheels.
  if (quadrant == 0) {
    desired_pos[0] = 0;
    desired_pos[1] = 0;
  }
  else if (quadrant == 1) {
    desired_pos[0] = 0;
    desired_pos[1] = PI;
  }
  else if (quadrant == 2) {
    desired_pos[0] = PI;
    desired_pos[1] = PI;
  }
  else if (quadrant == 3) {
    desired_pos[0] = PI;
    desired_pos[1] = 0;
  }
  
  // Calculates the current positions on the motors in radians.
  for (int i = 0; i < 2; i++) {
    pos_counts[i] = myEnc(i + 1);
    actual_pos[i] = 2*PI*(float)pos_counts[i]/3200;

    // Calculates the actual velocities in rad/sec on the motors.
    actual_speed[i] = (actual_pos[i] - prev_pos[i])/((float)desired_Ts_ms/1000);
    prev_pos[i] = actual_pos[i];
  }

// Controller for aiming towards the desired position on both motors.
  for (int j = 0; j < 2; j++) {
    pos_error[j] = desired_pos[j] - actual_pos[j];
    integral_error[j] = integral_error[j] + pos_error[j]*((float)desired_Ts_ms / 1000);
    desired_speed[j] = Kp_pos * pos_error[j] + Ki_pos * integral_error[j];
    error[j] = desired_speed[j] - actual_speed[j];

    // Anti-windup for desired speed to limit integrator overshoot.
    if (abs(desired_speed[j]) > maxVelocity) {
      desired_speed[j] = sgn(desired_speed[j]) * maxVelocity;
      error[j] = sgn(error[j]) * min(maxVelocity / Kp_pos, abs(error[j]));
      integral_error[j] = (desired_speed[j] - Kp_pos*error[j]) / Ki_pos;
    }
    
    // Calculates voltage.
    Voltage[j] = Kp_vel * error[j];
  }

  // Sets speed and direction on motors.
  for (int k = 0; k < 2; k++) {
    if (Voltage[k] > 0) {
      // Rotates the motors counter-clockwise.
      digitalWrite(VoltageSign[k], HIGH);  
    }
    else {
      // Rotates the motors clockwise.
      digitalWrite(VoltageSign[k], LOW);  
    }

    // Saturates the voltage to be within the batter voltage range.
    Voltage[k] = min(abs(Voltage[k]), Battery_Voltage);

    // Applies the saturated requested voltage and models as PWM signal.
    PWM[k] = 255 * Voltage[k] / Battery_Voltage;

    // Sets motor speed
    analogWrite(MotorVoltage[k], PWM[k]);
  }

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
    quadrant = instruction[i];
  }
}

// Sends back quadrant if Pi needs to receive data from Arduino.
void receive() {
  // Set the offset, this will always be the first byte.
  offset = Wire.read();
  // If there is information after the offset, it is telling us more about the command.
  while (Wire.available()) {
    instruction[msgLength] = Wire.read();
    msgLength++;
    reply = quadrant;
  }
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

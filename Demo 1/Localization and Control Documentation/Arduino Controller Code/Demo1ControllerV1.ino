// Demo 1 Robot Localization and Control V1
// Author: Quinn Hejmanowski

// The purpose of this code is to provide a
// draft of the robot controller for localization
// and control. The code sets the desired voltage
// levels of the motors to move and turn the robot.

#define NUM_MOTOR_PINS 5
int MotorPinArray[NUM_MOTOR_PINS] = {4, 7, 8, 9, 10};
int Motor1Chan[2] = {2, 5};
int Motor2Chan[2] = {3, 6};
int VoltageSign[2] = {7,8};
int MotorVoltage[2] = {9,10};


long Lcount = 0;
long Rcount = 0;

long LInteruptTriggered = 0;
long RInteruptTriggered = 0;
int sampletime = 100;
long lastTime = 0;

int robotAngle = 0;
double prevRobotAngle = 0;
double robotAngularSpeed = 0;
double robotDistance = 0;
double prevRobotDistance = 0;
double robotSpeed = 0;

double desiredDistance = 0;
double desiredRobotAngle = 0;

double distanceError = 0;
double angleError = 0;

float angleKp = 1;
float distanceKp = 1;


double currentdl[2] = {0,0};
double lastdl[2] = {0,0};

double width = 11.375/12;

double encClicksToFeet = 40; //This number will likely need to be recalibrated

float Voltage[2] = {0,0};
float avgVolt = 0;
float difVolt = 0;

// The interupt for the left encoder
void LARise(){
  if(millis() > LInteruptTriggered + sampletime){
    LInteruptTriggered = millis();
 // Implements the logic to interpret the encoder. If A toggled so that A and B are equal, it's rotated twice clockwise. Otherwise, A toggled and it rotated twice counterclockwise
    if (digitalRead(Motor1Chan[0]) == digitalRead(Motor1Chan[1])){
      Lcount = Lcount + 2;
    } else {
      Lcount = Lcount - 2;
    }
  }
}

// The function for reading the left encoder
long LEncoder(){
  if (digitalRead(Motor1Chan[0]) != digitalRead(Motor1Chan[1])){
    return (Lcount + 1);
  } else {
    return (Lcount);
  }
}

// The interupt for the right encoder
void RARise(){
  if(millis() > RInteruptTriggered + sampletime){
    RInteruptTriggered = millis();
 // Implements the logic to interpret the encoder. If A toggled so that A and B are equal, it's rotated twice clockwise. Otherwise, A toggled and it rotated twice counterclockwise
    if (digitalRead(Motor2Chan[0]) == digitalRead(Motor2Chan[1])){
      Rcount = Rcount + 2;
    } else {
      Rcount = Rcount - 2;
    }
  }
}

// The function for reading the right encoder
long REncoder(){
  if (digitalRead(Motor2Chan[0]) != digitalRead(Motor2Chan[1])){
    return (Rcount + 1);
  } else {
      return (Rcount);
  }
}


void setup() {
  Serial.begin(115200);
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
  attachInterrupt(digitalPinToInterrupt(Motor1Chan[0]), LARise, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Motor2Chan[0]), RARise, CHANGE);

  // Enables set up pin for motor drive shield.
  digitalWrite(4, HIGH);

}

void loop() {
  lastTime = millis();
   // Convert encoder clicks to feet
  currentdl[0] = LEncoder()/encClicksToFeet;
  currentdl[1] = REncoder()/encClicksToFeet; 

  float Kp_vel = 2.5;
  float Kp_pos = 18.7;
  float Ki_pos = 4.5;
  float maxVelocity = 8.5;
  float Battery_Voltage = 7.8;
  float Voltage[2] = {0, 0};
  unsigned int PWM[2] = {0, 0};

  // Calculate the change in position and angle based on the encoder, and add to it previous values
  robotDistance = prevRobotDistance + (currentdl[0]-lastdl[0]+currentdl[1]-lastdl[1])/2;
  robotAngle = prevRobotAngle + (currentdl[0]-lastdl[0]-currentdl[1]+lastdl[1])/width;

  // Current values become previous values for next loop
  lastdl[0] = currentdl[0];
  lastdl[1] = currentdl[1];
  prevRobotDistance = robotDistance;
  prevRobotAngle = robotAngle;

  Serial.println(Rcount);

  // Determine the voltage to apply to the motors
  distanceError = desiredDistance - robotDistance;
  angleError = desiredRobotAngle - robotAngle;
  difVolt = angleKp * angleError;
  avgVolt = distanceKp * distanceError;

  Voltage[0] = avgVolt + difVolt/2;
  Voltage[1] = avgVolt - difVolt/2;

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

  while (millis() < lastTime + sampletime) {}

}


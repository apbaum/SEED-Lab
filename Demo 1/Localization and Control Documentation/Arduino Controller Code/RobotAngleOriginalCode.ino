// Robot Angle Code (Original)

// CHECK THESE VARIABLES

// GLOBAL VARIABLES
// Width of the robot and radius of the wheel. Both are in feet.
float width = 0.9479;
float radius = 0.268;

// Converts encoder counts to feet.
double encClicksToFeet = 2*PI*radius/3200;

// INSIDE LOOP()
// These variables are the gains for the angle controller.
float Kp_angle = 15;
float Ki_angle= 11;
float angleVelKp = 4.5;
float maxAngleVel = 4;

// Sets the input for how much time to spend on ramp rise.
float riseTimeAngle = 2.5;

// Corrects angle rise time if the angle is more than 180 degrees.
if (abs(TARGET_ANGLE_DEG) >= 180) {
  riseTimeAngle = 7;
}

float angleRise = TARGET_ANGLE_DEG*PI/180/(riseTimeAngle * 1000) * desired_Ts_ms;

robotAngle = prevRobotAngle - (currentdl[0] - lastdl[0] - currentdl[1] + lastdl[1])/width;

if (current_time < riseTimeAngle) {
  desiredAngle = desiredAngle + angleRise;
}
// End of ramp function.
else {
  desiredAngle = TARGET_ANGLE_DEG*PI/180;
}

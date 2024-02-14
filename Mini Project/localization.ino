// This is the code for localization. To set up, connect the CLK and DT (yellow and white) 
// of the left motor to pins 3 and 6 and the right motor to 5 and 2.
// Both encoders also need power, so connect both blue wires to Vcc and green to gnd
// (easiest to do this through the breadboard). Then, run the program. It will print time,
// x position, y position, and angle to the serial moniter. Copy this to the 'data' variable
// in the matlab code, and run that to see the animation. 


int LAPIN = 3;
int LBPIN = 6;

int RAPIN = 2;
int RBPIN = 5;

long Lcount = 0;
long Rcount = 0;

long LInteruptTriggered;
long RInteruptTriggered;
int sampletime = 100;
long lastTime = 0;

double currentdl = 0;
double currentdr = 0;
double lastdl = 0;
double lastdr = 0;
double currentangle = 0;
double lastangle = 0;
double currentx = 0;
double currenty = 0;
double lastx = 0;
double lasty = 0;
double width = 1.5;
double encClicksToFeet = 4; //currently converting to inches instead of feet in the name of the animation looking precise. This number will likely need to be recalibrated


long i = 0;

void setup(){
  pinMode(LAPIN, INPUT_PULLUP);
  pinMode(LBPIN, INPUT_PULLUP);
  pinMode(RAPIN, INPUT_PULLUP);
  pinMode(RBPIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LAPIN), LARise, RISING);
  attachInterrupt(digitalPinToInterrupt(RAPIN), RARise, RISING);

 // Opens a serial connection
  Serial.begin(9600);
  Serial.println("Started");
}

void loop(){

  //this if statement ends the serial moniter after 5 seconds, so that it's easier to copy to matlab
  if (sampletime*i/1000.0 > 5){
    Serial.end();
  }

  // convert encoder clicks to feet
  currentdl = LEncoder()/encClicksToFeet;
  currentdr = REncoder()/encClicksToFeet; 

  // calculate the change in position and angle based on the encoder, and add to it previous values
  currentx = lastx + cos(lastangle)*(currentdl-lastdl+currentdr-lastdr)/2;
  currenty = lasty + sin(lastangle)*(currentdl-lastdl+currentdr-lastdr)/2;
  currentangle = lastangle + (currentdl-lastdl-currentdr+lastdr)/width;

  // print the output in a format that is easy to copy into matlab
  Serial.print(sampletime*i/1000.0);
  Serial.print('\t');
  Serial.print(currentx);
  Serial.print('\t');
  Serial.print(currenty);
  Serial.print('\t');
  Serial.print(currentangle);
  Serial.println(";");

  // current values become previous values for next loop
  lastdl = currentdl;
  lastdr = currentdr;
  lastx = currentx;
  lasty = currenty;
  lastangle = currentangle;

  i++;

  // debounce
  while (millis() < lastTime + sampletime) {}
  lastTime = millis();
}

  // the interupt for the left encoder
void LARise(){
  if(millis() > LInteruptTriggered + sampletime){
    LInteruptTriggered = millis();
 // implements the logic to interpret the encoder. If A toggled so that A and B are equal, it's rotated twice clockwise. Otherwise, A toggled and it rotated twice counterclockwise
    if (digitalRead(LAPIN) == digitalRead(LBPIN)){
      Lcount = Lcount + 2;
    } else {
      Lcount = Lcount - 2;
    }
  }
}

  // the function for reading the left encoder
long LEncoder(){
  if (digitalRead(LAPIN) != digitalRead(LBPIN)){
    return (Lcount + 1);
  } else {
    return (Lcount);
  }
}

  // the interupt for the right encoder
void RARise(){
  if(millis() > RInteruptTriggered + sampletime){
    RInteruptTriggered = millis();
 // implements the logic to interpret the encoder. If A toggled so that A and B are equal, it's rotated twice clockwise. Otherwise, A toggled and it rotated twice counterclockwise
    if (digitalRead(RAPIN) == digitalRead(RBPIN)){
      Rcount = Rcount + 2;
    } else {
      Rcount = Rcount - 2;
    }
  }
}

  // the function for reading the right encoder
long REncoder(){
  if (digitalRead(RAPIN) != digitalRead(RBPIN)){
    return (Rcount + 1);
  } else {
      return (Rcount);
  }
}

// Pi to Arduino Communication Code
// Author: Adam Nussbaum

// This code communicates from a Pi and an Arduino for the
// Pi to send commands and the Arduino to respond to those
// commands. In this code, the Pi sends a number and the Arduino
// replies with the number received + 100.

// Global variables to be used for I2C communication.
#include <Wire.h>
#define MY_ADDR 8
volatile uint8_t offset = 0;
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;
char stringInput[32] = {};
volatile uint8_t reply = 0;

// Initializes Arduino.
void setup() {
  Serial.begin(115200);
  // We want to control the built-in LED (pin 13).
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize I2C.
  Wire.begin(MY_ADDR);
  // Set callbacks for I2C interrupts.
  Wire.onReceive(receive);
  Wire.onRequest(request);
}

// Runs program on Arduino.
void loop() {
  // If there is data on the buffer, read it.
  if (msgLength > 0) {
    if (offset==1) {
      digitalWrite(LED_BUILTIN,instruction[0]);
    }
    printReceived();
    
    msgLength = 0;
  }
}

// printReceived helps us see what data we are getting from the leader.
void printReceived() {
  // Print on serial console.
  Serial.print("Offset received: ");
  Serial.println(offset);
  Serial.print("Message Length: ");
  Serial.println(msgLength);
  Serial.print("Instruction received: ");
  for (int i=0;i<msgLength;i++) {
    Serial.print(String(instruction[i])+"\t");
    stringInput[i] = instruction[i];
  }
  Serial.println("");
  Serial.print("String: ");
  Serial.println(stringInput);
}

// Function called when an I2C interrupt event happens
void receive() {
  // Set the offset, this will always be the first byte.
  offset = Wire.read();
  // If there is information after the offset, it is telling us more about the command.
  while (Wire.available()) {
    instruction[msgLength] = Wire.read();
    msgLength++;
    reply = instruction[0] + 100;
  }
}

void request() {
  // According to the Wire source code, we must call write() within therequesting ISR
  // and nowhere else. Otherwise, the timing does not work out. See line 238:
  // https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/Wire/src/Wire.cpp
  Wire.write(reply);
  reply = 0;
 }
  }
}

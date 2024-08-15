#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "sensors.h"

void ejection(int pin) {
  // Set the pin as output
  pinMode(pin, OUTPUT);

  // Activating the parachute 
  digitalWrite(pin, HIGH);

  //Deployment delay
  delay(1000); // Delay for 1 second

  // Deactivating the parachute 
  digitalWrite(pin, LOW);
};

float getAltitude();
float getAcceleration();
float getVelocity();


void readAltimeterTask();


#endif
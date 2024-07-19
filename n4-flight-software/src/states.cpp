#include "states.h"
#include "functions.h"
#include "defs.h"
#include "sensors.h"
#include "elapsedMillis.h"

const int DROGUE_PIN = 3;
const int MAIN_PIN = 5;

elapsedMillis decelerationTimer; // Timer to track deceleration duration

bool isDeceleratingContinuously(){
  static float previousAltitude = 0.0f; // Store previous altitude
  float currentAltitude = getAltitude(); // Read current altitude
  float verticalAccel = getAcceleration(MPU_SENSOR_Z_AXIS) - 9.81f; // Calculate vertical acceleration

  // Check for deceleration event
  bool isDecelerating = verticalAccel < 0

 // Update deceleration timer
  if (isDecelerating) {
    if (!decelerationTimer.start()) {
      decelerationTimer.operator unsigned long(); // Start timer if deceleration starts
    }
  } else {
    decelerationTimer.stop(); // Reset timer if deceleration stops
  

  // Check for continuous deceleration
  return isDecelerating && decelerationTimer >= DECELERATION_DURATION;
  }
}

//define default current state
State currentState = State::PRE_FLIGHT_GROUND;

//State machine transition conditions

//checks if altitude is greater than 50m to determine powered flight
State isInPoweredFlight(float altitude) {
  if (altitude > 50.0f) { 
    return State::POWERED_FLIGHT;
  }
}
// Checks for continuous deceleration to determine coasting
State isInCoasting(bool isDecelerating) {
  if (isDeceleratingContinuously() && millis() >= DECELERATION_CHECK_DURATION) {
    return State::COASTING;
   } else {}
}
//Checks for velocity is 0 or negative to determine apogee is reached and deploy drogue
State isInApogee(float velocity, float altitude) {
  if (velocity <= 0 && altitude >= APOGEE_ALTITUDE) {
    return State::DROGUE_DEPLOY;
   } else if (altitude < DROGUE_DEPLOY_MIN_ALTITUDE) {}
    else {}
}
// Checks for an altitude of 450 meteres to deploy the main chute 
State isInMainChuteDeploy(float altitude) {
  if (altitude <= MAIN_CHUTE_DEPLOY_ALTITUDE) {
    return State::MAIN_CHUTE_DEPLOY;
   } else {}
}
// Checks for an acceleration equal or greater to gravity to determine post flight
State isInPostFlight(float acceleration) {
  if (acceleration >= 9.81f) { // Assuming acceleration due to gravity
    return State::POST_FLIGHT_GROUND;
   } else {}
}

void loop() {

  // Read sensor data
  float currentAltitude = getAltitude();
  float velocity = getVelocity(); 
  bool isDecelerating = isDeceleratingContinuously();

  if(modf == FLIGHT_MODE) {
    
    // Call state transition functions
   currentState = isInPoweredFlight(currentAltitude);
   currentState = isInCoasting(isDecelerating);
   currentState = isInApogee(velocity,currentAltitude);
   currentState = isInMainChuteDeploy(currentAltitude);

   } else {
    currentState = PRE_FLIGHT_GROUND;
  }
  
  
 // State transitions
  switch (currentState) {
   case State::PRE_FLIGHT_GROUND:
    Serial.println( "Pre-Flight State");
    break;
   case State::POWERED_FLIGHT:
    Serial.println("Powered Flight state");
    break;
   case State::COASTING:
    Serial.println("Coasting State");
    break;
   case State::DROGUE_DEPLOY:
    Serial.println("Apogee reached");
   // Call drogue eject function
    ejection(DROGUE_PIN);
    break;
   case State::DROGUE_DESCENT:
    Serial.println("Drogue Descent");
    break;
   case State::MAIN_CHUTE_DEPLOY:
    Serial.println("Main Chute Deploy");
    // Call main chute eject function
    ejection(MAIN_PIN);
    break;
   case State::POST_FLIGHT_GROUND:
    Serial.println("Post-Flight state");
    break;
   default:
    break;
  }

 // Delay between loop iterations
  delay(100); // will adjust as needed
}

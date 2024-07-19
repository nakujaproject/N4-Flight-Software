#ifndef STATES_H
#define STATE_H
#include "stdint.h"

//we initialize functions that will be included in checkstate.cpp


enum State {
  PRE_FLIGHT_GROUND,
  POWERED_FLIGHT,
  COASTING,
  DROGUE_DEPLOY,
  DROGUE_DESCENT,
  MAIN_CHUTE_DEPLOY,
  MAIN_DESCENT,
  POST_FLIGHT_GROUND
};

State isInPoweredFlight(float altitude);
State isInCoasting(bool isDecelerating);
State isInApogee(float velocity, float altitude);
State isInDrogueDeploy(float altitude);
State isInMainChuteDeploy(float altitude);
State isInPostFlight(float acceleration);


#endif
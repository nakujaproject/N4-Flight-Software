/**
 * @file states.h
 * This file defines the flight states
 *
 */
#ifndef STATES_H
#define STATES_H

typedef enum {
	PRE_FLIGHT_GROUND = 0,
	POWERED_FLIGHT,
	COASTING,
	APOGEE,
	DROGUE_DEPLOY,
	DROGUE_DESCENT,
	MAIN_DEPLOY,
	MAIN_DESCENT,
	POST_FLIGHT_GROUND
} ARMED_FLIGHT_STATE;

#endif

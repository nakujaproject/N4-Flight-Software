/**
 * @file states.h
 * This file describes the flight states 
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
} FLIGHT_STATE;

#endif

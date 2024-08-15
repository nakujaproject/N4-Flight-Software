/**
 * Written by: Edwin Mwiti
 * Email: emwiti658@gmail.com
 *
 * This file is responsible for defining the message log levels for post flight analysis
 * Use sparingly and accurately depending on the message being logged
 *
 */

#ifndef SYSTEM_LOG_LEVELS_H
#define SYSTEM_LOG_LEVELS_H

typedef enum{
	DEBUG = 0,
	INFO,
	WARNING,
	CRITICAL,
	ERROR
} LOG_LEVEL;

#endif

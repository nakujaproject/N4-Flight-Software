/**
 * @file system_log_levels.h
 * Defines the log levels we need for in flight event logging
 * 
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

typedef enum {
	WRITE = 0,
	APPEND
} LOG_MODE;

#endif

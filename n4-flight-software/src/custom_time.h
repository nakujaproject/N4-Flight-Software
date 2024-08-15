/*!****************************************************************************
 * @file custom-time.h
 * @brief This file defines functions needed for time conversions
 * for data logging 
 *******************************************************************************/

#ifndef CUSTOM_TIME_H
#define CUSTOM_TIME_H

#include <Arduino.h>

extern char tstamp[]; // to hold a human readable timestamp
void convertTimestamp(unsigned long);

#endif
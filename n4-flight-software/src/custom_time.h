/*!****************************************************************************
 * @file custom_time.h
 * @brief This file defines functions needed for human readable time conversion for data logging
 *******************************************************************************/

#ifndef CUSTOM_TIME_H
#define CUSTOM_TIME_H

#include <Arduino.h>

extern char tstamp[]; /*!< buffer to hold a human readable timestamp */
void convertTimestamp(unsigned long);

#endif
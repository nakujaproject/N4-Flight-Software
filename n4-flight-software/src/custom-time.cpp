/*!****************************************************************************
 * @file custom-time.c
 * @brief This file implements functions to convert time in milliseconds to 
 * a human readable format to be used for logging time 
 *******************************************************************************/


#include "custom-time.h"

char tstamp[50]; // to hold a timestamp
int minute=0, sec=0, msec=0;

/*!****************************************************************************
 * @brief convert time in millisecsonds to minutes, seconds and time that are human readable
 * @param msec time in milliseconds, got from millis() function
 *******************************************************************************/
char* convertTimestamp(unsigned long msec) {
    minute = ((msec/1000)/60) % 60;
    sec = (msec/1000) % 60;
    msec = msec%1000;

    sprintf(tstamp, "%d:%d:%ul", minute, sec, msec);
    return tstamp;

}

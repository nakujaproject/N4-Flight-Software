
/*!****************************************************************************
 * @file custom-time.c
 * @brief This file implements functions to convert time in milliseconds to 
 * a human readable format to be used for logging time 
 *******************************************************************************/


#include "custom-time.h"

char tstamp[50]; // to hold a timestamp
int minute=0, sec=0, msec=0;

/*!****************************************************************************
 * @brief convert time in milliseconds to minutes, seconds and time that are human readable
 * @param msec time in milliseconds, got from millis() function
 *******************************************************************************/
void convertTimestamp(unsigned long msec) {
    minute = ((msec/1000)/60) % 60;
    sec = (msec/1000) % 60;
    msec = msec%1000;

    sprintf(tstamp, "%d:%d:%ul", minute, sec, msec);

}

void setup() {
  Serial.begin(115200);

}

void loop() {
  convertTimestamp(millis());
  Serial.println(tstamp);
  delay(200);
}


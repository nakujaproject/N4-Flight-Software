#include <Arduino.h>
#include "FS.h"
#include "SD.h"
#include <CSV_Parser.h>

#define DEBUG 1
#if DEBUG
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif


#define SD_CS_PIN 5
enum TEST_STATE{
    DATA_CONSUME = 0,
    DONE_CONSUMING
};
int current_test_state = TEST_STATE::DATA_CONSUME;
const char* f_name = "/short_data.csv";
File file;

char feedRowParser(){
    return file.read();
}

bool rowParserFinished() {
    return ((file.available() > 0) ? false:true);
}

void setup() {
    Serial.begin(115200);

    if(!SD.begin(SD_CS_PIN)) {
        debugln("SD Card not found");

    } else {
        debugln("SD Card found");
    }

    // check if data file exists
    if(!SD.exists(f_name)) {
        debugln("ERR: File \" " + String(f_name) + " \" does not exist");
    }

    file = SD.open(f_name, FILE_READ);
    if(!file) {
        debugln("ERR: File open failed");
    }
}

void loop() {
    // assume we are in TEST MODE

    // check that we are in the DATA_CONSUME STATE
    if(current_test_state == TEST_STATE::DATA_CONSUME ) {
        // format float, float
        // params: format, has_header, delimiter
        CSV_Parser cp("ff", false, ',');

        if(cp.readSDfile(f_name)) {
            float* col1 = (float*)cp[0];
            float* col2 = (float*)cp[1];

            if(col1 && col2) {
                for(int row = 0; row < cp.getRowsCount(); row++) {
                    Serial.print("row = ");
                    Serial.print(row);
                    Serial.print(", col_1 = ");
                    Serial.print(col1[row]);
                    Serial.print(", col_2 = ");
                    Serial.println(col2[row]);
                }

                debugln("END OF FILE");
                current_test_state = TEST_STATE::DONE_CONSUMING;
            } else {
                debug("Error: at least one of the columns was not found");
            }

        } else {
            debug("File does not exist");
        }
    } else {
        //debugln("Done testing");
    }


}
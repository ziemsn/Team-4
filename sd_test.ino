#include <SD.h>
#include "ClearCore.h"

const int chipSelect = 4;

void setup() {
  Serial.begin(9600);

  // Initialize the SD card.
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed.");
    return;
  }

  // Open a file for writing.
  File dataFile = SD.open("data.txt", FILE_WRITE);
  if (!dataFile) {
    Serial.println("Error opening file for writing.");
    return;
  }
  int data1, data2;
  data1 = 68;
  data2 = 81;

    for(int i = 0; i < 10; i++){
        // Write some data to the file.
        dataFile.println(data1, ",", data2); 
    }

  // Close the file.
  dataFile.close();

  Serial.println("Data written to SD card.");
}

void loop() {
  // Nothing to do here.
}


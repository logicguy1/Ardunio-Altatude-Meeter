 /* ********************************************************************** 
 *  Micro SD card Adapter
 *
 * Circuit:
 * 
 *  |                              |
 *  | GND  VCC  MISO MOSI  SCK  CS |
 *  |--|----|----|----|----|----|--| 
 *     |    |    |    |    |    |  
 *    GND   5V  D12  D11  D13  D10
 *
 ****************************************************************************/


#include <SPI.h>
#include <SD.h>
File myFile;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    while (1); // wait for serial port to connect. Needed for native USB port only
  } 
}

void loop() {
  write_to_file("Test");

}

void write_to_file(String data) {
  if (!SD.begin(10)) { // Initialize the sd card 
    Serial.println("initialization failed!");
    while (1);
  }

  myFile = SD.open("data.txt", FILE_WRITE); // Open te file 
  
  if (!myFile) {
    // if the file didn't open, print an error:
    Serial.println("error opening data.txt");
    while (1);
  }

  myFile.println(data);
  myFile.close();
}

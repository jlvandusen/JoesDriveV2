#include <SoftwareSerial.h>
#include "DFRobotDFPlayerMini.h"

SoftwareSerial mySerial(5, 13); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

void setup() {
  delay (10000);
  Serial.begin(115200);
  mySerial.begin(9600);

  if (myDFPlayer.begin(mySerial, true, false)) {
    Serial.println("DFPlayer Mini initialized successfully!");
    myDFPlayer.volume(30); // Set volume to maximum (0 to 30)
    myDFPlayer.playMp3Folder(1); // Play the "0001.mp3" in the "mp3" folder
  } else {
    Serial.println("Connecting to DFPlayer Mini failed!");
  }
}

void loop() {
  // Your other code here
}

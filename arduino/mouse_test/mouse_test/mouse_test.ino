

#include "HID-Project.h"
// #include <SoftwareSerial.h>


void setup() {

  Serial1.begin(9600);
  // Serial.begin(9600);
  AbsoluteMouse.begin();
}

void loop() {


  if (Serial1.available() > 4) {
    // read byte data from Bluetooth
    byte x1 = Serial1.read();
    short int x2 = Serial1.read();
    byte y1 = Serial1.read();
    short int y2 = Serial1.read();
    byte buttons = Serial1.read();
    // get X/Y from bytes.
    short int x = x1 + (x2 << 8);
    short int y = y1 + (y2 << 8);
    // compute mouse position.
    int mx = (float(x) / 1920.0) * 65535 - 32768;
    int my = (float(y) / 1080.0) * 65535 - 32768;

    AbsoluteMouse.moveTo(mx, my);
  }
}

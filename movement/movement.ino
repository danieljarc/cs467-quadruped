#include "Quadruped.h"

SoftwareSerial bt(0, 1); //Bluetooth module (RX | TX)
Quadruped quadruped(&bt);

void setup() {
  quadruped.initialize();
}

void loop() {
  quadruped.processCommands();
}

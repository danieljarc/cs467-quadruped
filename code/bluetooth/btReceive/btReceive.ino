
#include <SoftwareSerial.h>
SoftwareSerial btSerial(2, 3); // (Rx pin, TX pin)

void setup() {
 Serial.begin(9600); // For arduino serial monitor
 Serial.println("Arduino started. Starting bluetooth setup...");
 btSerial.begin(9600); 
}

void loop() {
if (btSerial.available())
 { 

  Serial.flush();
  String str = btSerial.readString();
  Serial.println(str);
  btSerial.flush();
 }
}

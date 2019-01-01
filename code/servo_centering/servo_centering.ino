#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
const int servo_min[12] = { 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150 };
const int servo_max[12] = { 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600 };

void setup() {  
  Serial.begin(9600);
  Serial.println("---------------");
  Serial.println("Servo centering");
  Serial.println("---------------");
  pwm.begin();
  pwm.setPWMFreq(60);
  delay(10);
}

void loop() {
  
  for (uint8_t servo = 0; servo < 12; servo++) {
    Serial.println("Servo: " + servo);
    pulselength = map(90, 0, 180, servo_min[servo], servo_max[servo]);
    pwm.setPWM(servo, 0, pulselength);
    delay(500);
  }
}

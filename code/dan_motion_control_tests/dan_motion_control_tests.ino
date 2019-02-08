/*
 * Delay/motion test
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define TEST_SERVO 0
#define DELAY 15

unsigned int SERVO_MIN[1];
unsigned int SERVO_MAX[1];
int deg = 0; 

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {

  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60); 
  SERVO_MIN[TEST_SERVO] = 160;
  SERVO_MAX[TEST_SERVO] = 600;
  yield();
}

void loop() {

  
  for(int i = 0; i < 180; i++) {
    deg++; 
    pwm.setPWM(TEST_SERVO, 0, map(deg, 0, 180, SERVO_MIN[TEST_SERVO], SERVO_MAX[TEST_SERVO]));
    delay(DELAY); 
  }

  deg = 0;
  delay(10000);
}

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Min and max values for individual servos (150 to 600 default) 
const int servo_min[12] = { 120, 110, 120, 185, 175, 110, 120, 110, 110, 120, 155, 163 };
const int servo_max[12] = { 600, 570, 590, 585, 600, 600, 600, 570, 580, 570, 570, 570 };
const int servo = 11;
int deg;
int pulse_width;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {  
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
  delay(10);
}


void loop() {

    // Move to 0, 90, 180 degree positions using map() and servo min/max values...
    Serial.print("Servo: ");
    Serial.println(servo);
    
    deg = 0;
    pulse_width = map(deg, 0, 180, servo_min[servo], servo_max[servo]);
    pwm.setPWM(servo, 0, pulse_width);
    Serial.print("0 degrees - pulse_width: ");
    Serial.println(pulse_width);
    delay(5000);
    
    deg = 90;
    pulse_width = map(deg, 0, 180, servo_min[servo], servo_max[servo]);
    pwm.setPWM(servo, 0, pulse_width);
    Serial.print("90 degrees - pulse_width: ");
    Serial.println(pulse_width);
    delay(2000);    
    
    deg = 180;
    pulse_width = map(deg, 0, 180, servo_min[servo], servo_max[servo]);
    pwm.setPWM(servo, 0, pulse_width);
    Serial.print("180 degrees - pulse_width: ");
    Serial.println(pulse_width);
    delay(2000);
}

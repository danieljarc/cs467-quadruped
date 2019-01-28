/*
 * Description: Servo centering code (no potentiometer)
 * Usage: Manually check servo min/max values by altering servo, servo_min, servo_max and deg values then uploading
 * and checking for movement near min max positions (0 and 180). When there is no movement servo is at min or max. 
 * Store/copy value for that servo for later use. 
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Min and max values for individual servos (150 to 600 default). Values are different for every servo. 
// Change values until servo reaches mechanical min and max during tests... 
const int servo_min[12] = { 200, 115, 120, 185, 175, 110, 120, 110, 110, 120, 155, 163 };
const int servo_max[12] = { 500, 590, 590, 585, 600, 600, 600, 570, 580, 570, 570, 570 };

int servo;
int deg;
int pulse_width;  

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {  
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
}

/*
 * This will set servo 0 to 0 degrees based on the servo_min[0] value above,
 * Wait 2 seconds, then set servo 0 to 180 degrees based on the servo_max[0] value.
 */
void loop() {
  
    servo = 0;

    deg = 0; 
    pulse_width = map(deg, 0, 180, servo_min[servo], servo_max[servo]);
    pwm.setPWM(servo, 0, pulse_width);
    delay(2000);

    deg = 180;
    pulse_width = map(deg, 0, 180, servo_min[servo], servo_max[servo]);
    pwm.setPWM(servo, 0, pulse_width);
    delay(2000);
}

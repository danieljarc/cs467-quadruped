/*
 * Description: Servo centering (with potentiometer)
 * Credit: https://create.arduino.cc/projecthub/jeremy-lindsay/calibrating-my-servos-fa27ce
 */

#include <Adafruit_PWMServoDriver.h>
#define analogIn A0 // potentiometer pin

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int inputValue = 0; 
int pwmValue = 0; 
int tmpPwmValue = 0;

void setup() {  
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
}

void loop() {

    // Get potentiometer reading (0 to 1024)
    inputValue = analogRead(analogIn);

    // Convert to PWM value (100 to 612)
    pwmValue = (int)(100 + (inputValue / 2));

    // Set servo
    pwm.setPWM(0,0,pwmValue);

    // Display input and pwm if they change
    if (pwmValue != tmpPwmValue)
    {
      Serial.print("Input: ");
      Serial.print(inputValue);
      Serial.print(", PWM: ");
      Serial.println(pwmValue);
    }
    
    tmpPwmValue = pwmValue;
    delay(15);
}

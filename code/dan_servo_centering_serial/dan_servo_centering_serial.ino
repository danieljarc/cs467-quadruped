/*
 * Servo centering with serial input
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//Leg definitions
#define LEG_1 0
#define LEG_2 1
#define LEG_3 2
#define LEG_4 3
#define COXA 0
#define FEMUR 1
#define TIBIA 2
#define LEGS 4
#define LEG_SERVOS 3

int leg;
int servo;
int pwmMin[4][3];
int pwmMax[4][3];
int servoPin[4][3];
float legAngle[4][3];

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//Processes chars sent over serial
void processCommands() {

  if (Serial.available() > 0) {
    char c = Serial.read();
    switch(c){
      case '=':
        if(servo == 2) {
          servo=0;      
          if(leg<3){
            leg++;
          } else {
            leg=0;
          }
        } else {
          servo++;
        }
        Serial.print("LEG ");
        Serial.print(leg);
        Serial.print(", SERVO: ");
        Serial.println(servo);
        break;
      case '-':
        if(servo == 0) {
          servo=0;      
          if(leg>0){
            leg--;
          } else {
            leg=3;
          }
        } else {
          servo--;
        }
        Serial.print("LEG ");
        Serial.print(leg);
        Serial.print(", SERVO: ");
        Serial.println(servo);
        break; 
      case '[':
        pwmMin[leg][servo]--;
        Serial.print("pwmMin[");
        Serial.print(leg);
        Serial.print("][");
        Serial.print(servo);
        Serial.print("]: ");
        Serial.println(pwmMin[leg][servo]);
        pwm.setPWM(servoPin[leg][servo], 0, pwmMin[leg][servo]);
        break;
      case ']':
        pwmMin[leg][servo]++;
        Serial.print("pwmMin[");
        Serial.print(leg);
        Serial.print("][");
        Serial.print(servo);
        Serial.print("]: ");
        Serial.println(pwmMin[leg][servo]);
        pwm.setPWM(servoPin[leg][servo], 0, pwmMin[leg][servo]);
        break;
      case ',':
        pwmMax[leg][servo]--;
        Serial.print("pwmMax[");
        Serial.print(leg);
        Serial.print("][");
        Serial.print(servo);
        Serial.print("]: ");
        Serial.println(pwmMax[leg][servo]);
        pwm.setPWM(servoPin[leg][servo], 0, pwmMax[leg][servo]);
        break;
      case '.':
        pwmMax[leg][servo]++;
        Serial.print("pwmMax[");
        Serial.print(leg);
        Serial.print("][");
        Serial.print(servo);
        Serial.print("]: ");
        Serial.println(pwmMax[leg][servo]);
        pwm.setPWM(servoPin[leg][servo], 0, pwmMax[leg][servo]);
        break;
      case 'x':
        Serial.println("90 deg");
        pwm.setPWM(servoPin[leg][servo], 0, map(90, 0, 180, pwmMin[leg][servo], pwmMax[leg][servo]));
        break;
      case 'v':
        Serial.println("All 90 deg");
        for (int i=0;i<LEGS;i++){
          for(int j=0;j<LEG_SERVOS;j++){
             pwm.setPWM(servoPin[i][j], 0, map(90, 0, 180, pwmMin[i][j], pwmMax[i][j]));
          }
        }
        break;    
      default:
        break;
    }
  }
}

void setup() {  
  
  leg = 0;
  servo = 0;,
  
  
  for(int i=0;i<LEGS;i++){
    for(int j=0;j<LEG_SERVOS;j++){
      pwmMin[i][j]=120;
      pwmMax[i][j]=540;
    }
  }
  
  servoPin[LEG_1][COXA]=0;
  servoPin[LEG_1][FEMUR]=1;
  servoPin[LEG_1][TIBIA]=2;
  servoPin[LEG_2][COXA]=3;
  servoPin[LEG_2][FEMUR]=4;
  servoPin[LEG_2][TIBIA]=5;
  servoPin[LEG_3][COXA]=6;
  servoPin[LEG_3][FEMUR]=7;
  servoPin[LEG_3][TIBIA]=8; 
  servoPin[LEG_4][COXA]=9;
  servoPin[LEG_4][FEMUR]=10;
  servoPin[LEG_4][TIBIA]=11;

  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
  yield();
}

//Wait for command
void loop() {
    processCommands();
}

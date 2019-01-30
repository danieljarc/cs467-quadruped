/*
 * Graphical method/IK solution on a single leg....
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define COXA_LF 0           //coxa servo pin
#define FEMUR_LF 1          //femur servo pin
#define TIBIA_LF 2          //tibia servo pin
#define COXA_LENGTH 2.5     //coxa to femur in cm
#define FEMUR_LENGTH 4.3    //femur to tibia in cm
#define TIBIA_LENGTH 7.1    //tibia to foot in cm

unsigned int SERVO_MIN[12];
unsigned int SERVO_MAX[12];
float t1_tmp = -1.0;
float t2_tmp = -1.0;
float t3_tmp = -1.0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {

  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60); 
  SERVO_MIN[COXA_LF] = 120;
  SERVO_MAX[COXA_LF] = 590;
  SERVO_MIN[FEMUR_LF] = 130;
  SERVO_MAX[FEMUR_LF] = 590;
  SERVO_MIN[TIBIA_LF] = 120;
  SERVO_MAX[TIBIA_LF] = 600;
  yield();
}

void loop() {

  double x = 6.8;   //cm
  double y = 6.8;   //cm
  double z = 7.1;   //cm

  double theta1 = 0.0;      //radians
  double theta2 = 0.0;      //radians
  double theta3 = 0.0;      //radians
  double t1_deg = 0.0;      //deg
  double t2_deg = 0.0;      //deg
  double t3_deg = 0.0;      //deg
  
  double a1 = 0.0;          //radians
  double a2 = 0.0;          //radians
  double L1 = 0.0;          //cm
  double L = 0.0;           //cm
  
  //T1 -- COXA ANGLE (WORKS)
  theta1 = atan2(x, y); //eq. 1
  t1_deg = ((theta1/PI) * 180.0) + 45; //servo deg (45 degree offset)
  
  if (t1_deg > 180.0) {
    t1_deg = 180.0;
  }
  if (t1_deg < 0.0) {
    t1_deg = 0.0;
  }
  
  //T2 -- FEMUR ANGLE
  int test = pow(2,2);
  L1 = sqrt(pow(x,2) + pow(y,2)); //eq. 2
  L = sqrt(pow(z,2) + pow((L1 - COXA_LENGTH),2)); //eq. 3
  a1 = acos(z/L); //eq. 4
  a2 = acos((pow(FEMUR_LENGTH,2) + pow(L,2) - pow(TIBIA_LENGTH,2)) / (2 * FEMUR_LENGTH * L)); // eq. 5
  theta2 = a1 + a2; //eq.6
  t2_deg = ((theta2/PI) * 180.0) - 90.0;
  
  if (t2_deg > 180.0) {
    t2_deg = 180.0;
  }
  if (t2_deg < 0.0) {
    t2_deg = 0.0;
  }
  
  //T3 -- TIBIA ANGLE
  theta3 = acos((pow(FEMUR_LENGTH,2) + pow(TIBIA_LENGTH,2) - pow(L,2))/(2 * FEMUR_LENGTH * TIBIA_LENGTH)); //eq. 7
  t3_deg = (theta3/PI) * 180.0;
  
  if (t3_deg > 180.0) {
    t3_deg = 180.0;
  }
  if (t3_deg < 0.0) {
    t3_deg = 0.0;
  }
    
  if ((theta1 != t1_tmp) || (theta2 != t2_tmp) || (theta3 != t3_tmp)) {
    
    //pwm.setPWM(COXA_LF, 0, map(t1_deg, 0, 180, SERVO_MIN[COXA_LF], SERVO_MAX[COXA_LF]));
    //pwm.setPWM(FEMUR_LF, 0, map(t2_deg, 0, 180, SERVO_MIN[FEMUR_LF], SERVO_MAX[FEMUR_LF]));
    //pwm.setPWM(TIBIA_LF, 0, map(t3_deg, 0, 180, SERVO_MIN[TIBIA_LF], SERVO_MAX[TIBIA_LF])); 
    t1_tmp = theta1;
    t2_tmp = theta2;
    t3_tmp = theta3;

    // DEBUG....
    Serial.print("test: ");
    Serial.println(test);   
    Serial.println("---INPUT----");
    Serial.print("x: ");
    Serial.println(x);
    Serial.print("y: ");
    Serial.println(y);
    Serial.print("z: ");
    Serial.println(z);
    Serial.println("---CALC----");
    Serial.print("L1: ");
    Serial.println(L1);
    Serial.print("L: ");
    Serial.println(L);
    Serial.print("a1: ");
    Serial.println(a1);
    Serial.print("a2: ");
    Serial.println(a2);
    Serial.println("---OUTPUT----");
    Serial.print("THETA1: ");
    Serial.println(t1_deg);    
    Serial.print("THETA2: ");
    Serial.println(t2_deg);
    Serial.print("THETA3: ");
    Serial.println(t3_deg);
  }
}

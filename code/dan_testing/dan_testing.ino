/*
 * Graphical method/IK solution on a single leg....
 * Reference diagram for additional details. 
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define COXA_LF 0
#define FEMUR_LF 1
#define TIBIA_LF 2

#define COXA_LENGTH 2.5     //coxa (shoulder) to femur in cm
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
  SERVO_MIN[COXA_LF]=120;
  SERVO_MAX[COXA_LF]=590;
  SERVO_MIN[FEMUR_LF]=130;
  SERVO_MAX[FEMUR_LF]=590;
  SERVO_MIN[TIBIA_LF]=120;
  SERVO_MAX[TIBIA_LF]=600;
  yield();
}

void loop() {

  float target_x = 10.0;   //cm
  float target_y = 10.0;   //cm
  float target_z = 2.0;   //cm

  float theta1 = 0.0;     //radians
  float theta2 = 0.0;     //radians
  float theta3 = 0.0;     //radians
  float t1_deg = 0.0;     //deg
  float t2_deg = 0.0;     //deg
  float t3_deg = 0.0;     //deg
  
  float phi1 = 0.0;       //radians
  float phi2 = 0.0;       //radians
  float L0 = 0.0;         //cm
  float L1 = 0.0;         //cm
  float L2 = 0.0;         //cm
 
  //T1 -- COXA
  theta1 = atan(target_x/target_y); //eq. 1
  t1_deg = ((theta1/PI) * 180.0) + 45; //sevo deg (45 offset)

  if (theta1 != t1_tmp) {
    pwm.setPWM(COXA_LF, 0, map(t1_deg, 0, 180, SERVO_MIN[COXA_LF], SERVO_MAX[COXA_LF]));
    t1_tmp = theta1;
    Serial.print("THETA1: ");
    Serial.println(t1_deg);
  }

  //T2 -- FEMUR
  L0 = sqrt(pow(target_x,2) + pow(target_y,2)); //eq. 2
  L1 = sqrt(pow(TIBIA_LENGTH,2) + pow(FEMUR_LENGTH,2)); //eq. 3
  L2 = L0 - COXA_LENGTH; //eq. 4
  phi1 = atan(L2/target_z); //eq. 5
  phi2 = acos((pow(TIBIA_LENGTH,2) - pow(FEMUR_LENGTH,2) - pow(L1,2)) / (-2 * FEMUR_LENGTH * L1)); // eq. 6
  theta2 = phi1 + phi2; //eq.7
  t2_deg = (theta2/PI) * 180.0;  //sevo deg (no offset)
  
  if (theta2 != t2_tmp) {
    pwm.setPWM(FEMUR_LF, 0, map(t2_deg, 0, 180, SERVO_MIN[FEMUR_LF], SERVO_MAX[FEMUR_LF]));
    t2_tmp = theta2;
    Serial.print("THETA2: ");
    Serial.println(t2_deg);
  }

  //T3 -- TIBIA
  theta3 = acos((pow(L1,2) - pow(TIBIA_LENGTH,2) - pow(FEMUR_LENGTH,2))/(-2 * TIBIA_LENGTH * FEMUR_LENGTH)); //eq. 8
  t3_deg = (theta3/PI) * 180.0;  //sevo deg (no offset)
  
  if (theta3 != t3_tmp) {
    pwm.setPWM(TIBIA_LF, 0, map(t3_deg, 0, 180, SERVO_MIN[TIBIA_LF], SERVO_MAX[TIBIA_LF]));
    t3_tmp = theta3;
    Serial.print("THETA3: ");
    Serial.println(t3_deg);
  }
}

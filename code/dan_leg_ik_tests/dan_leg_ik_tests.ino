/*
 * Graphical method/IK solution on a single leg....
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define COXA 0              //coxa servo pin
#define FEMUR 1             //femur servo pin
#define TIBIA 2             //tibia servo pin
#define COXA_LENGTH 2.5     //coxa to femur in cm
#define FEMUR_LENGTH 4.3    //femur to tibia in cm
#define TIBIA_LENGTH 7.1    //tibia to foot in cm

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


void setup() {

  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60); 
  yield();

  double x = 4.8;   //cm
  double y = -4.8;   //cm
  double z = 8.1;   //cm

  double theta1 = 0.0;      //radians
  double theta2 = 0.0;      //radians
  double theta3 = 0.0;      //radians
  int t1_deg = 0;           //deg
  int t2_deg = 0;           //deg
  int t3_deg = 0;           //deg
  double phi1 = 0.0;        //radians
  double phi2 = 0.0;        //radians
  double beta = 0.0;        //radians
  double L0 = 0.0;          //cm
  double L1 = 0.0;          //cm
  double L2 = 0.0;          //cm
  
  //Theta1 - Coxa
  theta1 = atan2(y,x);
  
  //Theta 2 - Femur
  L0 = sqrt((x*x) + (y*y));
  L1 = L0 - COXA_LENGTH;
  L2 = sqrt((L1*L1) + (z*z));
  phi1 = atan(L1 / z);
  phi2 = acos(((L2*L2) + (FEMUR_LENGTH*FEMUR_LENGTH) - (TIBIA_LENGTH*TIBIA_LENGTH)) / (2 * L2 * FEMUR_LENGTH));
  beta = acos(((FEMUR_LENGTH*FEMUR_LENGTH) + (TIBIA_LENGTH*TIBIA_LENGTH) - (L2*L2)) / (2 * FEMUR_LENGTH * TIBIA_LENGTH));
  theta2 = phi1 + phi2;
    
  //Theta3 - Tibia
  theta3 = (PI - beta);

  //Coversion
  t1_deg = theta1*RAD_TO_DEG;
  t2_deg = theta2*RAD_TO_DEG;
  t3_deg = theta3*RAD_TO_DEG;

  //Results
  Serial.println("---INPUT----");
  Serial.print("x: ");
  Serial.println(x);
  Serial.print("y: ");
  Serial.println(y);
  Serial.print("z: ");
  Serial.println(z);
  Serial.println("---CALC----");
  Serial.print("L0: ");
  Serial.println(L0);
  Serial.print("L1: ");
  Serial.println(L1);
  Serial.print("L2: ");
  Serial.println(L2);
  Serial.print("phi1: ");
  Serial.println(phi1);
  Serial.print("phi2: ");
  Serial.println(phi2);
  Serial.print("beta: ");
  Serial.println(beta);
  Serial.println("---OUTPUT----");
  Serial.print("THETA1: ");
  Serial.println(t1_deg);    
  Serial.print("THETA2: ");
  Serial.println(t2_deg);
  Serial.print("THETA3: ");
  Serial.println(t3_deg);

}

void loop() {
  //Process commands
}

/*
 * Body IK testing...
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//General definitions
#define dt 5

//Body specific definitions
#define X 0
#define Y 1
#define Z 2
#define ROLL 0
#define PITCH 1
#define YAW 2

//Leg specific definitions
#define LEGS 1          //Leg count, 1 for testing...
#define LEG_1 0         //Right front
#define LEG_2 1         //Right rear
#define LEG_3 2         //Left rear
#define LEG_4 3         //Left front
#define COXA_CURRENT 0
#define COXA_TARGET 1
#define FEMUR_CURRENT 2
#define FEMUR_TARGET 3
#define TIBIA_CURRENT 4
#define TIBIA_TARGET 5
#define COXA_LENGTH 1.0   //cm
#define FEMUR_LENGTH 1.0  //cm
#define TIBIA_LENGTH 1.0  //cm

//Servo shield
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//Angles and positions...
float bodyAngle[3];
float bodyPos[3];
float legAngle[4][6];
float legPos[4][3];

//bodyAngle[ROLL]....bodyAngle[PITCH]....bodyAngle[YAW]
//bodyPos[X].........bodyPos[Y]..........bodyPos[Z]

//legAngle[LEG_1][COXA_CURRENT]....legAngle[LEG_2][COXA_CURRENT]....legAngle[LEG_3][COXA_CURRENT]....legAngle[LEG_4][COXA_CURRENT]
//legAngle[LEG_1][COXA_TARGET].....legAngle[LEG_2][COXA_TARGET].....legAngle[LEG_3][COXA_TARGET].....legAngle[LEG_4][COXA_TARGET]
//legAngle[LEG_1][FEMUR_CURRENT]...legAngle[LEG_2][FEMUR_CURRENT]...legAngle[LEG_3][FEMUR_CURRENT]...legAngle[LEG_4][FEMUR_CURRENT]
//legAngle[LEG_1][FEMUR_TARGET]....legAngle[LEG_2][FEMUR_TARGET]....legAngle[LEG_3][FEMUR_TARGET]....legAngle[LEG_4][FEMUR_TARGET]
//legAngle[LEG_1][TIBIA_CURRENT]...legAngle[LEG_2][TIBIA_CURRENT]...legAngle[LEG_3][TIBIA_CURRENT]...legAngle[LEG_4][TIBIA_CURRENT]
//legAngle[LEG_1][TIBIA_TARGET]....legAngle[LEG_2][TIBIA_TARGET]....legAngle[LEG_3][TIBIA_TARGET]....legAngle[LEG_4][TIBIA_TARGET]

//legPos[LEG_1][X]...legPos[LEG_2][X]...legPos[LEG_3][X]...legPos[LEG_4][X]
//legPos[LEG_1][Y]...legPos[LEG_2][Y]...legPos[LEG_3][Y]...legPos[LEG_4][Y]
//legPos[LEG_1][Z]...legPos[LEG_2][Z]...legPos[LEG_3][Z]...legPos[LEG_4][Z]

//Struct used for leg IK solutions
struct IKSolution {
  bool possible[4]; 
  float coxa_angle[4]; 
  float femur_angle[4];
  float tibia_angle[4];
  float legPos[4][3];
};

//Conversion to degrees
float toDeg(float rad){
  return RAD_TO_DEG*rad;
}

//Conversion to radians
float toRad(float deg){
  return DEG_TO_RAD*deg;
}

//Prints body info...
void printBodyInfo(){
  Serial.println("---------");
  Serial.println("Body Info");
  Serial.println("---------");
  Serial.print("X: ");
  Serial.print(bodyPos[X]); 
  Serial.print(" Y: ");
  Serial.print(bodyPos[Y]);
  Serial.print(" Z: ");
  Serial.println(bodyPos[Z]); 
  Serial.print("ROLL (u): ");
  Serial.print(bodyAngle[ROLL]);
  Serial.print(" PITCH (v): ");
  Serial.print(bodyAngle[PITCH]); 
  Serial.print(" YAW (w): ");
  Serial.println(bodyAngle[YAW]);
}

//Prints leg info...
void printLegInfo(int leg){
  Serial.println("----------");
  Serial.print("Leg ");
  Serial.print(leg);
  Serial.println(" Info");
  Serial.println("----------");
  Serial.print("X: ");
  Serial.print(legPos[leg][X]); 
  Serial.print(" Y: ");
  Serial.print(legPos[leg][Y]);
  Serial.print(" Z: ");
  Serial.println(legPos[leg][Z]); 
//  Serial.print("COXA: ");
//  Serial.print(legAngle[leg][COXA_CURRENT]);
//  Serial.print("/");
//  Serial.print(legAngle[leg][COXA_TARGET]);
//  Serial.print(" FEMUR: ");
//  Serial.print(legAngle[leg][FEMUR_CURRENT]);
//  Serial.print("/");
//  Serial.print(legAngle[leg][FEMUR_TARGET]);  
//  Serial.print(" TIBIA: ");
//  Serial.print(legAngle[leg][TIBIA_CURRENT]);
//  Serial.print("/");
//  Serial.println(legAngle[leg][TIBIA_TARGET]);
}

//Prints IK Solution Info...
void printIKSolution(struct IKSolution *sol){
  
  Serial.println("---------------");
  Serial.println("IK Solution Set");
  Serial.println("---------------");
  
  for(int i=0;i<LEGS;i++){
    Serial.print("possible[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.print(sol->possible[i]);
    Serial.print(", coxa_angle[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.print(sol->coxa_angle[i]);
    Serial.print(", femur_angle[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.print(sol->femur_angle[i]);
    Serial.print(", tibia_angle[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(sol->tibia_angle[i]);
  }
}

//Moves the servos
void moveServos(){
  
  bool complete = false; 
  bool converged[4]={false,false,false,false};

  //While servos still need to be adjusted
  while(!complete){

    //Iterate through each leg...
    for(int i=0;i<LEGS;i++){
      
      //If the leg angles have not coverged on target...
      if(!converged[i]){
      
          //Check latest differences
          float dCAngle = legAngle[i][COXA_TARGET]-legAngle[i][COXA_CURRENT];
          float dFAngle = legAngle[i][FEMUR_TARGET]-legAngle[i][FEMUR_CURRENT];
          float dTAngle = legAngle[i][TIBIA_TARGET]-legAngle[i][TIBIA_CURRENT];
      
          //All leg angles are converged
          if((dCAngle + dFAngle + dTAngle) == 0.0) {
            
            converged[i] = true; 
            
          } else {
      
            //Increment servo angles as necessary...
            if(dCAngle != 0.0) {
              legAngle[i][COXA_CURRENT]+=dCAngle/abs(dCAngle);
              //Set PWM
              delay(dt);
            }
            if(dFAngle != 0.0){
              legAngle[i][FEMUR_CURRENT]+=dFAngle/abs(dFAngle);
              //Set PWM
              delay(dt);
            }
            if(dTAngle != 0.0){
              legAngle[i][TIBIA_CURRENT]+=dTAngle/abs(dTAngle);
              //Set PWM
              delay(dt);
            }
          }       
        }
      }   
    complete = (converged[LEG_1] && converged[LEG_2] && converged[LEG_3] && converged[LEG_4]); 
  }  
}

//Calculates an IK solution for legs
void legIK(int leg, float newX, float newY, float newZ, struct IKSolution* solution) {

  // Calculate angles based on previously written IK leg code or jacobian if I ever understand it...
  //ADD
  
  //Check within constraints....
  //ADD
  
  //Set IKSolution values
  solution->possible[leg] = true;
  solution->coxa_angle[leg] = PI;
  solution->femur_angle[leg] = PI;
  solution->tibia_angle[leg] = PI;
  solution->legPos[leg][X] = newX;
  solution->legPos[leg][Y] = newY;
  solution->legPos[leg][Z] = newZ;
}

//Checks body angle constraints 
bool checkBodyConstraints(float u, float v, float w){
  return (abs(u)<=20.0 && abs(v)<=20.0 && abs(w)<=20.0); 
}

//Changes leg positioning based on body angles (Euler)
// u == roll (rotations around x axis)
// v == pitch (rotation around y axis)
// w == yaw (rotation around z axis)
void bodyIK(float u, float v, float w) {

  //Check within general body angle constraints (in degrees)
  bool valid = checkBodyConstraints(u, v, w);
  
  //Good so far...calculate new leg positions..
  if(valid) {

    u = toRad(u);
    v = toRad(v);
    w = toRad(w);
    
    //Body rotation matrix
    float R11 = cos(v)*cos(w);
    float R12 = sin(u)*sin(v)*cos(w) - cos(u)*sin(w);
    float R13 = sin(u)*sin(w) + cos(u)*sin(v)*cos(w);
    float R21 = cos(v)*sin(w);
    float R22 = cos(u)*cos(w) + sin(u)*sin(v)*sin(w);
    float R23 = cos(u)*sin(v)*sin(w) - sin(u)*cos(w);
    float R31 = -sin(v);
    float R32 = sin(u)*cos(v);
    float R33 = cos(u)*cos(v);
    
    float newX; //Leg X after R
    float newY; //Leg Y after R
    float newZ; //Leg Z after R
    
    struct IKSolution solution; //Stores leg solutions

    //For each leg, calculate transformed foot position, then get IK solution
    for(int i=0;i<LEGS;i++){  
      newX = legPos[i][X]*R11 + legPos[i][Y]*R12 + legPos[i][Z]*R13;
      newY = legPos[i][X]*R21 + legPos[i][Y]*R22 + legPos[i][Z]*R23;
      newZ = legPos[i][X]*R31 + legPos[i][Y]*R32 + legPos[i][Z]*R33;
      legIK(i, newX, newY, newZ, &solution);
    }

    //DEBUG...
    //printIKSolution(&solution);
    
    //Check colution set for failures
    for(int i=0;i<LEGS;i++){
      if(solution.possible[i]==false){
        valid = false; 
      }
    }

    //Still valid? Set new body angle values, leg angle targets....
    if(valid) {
      
      bodyAngle[ROLL]=toDeg(u);
      bodyAngle[PITCH]=toDeg(v);
      bodyAngle[YAW]=toDeg(w);
      
      for(int i=0;i<LEGS;i++){
        legAngle[i][COXA_TARGET] = toDeg(solution.coxa_angle[i]);
        legAngle[i][FEMUR_TARGET] = toDeg(solution.femur_angle[i]);
        legAngle[i][TIBIA_TARGET] = toDeg(solution.tibia_angle[i]);
        legPos[i][X] = solution.legPos[i][X];
        legPos[i][Y] = solution.legPos[i][Y];
        legPos[i][Z] = solution.legPos[i][Z];
      }  
    }

    //Actually move the servos...
    //moveServos();
  }
}

//Handle input from BT
bool processCommand(char c) {
  bool valid = true;
    switch(c){
      default:
      valid = false;
      break;
    } 
    return valid;
}

void setup() {
  
  //Opens serial port, sets data rate to 9600 bps
  Serial.begin(9600);

  //Initialize servo shield
  pwm.begin();
  pwm.setPWMFreq(60);

  //Initialize body angles and position
  for(int i=0;i<3;i++){
      bodyAngle[i]=0.0;
      bodyPos[i]=0.0;
  }

  //Initialize leg angles and positions
  for(int i=0;i<4;i++){
    for(int j=0;j<6;j++){
      legAngle[i][j]=0.0;
    }
    for(int j=0;j<3;j++) {
      legPos[i][j]=0.0;
    }
  }

  //Set default leg position
  legPos[LEG_1][X] = 5.0;
  legPos[LEG_1][Y] = 5.0;
  legPos[LEG_1][Z] = 5.0;
    
  //Send rotation request to body
  //bodyIK(roll, pitch, yaw)
  //-roll is left, +roll is right (max 20.0)
  //-pitch is down, +pitch is up (max 20.0)
  //-yaw is left, +yaw is right (max 20.0)
  bodyIK(-10.0,0.0,0.0);
  
  //Check result...
  printBodyInfo();
  printLegInfo(LEG_1);

  yield();
}


void loop() {
  
//Receive input and process commands...
//  if (Serial.available() > 0) {
//    char c = Serial.read();
//    processCommand(c);
//  }

}

/*
 * Body IK testing...
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//Body definitions
#define X 0            
#define Y 1             
#define Z 2             
#define ROLL 0
#define PITCH 1
#define YAW 2

//Leg definitions
#define LEG_1 0
#define LEG_2 1
#define LEG_3 2
#define LEG_4 3
#define COXA 0
#define FEMUR 1
#define TIBIA 2
#define COXA_LENGTH 2.5f 
#define FEMUR_LENGTH 4.3f
#define TIBIA_LENGTH 7.1f
#define LEGS 4
#define LEG_SERVOS 3

//Servo shield
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//Stores body and leg angles/positions
float bodyAngle[3];
float bodyPos[3];
float legAngle[4][3];
float legPos[4][3];

//Stores PWM min and max values, pin map, degree min/max for servo
int pwmMin[4][3];
int pwmMax[4][3];
int servoPin[4][3];
int servoDegMin[4][3];
int servoDegMax[4][3];

//Used for IK calculations
struct Solution {
  bool possible[4]; 
  float legAngle[4][3];
  float legPos[4][3];
};

//Save leg solution values
void setLegValues(struct Solution *sol) {
  for(int i=0;i<4;i++){
    for(int j=0;j<3;j++){
      legPos[i][j]=sol->legPos[i][j];
      legAngle[i][j]=sol->legAngle[i][j];
    }  
  } 
}

//Save body values
void setBodyValues(float u, float v, float w, float x, float y, float z) {
  bodyAngle[ROLL]=u;
  bodyAngle[PITCH]=v;
  bodyAngle[YAW]=w;
  bodyPos[X]=x;
  bodyPos[Y]=y;
  bodyPos[Z]=z;
}

//Prints body info...
void printBodyInfo(){
  Serial.println("---------");
  Serial.println("Body Info");
  Serial.println("---------");
  Serial.print("X: ");
  Serial.print(bodyPos[X]); 
  Serial.print(", Y: ");
  Serial.print(bodyPos[Y]);
  Serial.print(", Z: ");
  Serial.print(bodyPos[Z]); 
  Serial.print(", Roll(u): ");
  Serial.print(bodyAngle[ROLL]);
  Serial.print(", Pitch(v): ");
  Serial.print(bodyAngle[PITCH]); 
  Serial.print(", Yaw(w): ");
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
  Serial.print(", Y: ");
  Serial.print(legPos[leg][Y]);
  Serial.print(", Z: ");
  Serial.print(legPos[leg][Z]); 
  Serial.print(", Coxa Angle: ");
  Serial.print(legAngle[leg][COXA]);
  Serial.print(", Femur Angle: ");
  Serial.print(legAngle[leg][FEMUR]);  
  Serial.print(", Tibia Angle: ");
  Serial.println(legAngle[leg][TIBIA]);
}

//Prints Solution struct info...
void printIKSolution(struct Solution *sol){
  Serial.println("-------------");
  Serial.println("Solution Info");
  Serial.println("-------------");
  for(int i=0;i<LEGS;i++){
    Serial.print("Leg ");
    Serial.print(i);
    Serial.print(" - Possible: ");
    Serial.print(sol->possible[i]);
    Serial.print(", X: ");
    Serial.print(sol->legPos[i][X]);
    Serial.print(", Y: ");
    Serial.print(sol->legPos[i][Y]);
    Serial.print(", Z: ");
    Serial.print(sol->legPos[i][Z]);
    Serial.print(", Coxa Angle: ");
    Serial.print(sol->legAngle[i][COXA]);
    Serial.print(", Femur Angle: ");
    Serial.print(sol->legAngle[i][FEMUR]);
    Serial.print(", Tibia Angle: ");
    Serial.println(sol->legAngle[i][TIBIA]);
  }
}

//Moves the servos based on data in solution struct
void moveServos(struct Solution *sol){
  for(int i=0;i<LEGS;i++){
    for(int j=0;j<LEG_SERVOS;j++){
      pwm.setPWM(servoPin[i][j], 0, map(sol->legAngle[i][j], servoDegMin[i][j], servoDegMax[i][j], pwmMin[i][j], pwmMax[i][j]));
    }
  }
}

//Calculates a final IK solution for legs based on data passed in solution struct
void legIK(struct Solution* sol) {
 
  float theta1 = 0.0f;      //radians
  float theta2 = 0.0f;      //radians
  float theta3 = 0.0f;      //radians
  float phi1 = 0.0f;        //radians
  float phi2 = 0.0f;        //radians
  float beta = 0.0f;        //radians
  float L0 = 0.0f;          //cm
  float L1 = 0.0f;          //cm
  float L2 = 0.0f;          //cm
  float x = 0.0f;           //cm
  float y = 0.0f;           //cm
  float z = 0.0f;           //cm
  
  //Iterate through each leg...
  for (int i=0; i<LEGS;i++){

    x = sol->legPos[i][X];
    y = sol->legPos[i][Y];
    z = sol->legPos[i][Z];

    theta1 = atan2(y,x);
    L0 = sqrt((x*x) + (y*y));
    L1 = L0 - COXA_LENGTH;
    L2 = sqrt((L1*L1) + (z*z));
    phi1 = atan(L1/z);
    phi2 = acos(((L2*L2) + (FEMUR_LENGTH*FEMUR_LENGTH) - (TIBIA_LENGTH*TIBIA_LENGTH)) / (2 * L2 * FEMUR_LENGTH));
    beta = acos(((FEMUR_LENGTH*FEMUR_LENGTH) + (TIBIA_LENGTH*TIBIA_LENGTH) - (L2*L2)) / (2 * FEMUR_LENGTH * TIBIA_LENGTH));
    theta2 = phi1 + phi2;
    theta3 = PI - beta;

    //Check within constraints for leg....ADD
    bool possible = true;
    
    //Set appropriate values in solution set
    sol->possible[i] = possible;
    sol->legAngle[i][COXA] = theta1*RAD_TO_DEG;
    sol->legAngle[i][FEMUR] = theta2*RAD_TO_DEG;
    sol->legAngle[i][TIBIA] = theta3*RAD_TO_DEG;
  }
}

//Changes leg positioning based on translation in body x, y or z
void translateBody(float dX, float dY, float dZ) {
  
  float bodyX = bodyPos[X]-=dX;
  float bodyY = bodyPos[Y]-=dY;
  float bodyZ = bodyPos[Z]+=dZ;
  
  //Check within translation constraints...ADD
  bool valid = true; 
  //(abs(bodyX) < 2.0f && abs(bodyY) < 2.0f && abs(bodyZ) < 2.0f);
  
  if (valid) {
  
    struct Solution sol; 
    
    for(int i=0;i<LEGS;i++){
      float newX = legPos[i][X]-=dX;
      float newY = legPos[i][Y]-=dY;
      float newZ = legPos[i][Z]+=dZ;
      sol.legPos[i][X]=newX;
      sol.legPos[i][Y]=newY;
      sol.legPos[i][Z]=newZ;
      legIK(&sol);
    }
    
    //If all legs have a valid IK solution...
    if (sol.possible[LEG_1] && sol.possible[LEG_2] && sol.possible[LEG_3] && sol.possible[LEG_4]) {
      setLegValues(&sol);
      setBodyValues(0.0f,0.0f,0.0f,bodyX,bodyY,bodyZ);
      printIKSolution(&sol);
      printBodyInfo();
      moveServos(&sol);
    } 
  }
}

//Changes leg positioning based on pitch/roll/yaw change
void rotateBody(float dU, float dV, float dW) {

  float u = bodyAngle[ROLL]+=dU;
  float v = bodyAngle[PITCH]+=dU;
  float w = bodyAngle[YAW]+=dW;
  
  //Check within body angle constraints
  bool valid = (abs(u) < 20.0f && abs(v) < 20.0f && abs(w) < 20.0f);
  
  if (valid) {

    //Convert to radians
    u = u*DEG_TO_RAD; //roll(x axis)
    v = v*DEG_TO_RAD; //pitch(y axis)
    w = w*DEG_TO_RAD; //yaw(z axis)
    
    //Body rotation matrix R
    float R11 = cos(v)*cos(w);
    float R12 = sin(u)*sin(v)*cos(w) - cos(u)*sin(w);
    float R13 = sin(u)*sin(w) + cos(u)*sin(v)*cos(w);
    float R21 = cos(v)*sin(w);
    float R22 = cos(u)*cos(w) + sin(u)*sin(v)*sin(w);
    float R23 = cos(u)*sin(v)*sin(w) - sin(u)*cos(w);
    float R31 = -sin(v);
    float R32 = sin(u)*cos(v);
    float R33 = cos(u)*cos(v);

    struct Solution sol; 
    
    //For each leg, calculate transformed foot position, then get an IK solution for that leg
    for(int i=0;i<LEGS;i++){
      float newX = legPos[i][X]*R11 + legPos[i][Y]*R12 + legPos[i][Z]*R13;
      float newY = legPos[i][X]*R21 + legPos[i][Y]*R22 + legPos[i][Z]*R23;
      float newZ = legPos[i][X]*R31 + legPos[i][Y]*R32 + legPos[i][Z]*R33;
      sol.legPos[i][X]=newX;
      sol.legPos[i][Y]=newY;
      sol.legPos[i][Z]=newZ;
      legIK(&sol);
    }

    //If all legs have a valid IK solution...
    if (sol.possible[LEG_1] && sol.possible[LEG_2] && sol.possible[LEG_3] && sol.possible[LEG_4]) {
      setLegValues(&sol);
      setBodyValues(u*RAD_TO_DEG,v*RAD_TO_DEG,w*RAD_TO_DEG,0.0f,0.0f,0.0f);
      moveServos(&sol);
    }
  }
}

//Handle input from BT
bool processCommands() {

  if (Serial.available() > 0) {
    char c = Serial.read();
    switch(c){
      case '`':
        printBodyInfo();
        break; 
      case '1':
        printLegInfo(1);        
        break;
      case '2':
        printLegInfo(2);        
        break;
      case '3':
        printLegInfo(3);        
        break;
      case '4':
        printLegInfo(4);        
        break;
      case 'z':
        translateBody(0.0,0.0,1.0);
        break;
      case 'x':
        translateBody(0.0,0.0,-1.0);
        break;
      case 'w':
        translateBody(1.0,0.0,0.0);
        break;
      case 's':
        translateBody(-1.0,0.0,0.0);
        break; 
      case 'a':
        translateBody(0.0,-1.0,0.0);
        break;
      case 'd':
        translateBody(0.0,1.0,0.0);
        break;        
      default:
        break;
    }
  }
}

void setup() {

  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
  
  //Initialize body angles and position
  for(int i=0;i<3;i++){
      bodyAngle[i]=0.0;
      bodyPos[i]=0.0;
  }

  //Servo pin mapping 
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

  //Servo PWM min/max values
  pwmMin[LEG_1][COXA]=275;
  pwmMax[LEG_1][COXA]=520;  
  pwmMin[LEG_1][FEMUR]=275;
  pwmMax[LEG_1][FEMUR]=475;
  pwmMin[LEG_1][TIBIA]=255;
  pwmMax[LEG_1][TIBIA]=550;
  
  pwmMin[LEG_2][COXA]=200;
  pwmMax[LEG_2][COXA]=450;
  pwmMin[LEG_2][FEMUR]=250;
  pwmMax[LEG_2][FEMUR]=440;
  pwmMin[LEG_2][TIBIA]=140;
  pwmMax[LEG_2][TIBIA]=420;
  
  pwmMin[LEG_3][COXA]=230;
  pwmMax[LEG_3][COXA]=470;
  pwmMin[LEG_3][FEMUR]=250;
  pwmMax[LEG_3][FEMUR]=440;
  pwmMin[LEG_3][TIBIA]=260;
  pwmMax[LEG_3][TIBIA]=540;

  pwmMin[LEG_4][COXA]=210;
  pwmMax[LEG_4][COXA]=440;
  pwmMin[LEG_4][FEMUR]=260;
  pwmMax[LEG_4][FEMUR]=460;
  pwmMin[LEG_4][TIBIA]=130;
  pwmMax[LEG_4][TIBIA]=410;
  
  //Servo degree-pwm mapping
  servoDegMin[LEG_1][COXA]=0;
  servoDegMax[LEG_1][COXA]=90;
  servoDegMin[LEG_1][FEMUR]=135;
  servoDegMax[LEG_1][FEMUR]=45;
  servoDegMin[LEG_1][TIBIA]=120;
  servoDegMax[LEG_1][TIBIA]=0;  

  servoDegMin[LEG_2][COXA]=-90;
  servoDegMax[LEG_2][COXA]=0;
  servoDegMin[LEG_2][FEMUR]=45;
  servoDegMax[LEG_2][FEMUR]=135;
  servoDegMin[LEG_2][TIBIA]=0;
  servoDegMax[LEG_2][TIBIA]=120;

  servoDegMin[LEG_3][COXA]=-180;
  servoDegMax[LEG_3][COXA]=-90;
  servoDegMin[LEG_3][FEMUR]=135;
  servoDegMax[LEG_3][FEMUR]=45;
  servoDegMin[LEG_3][TIBIA]=120;
  servoDegMax[LEG_3][TIBIA]=0;  

  servoDegMin[LEG_4][COXA]=90;
  servoDegMax[LEG_4][COXA]=180;
  servoDegMin[LEG_4][FEMUR]=45;
  servoDegMax[LEG_4][FEMUR]=135;
  servoDegMin[LEG_4][TIBIA]=0;
  servoDegMax[LEG_4][TIBIA]=120;  

  //Starting pos/angle - Right front (+y and +x)
  legPos[LEG_1][X]=4.8;
  legPos[LEG_1][Y]=4.8;
  legPos[LEG_1][Z]=7.1;
  legAngle[LEG_1][COXA]=45.0;
  legAngle[LEG_1][FEMUR]=89.0;
  legAngle[LEG_1][TIBIA]=90.0;
  
  //Starting pos/angle - Left front (-y and +x)
  legPos[LEG_2][X]=4.8;
  legPos[LEG_2][Y]=-4.8;
  legPos[LEG_2][Z]=7.1;
  legAngle[LEG_2][COXA]=-45.0;
  legAngle[LEG_2][FEMUR]=89.0;
  legAngle[LEG_2][TIBIA]=90.0;
  
  //Starting pos/angle - Left rear (-y and -x)
  legPos[LEG_3][X]=-4.8;
  legPos[LEG_3][Y]=-4.8;
  legPos[LEG_3][Z]=7.1;
  legAngle[LEG_3][COXA]=-135.0;
  legAngle[LEG_3][FEMUR]=89.0;
  legAngle[LEG_3][TIBIA]=90.0;
    
  //Starting pos/angle - ]Right rear (+y and -x)
  legPos[LEG_4][X]=-4.8;
  legPos[LEG_4][Y]=4.8;
  legPos[LEG_4][Z]=7.1;
  legAngle[LEG_4][COXA]=135.0;
  legAngle[LEG_4][FEMUR]=89.0;
  legAngle[LEG_4][TIBIA]=90.0;

  //Set initial leg positions
  for(int i=0;i<LEGS;i++){
    for(int j=0;j<LEG_SERVOS;j++){
      pwm.setPWM(servoPin[i][j], 0, map(legAngle[i][j], servoDegMin[i][j], servoDegMax[i][j], pwmMin[i][j], pwmMax[i][j]));
    }
  }
  
  yield();
}


void loop() {

  //Wait for commands
  processCommands();
  
}

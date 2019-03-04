/*********************************************************************
* CS467 - Quadruped Movement and Input Processing
* Author: Daniel Jarc (jarcd)
* Date: March 18, 2019
* Description: Processes user input, calculates servo angles
* using IK, executes commands. 
*********************************************************************/

#include "Quadruped.h"
#include "SoftwareSerial.h"

Quadruped::Quadruped(SoftwareSerial * bt_module) {

  //Bluetooth initialization
  bt = bt_module;
  bt->begin(9600);
  bt->println("HC-06 started"); 
}

/*********************************************************************
* initialize()
* Description: Sets default pin mappings, pwm min and max values,
* servo min and max angle values.
*********************************************************************/
void Quadruped::initialize(){

  //Used for printing quadruped info while connected
  Serial.begin(9600);

  //Servo shield initialization
  pwm.begin();
  pwm.setPWMFreq(60);
  
  //Servo pin mapping 
  servoPin[LEG_1][COXA]=12;
  servoPin[LEG_1][FEMUR]=13;
  servoPin[LEG_1][TIBIA]=14;
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
  pwmMin[LEG_1][TIBIA]=215;
  pwmMax[LEG_1][TIBIA]=500; 
  pwmMin[LEG_2][COXA]=200;
  pwmMax[LEG_2][COXA]=450;
  pwmMin[LEG_2][FEMUR]=180;
  pwmMax[LEG_2][FEMUR]=450;
  pwmMin[LEG_2][TIBIA]=180;
  pwmMax[LEG_2][TIBIA]=430;  
  pwmMin[LEG_3][COXA]=230;
  pwmMax[LEG_3][COXA]=470;
  pwmMin[LEG_3][FEMUR]=200;
  pwmMax[LEG_3][FEMUR]=430;
  pwmMin[LEG_3][TIBIA]=250;
  pwmMax[LEG_3][TIBIA]=500;
  pwmMin[LEG_4][COXA]=210;
  pwmMax[LEG_4][COXA]=440;
  pwmMin[LEG_4][FEMUR]=250;
  pwmMax[LEG_4][FEMUR]=490;
  pwmMin[LEG_4][TIBIA]=170;
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

  //Center body
  center();
  moveServos();
  yield(); 
}

/*********************************************************************
* processCommands()
* Description: Reads a byte from serial then executes the appropriate
* command. 
*********************************************************************/
void Quadruped::processCommands(){

  if (bt->available() > 0) {
    char c = bt->read();
    switch(c){
      case '`':                       //Print quadruped info
        printQuadInfo();
        break;
      case ',':                       //Sit
        sit();
        break;
      case '.':                       //Stand
        stand();
        break;
      case '/':                       //Center body
        center();
        moveServos();
        break;
      case 'q':                       //Translate body upward
        translateBody(0.0,0.0,1.0);
        break;
      case 'e':                       //Translate body downward
        translateBody(0.0,0.0,-1.0);
        break;
      case 'w':                       //Translate body forward
        translateBody(1.0,0.0,0.0);
        break;
      case 's':                       //Translate body back
        translateBody(-1.0,0.0,0.0);
        break; 
      case 'a':                       //Translate body left
        translateBody(0.0,-1.0,0.0);
        break;
      case 'd':                       //Translate body right
        translateBody(0.0,1.0,0.0);
        break;
      case 'u':                       //Yaw body left
        rotateBody(0.0,0.0,-10.0);
        break;
      case 'o':                       //Yaw body right
        rotateBody(0.0,0.0,10.0);
        break;
      case 'i':                       //Pitch body down
        rotateBody(0.0,-10.0,0.0);
        break; 
      case 'k':                       //Pitch body up
        rotateBody(0.0,10.0,0.0);
        break;
      case 'j':                       //Roll body left
        rotateBody(-10.0,0.0,0.0);
        break;
      case 'l':                       //Roll body right
        rotateBody(10.0,0.0,0.0);
        break;
      case 't':                       //Walk forward
        walk(FORWARD);
        break;
      case 'f':                       //Walk left
        walk(LEFT);
        break;   
      case 'h':                       //Walk right
        walk(RIGHT);
        break;
      case 'g':                       //Walk back
        walk(BACK);
        break;
      case 'r':                       //Turn left
        turn(LEFT);
        break;
      case 'y':                       //Turn right
        turn(RIGHT);
        break;
      default:
        break;
    }
  }
}

/*********************************************************************
* printQuadInfo()
* Description: Prints current quadruped body and leg data.
*********************************************************************/
void Quadruped::printQuadInfo(){  
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
  Serial.println("--------");
  Serial.println("Leg Info");
  Serial.println("--------");
  for (int i = 0; i < LEGS; i++){
    Serial.print("Leg ");
    Serial.print(i);
    Serial.print(" - ");
    Serial.print("X: ");
    Serial.print(legPos[i][X]); 
    Serial.print(", Y: ");
    Serial.print(legPos[i][Y]);
    Serial.print(", Z: ");
    Serial.print(legPos[i][Z]); 
    Serial.print(", Coxa Angle: ");
    Serial.print(legAngle[i][COXA]);
    Serial.print(", Femur Angle: ");
    Serial.print(legAngle[i][FEMUR]);  
    Serial.print(", Tibia Angle: ");
    Serial.println(legAngle[i][TIBIA]);  
  }
}

/*********************************************************************
* translateBody()
* Description: Shifts the body based on x,y and z input.
* @param dX - distance in x direction (cm)
* @param dY - distance in y direction (cm)
* @param dZ - distance in z direction (cm)
*********************************************************************/
void Quadruped::translateBody(float dX, float dY, float dZ) {

  float bodyX = bodyPos[X] + dX;
  float bodyY = bodyPos[Y] + dY;
  float bodyZ = bodyPos[Z] + dZ;
  
  //Check within translation constraints
  bool valid = (abs(bodyX) <= 4.0f && abs(bodyY) <= 4.0f && abs(bodyZ) <= 4.0f);
  
  if (valid) {
  
    struct Solution sol;
    
    //Get new leg coordinates   
    for(int i=0;i<LEGS;i++){
      float newX = legPos[i][X] - dX;
      float newY = legPos[i][Y] - dY;
      float newZ = legPos[i][Z] + dZ;
      sol.legPos[i][X]=newX;
      sol.legPos[i][Y]=newY;
      sol.legPos[i][Z]=newZ;
    }
    
    //Calculate leg IK using values
    legIK(&sol);
    
    //Set associated body values
    bodyPos[X]=bodyX;
    bodyPos[Y]=bodyY;
    bodyPos[Z]=bodyZ;

    //Set leg values from solution
    setLegValues(&sol);
    
    //Actually move the servos   
    moveServos();
    
  } else {
    Serial.println("Body has reached a translation limit.");   
  }  
}

/*********************************************************************
* rotateBody()
* Description: Rotates the body based on pitch, roll and yaw changes.
* @param dU - roll (degrees)
* @param dV - pitch (degrees)
* @param dW - yaw (degrees)
*********************************************************************/
void Quadruped::rotateBody(float dU, float dV, float dW) {

  float bodyU = bodyAngle[ROLL] + dU;
  float bodyV = bodyAngle[PITCH] + dV;
  float bodyW = bodyAngle[YAW] + dW;
  
  //Check within body angle constraints
  bool valid = (abs(bodyU) <= 20.0f && abs(bodyV) <= 20.0f && abs(bodyW) <= 45.0f);

  if (valid) {

    //Convert to radians
    float u = bodyU*DEG_TO_RAD;
    float v = bodyV*DEG_TO_RAD;
    float w = bodyW*DEG_TO_RAD;
    
    //Construct body rotation matrix R
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
    center();
    
    //Get new leg values
    for(int i=0;i<LEGS;i++){

      float x0 = legPos[i][X];
      float y0 = legPos[i][Y];
      float z0 = legPos[i][Z];

      float x1 = x0*R11 + y0*R12 + z0*R13;
      float y1 = x0*R21 + y0*R22 + z0*R23;
      float z1 = x0*R31 + y0*R32 + z0*R33;
      
      Serial.print("Leg ");
      Serial.print(i);
      Serial.print(" x1 ");
      Serial.print(x1);
      Serial.print(" y1 ");
      Serial.print(y1);
      Serial.print(" z1 ");
      Serial.println(z1);
      
      float dX = x0 - x1;
      float dY = y0 - y1;
      float dZ = z0 - z1;

      float newX = x0 + dX; 
      float newY = y0 + dY;
      float newZ = z0 + dZ;

      
      sol.legPos[i][X]=newX;
      sol.legPos[i][Y]=newY;
      sol.legPos[i][Z]=newZ;
    }

    //Calculate leg IK using values
    legIK(&sol);
      
    //Set associated body values
    bodyAngle[ROLL]=bodyU;
    bodyAngle[PITCH]=bodyV;
    bodyAngle[YAW]=bodyW;

    //Set leg values from solution
    setLegValues(&sol);

    //Actually move the servos
    moveServos();
    
  } else {  
    Serial.println("Body has reached a rotation limit.");   
  }
}

/*********************************************************************
* walk()
* Description: Executes one full walk cycle in the desired direction.
* @param dir - FORWARD, BACKWARD, LEFT or RIGHT
*********************************************************************/
void Quadruped::walk(int dir) {

  int dt = 200;         //Step delay
  float dX;             //Body movement in x (1/2 total x travel)
  float dY;             //Body movement in y (1/2 total y travel)
  float dZ = 2.0f;      //Step height (same for all legs)
  int legSeq[8];        //Leg movement sequence

  switch(dir) {
    case FORWARD:
      dX = 2.0f;
      dY = 0.0f;
      legSeq[0]=LEG_3;
      legSeq[1]=LEG_3;
      legSeq[2]=LEG_2;
      legSeq[3]=LEG_2;
      legSeq[4]=LEG_4;
      legSeq[5]=LEG_4;
      legSeq[6]=LEG_1;
      legSeq[7]=LEG_1;
      break;
   case BACK:
      dX = -2.0f;
      dY = 0.0f;
      legSeq[0]=LEG_1;
      legSeq[1]=LEG_1;
      legSeq[2]=LEG_4;
      legSeq[3]=LEG_4;
      legSeq[4]=LEG_2;
      legSeq[5]=LEG_2;
      legSeq[6]=LEG_3;
      legSeq[7]=LEG_3;
      break; 
    case LEFT: 
      dX = 0.0f;
      dY = -2.0f;
      legSeq[0]=LEG_4;
      legSeq[1]=LEG_4;
      legSeq[2]=LEG_3;
      legSeq[3]=LEG_3;
      legSeq[4]=LEG_1;
      legSeq[5]=LEG_1;
      legSeq[6]=LEG_2;
      legSeq[7]=LEG_2;
      break;
    case RIGHT:
      dX = 0.0f;
      dY = 2.0f;
      legSeq[0]=LEG_2;
      legSeq[1]=LEG_2;
      legSeq[2]=LEG_1;
      legSeq[3]=LEG_1;
      legSeq[4]=LEG_3;
      legSeq[5]=LEG_3;
      legSeq[6]=LEG_4;
      legSeq[7]=LEG_4;
      break; 
    default:
      break;
  }

  //Start centered
  center();
  moveServos();
  
  //Move body then two legs, then body, then remaining legs
  translateBody(dX,dY,0.0f);
  delay(dt);
  moveLeg(legSeq[0],2*dX,2*dY,-dZ);
  delay(dt);
  moveLeg(legSeq[1],0.0f,0.0f,dZ);
  delay(dt);
  moveLeg(legSeq[2],2*dX,2*dY,-dZ);
  delay(dt);
  moveLeg(legSeq[3],0.0f,0.0f,dZ);
  delay(dt);  
  translateBody(dX,dY,0.0f);
  delay(dt);
  moveLeg(legSeq[4],2*dX,2*dY,-dZ);
  delay(dt);
  moveLeg(legSeq[5],0.0f,0.0f,dZ);
  delay(dt);
  moveLeg(legSeq[6],2*dX,2*dY,-dZ);
  delay(dt);
  moveLeg(legSeq[7],0.0f,0.0f,dZ);
  delay(dt);
}

/*********************************************************************
* turn()
* Description: Turns the quadruped by stepping and then rotating in 
* the desired direction.
* @param dir - LEFT or RIGHT
*********************************************************************/
void Quadruped::turn(int dir) {
  
  int dt = 200;       //Step delay
  float dW = 30.0f;   //Degrees yaw per step cycle
  float dZ = 2.0f;    //Step height

  //Start centered
  center();
  moveServos();
  
  switch(dir) {
    case LEFT:
      moveLeg(LEG_1,0.0,-4.0,-dZ);
      delay(dt);
      moveLeg(LEG_1,0.0,0.0,dZ);
      delay(dt);
      moveLeg(LEG_2,-4.0,0.0,-dZ);
      delay(dt);
      moveLeg(LEG_2,0.0,0.0,dZ);
      delay(dt);
      moveLeg(LEG_3,0.0,4.0,-dZ);
      delay(dt);
      moveLeg(LEG_3,0.0,0.0,dZ);
      delay(dt);
      moveLeg(LEG_4,4.0,0.0,-dZ);
      delay(dt);
      moveLeg(LEG_4,0.0,0.0,dZ);
      delay(dt);
      center();
      moveServos();
      break;
    case RIGHT: 
      rotateBody(0.0f,0.0f,dW);
      moveLeg(LEG_1,-4.0,0.0,-dZ);
      delay(dt);
      moveLeg(LEG_1,0.0,0.0,dZ);
      delay(dt);
      moveLeg(LEG_2,0.0,4.0,-dZ);
      delay(dt);
      moveLeg(LEG_2,0.0,0.0,dZ);
      delay(dt);
      moveLeg(LEG_3,4.0,0.0,-dZ);
      delay(dt);
      moveLeg(LEG_3,0.0,0.0,dZ);
      delay(dt);
      moveLeg(LEG_4,0.0,-4.0,-dZ);
      delay(dt);
      moveLeg(LEG_4,0.0,0.0,dZ);
      delay(dt);
      center();
      moveServos();
      break;
    default: 
      break; 
  }
}

/*********************************************************************
* sit()
* Description: Transitions from a standing posture to sitting posture. 
*********************************************************************/
void Quadruped::sit() {
  center();
  moveServos();
  translateBody(0.0f,0.0f,-4.0f);
}

/*********************************************************************
* stand()
* Description: Transitions from a sitting posture to standing posture. 
*********************************************************************/
void Quadruped::stand() {
  center();
  moveServos();
}

/*********************************************************************
* center()
* Description: Centers body/leg positions and angles.
*********************************************************************/
void Quadruped::center() {

  for (int i=0;i<3;i++){
    bodyPos[i]=0.0f;
    bodyAngle[i]=0.0f;
  }
  
  legPos[LEG_1][X]=4.8;
  legPos[LEG_1][Y]=4.8;
  legPos[LEG_1][Z]=7.1;
  legAngle[LEG_1][COXA]=45.0;
  legAngle[LEG_1][FEMUR]=89.0;
  legAngle[LEG_1][TIBIA]=90.0;
  legPos[LEG_2][X]=4.8;
  legPos[LEG_2][Y]=-4.8;
  legPos[LEG_2][Z]=7.1;
  legAngle[LEG_2][COXA]=-45.0;
  legAngle[LEG_2][FEMUR]=89.0;
  legAngle[LEG_2][TIBIA]=90.0;
  legPos[LEG_3][X]=-4.8;
  legPos[LEG_3][Y]=-4.8;
  legPos[LEG_3][Z]=7.1;
  legAngle[LEG_3][COXA]=-135.0;
  legAngle[LEG_3][FEMUR]=89.0;
  legAngle[LEG_3][TIBIA]=90.0;
  legPos[LEG_4][X]=-4.8;
  legPos[LEG_4][Y]=4.8;
  legPos[LEG_4][Z]=7.1;
  legAngle[LEG_4][COXA]=135.0;
  legAngle[LEG_4][FEMUR]=89.0;
  legAngle[LEG_4][TIBIA]=90.0;
}

/*********************************************************************
* legIK()
* Description: Calculates a final IK solution for all legs based on 
* position data in solution struct. Fills in angle values in solution.
*********************************************************************/
void Quadruped::legIK(struct Solution* sol) {

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
   
  //Iterate through each leg
  for (int i=0; i<LEGS;i++){

    x = sol->legPos[i][X];
    y = sol->legPos[i][Y];
    z = sol->legPos[i][Z];

    //Calculate theta1, theta2 and theta3 based on leg x, y and z coordinate
    theta1 = atan2(y,x);
    L0 = sqrt((x*x) + (y*y));
    L1 = L0 - COXA_LENGTH;
    L2 = sqrt((L1*L1) + (z*z));
    phi1 = atan(L1/z);
    phi2 = acos(((L2*L2) + (FEMUR_LENGTH*FEMUR_LENGTH) - (TIBIA_LENGTH*TIBIA_LENGTH)) / (2 * L2 * FEMUR_LENGTH));
    beta = acos(((FEMUR_LENGTH*FEMUR_LENGTH) + (TIBIA_LENGTH*TIBIA_LENGTH) - (L2*L2)) / (2 * FEMUR_LENGTH * TIBIA_LENGTH));
    theta2 = phi1 + phi2;
    theta3 = PI - beta;

    //Set angle values in solution set
    sol->legAngle[i][COXA] = theta1*RAD_TO_DEG;
    sol->legAngle[i][FEMUR] = theta2*RAD_TO_DEG;
    sol->legAngle[i][TIBIA] = theta3*RAD_TO_DEG;
  }
}

/*********************************************************************
* setLegValues()
* Description: Sets leg values to values contained in Solution struct.
* @param sol - Solution struct with desired leg angles/positions.
*********************************************************************/
void Quadruped::setLegValues(struct Solution *sol) {
  for (int i=0;i<LEGS;i++){
    legPos[i][X]=sol->legPos[i][X];
    legPos[i][Y]=sol->legPos[i][Y];
    legPos[i][Z]=sol->legPos[i][Z];
    legAngle[i][COXA]=sol->legAngle[i][COXA];
    legAngle[i][FEMUR]=sol->legAngle[i][FEMUR];
    legAngle[i][TIBIA]=sol->legAngle[i][TIBIA];
  }
}

/*********************************************************************
* moveLeg()
* Description: Calculates angles and moves a single leg. 
* @param leg - LEG_1, LEG_2, LEG_3 or LEG_4
* @param dX - distance in x (cm)
* @param dY - distance in y (cm)
* @param dZ - distance in z (cm)
*********************************************************************/
void Quadruped::moveLeg(int leg, float dX, float dY, float dZ) {

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

  x = legPos[leg][X] + dX;
  y = legPos[leg][Y] + dY;
  z = legPos[leg][Z] + dZ;

  //Calculate theta1, theta2 and theta3 based on leg x, y and z coordinate
  theta1 = atan2(y,x);
  L0 = sqrt((x*x) + (y*y));
  L1 = L0 - COXA_LENGTH;
  L2 = sqrt((L1*L1) + (z*z));
  phi1 = atan(L1/z);
  phi2 = acos(((L2*L2) + (FEMUR_LENGTH*FEMUR_LENGTH) - (TIBIA_LENGTH*TIBIA_LENGTH)) / (2 * L2 * FEMUR_LENGTH));
  beta = acos(((FEMUR_LENGTH*FEMUR_LENGTH) + (TIBIA_LENGTH*TIBIA_LENGTH) - (L2*L2)) / (2 * FEMUR_LENGTH * TIBIA_LENGTH));
  theta2 = phi1 + phi2;
  theta3 = PI - beta;
  
  //Set values
  legPos[leg][X]=x;
  legPos[leg][Y]=y;
  legPos[leg][Z]=z;
  legAngle[leg][COXA] = theta1*RAD_TO_DEG;
  legAngle[leg][FEMUR] = theta2*RAD_TO_DEG;
  legAngle[leg][TIBIA] = theta3*RAD_TO_DEG;

  //Move
  moveServos();
}

/*********************************************************************
* moveServos()
* Description: Moves all servos to their set values.
*********************************************************************/
void Quadruped::moveServos(){
  for(int i=0;i<LEGS;i++){
    for(int j=0;j<LEG_SERVOS;j++){
      pwm.setPWM(servoPin[i][j], 0, map(legAngle[i][j], servoDegMin[i][j], servoDegMax[i][j], pwmMin[i][j], pwmMax[i][j]));
    }
  }
}

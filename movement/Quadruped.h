/*********************************************************************
* CS467 - Quadruped Movement and Input Processing
* Author: Daniel Jarc (jarcd)
* Date: March 1, 2019
* Description: Processes user input to serial, calculates servo angles
* using IK, executes commands. 
*********************************************************************/

#ifndef QUADRUPED_H
#define QUADRUPED_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "SoftwareSerial.h"

//Definitions
#define FORWARD 0
#define BACK 1
#define LEFT 2
#define RIGHT 3
#define X 0            
#define Y 1             
#define Z 2             
#define ROLL 0
#define PITCH 1
#define YAW 2
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

//Solution struct is used to store IK solutions
struct Solution {
  float legAngle[4][3];
  float legPos[4][3];
};

class Quadruped {

  public:
    Quadruped(SoftwareSerial* bt_module);
    void initialize();
    void processCommands();
    void printQuadInfo();
    
  private:
    void translateBody(float dX, float dY, float dZ);
    void rotateBody(float dU, float dV, float dW);
    void walk(int dir);
    void turn(int dir);
    void sit();
    void stand();
    void center();
    void legIK(struct Solution* sol);
    void setLegValues(struct Solution* sol);
    void moveLeg(int leg, float dX, float dY, float dZ);
    void moveServos();

    Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
    SoftwareSerial* bt;
    float bodyAngle[3];
    float bodyPos[3];
    float legAngle[4][3];
    float legPos[4][3];
    int pwmMin[4][3];
    int pwmMax[4][3];
    int servoPin[4][3];
    int servoDegMin[4][3];
    int servoDegMax[4][3];
};
#endif

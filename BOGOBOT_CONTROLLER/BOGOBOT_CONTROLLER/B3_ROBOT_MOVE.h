#ifndef B3_ROBOT_MOVE
#define B3_ROBOT_MOVE

#include "DXL_MX_ALL_MOTORS.h"
#include "B3_POS.h"
#include "INVERSE_KINEMATICS.h"
#include "FORWARD_KINEMATICS.h"

double vel[18] = {};

/*  MOVES BOGOBOT 3 ONE POSTURE AT THE TIME
 *   
 *  Available functions:
 * 
 *  void moveRobot_byPose()........Moves robot with given coordinates (x,y,z).......................................OK
 *  void IK_RightLeg().............Moves robot using articular values in bits (q)...................................OK
 *  
 */
 
void moveRobot_byPose(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
               dynamixel::GroupSyncWrite groupSyncWritePos, dynamixel::GroupSyncWrite groupSyncWriteVel,
               double IDs[18], double p[16]){

  // desiered positions for:
  //                       X       Y       Z       alpha
  //    LEFT LEG  (LL):    p[0]    p[1]    p[2]    p[3]
  //    RIGHT LEG (RL):    p[4]    p[5]    p[6]    p[7]
  //    LEFT ARM  (LA):    p[8]    p[9]    p[10]
  //    RIGHT ARM (RA):    p[11]   p[12]   p[13]

  // timigs:  
  //    PLAYTIME :  p[14]
  //    DELAYTIME : p[15]

  // calculate q of the robot with IK
  double LL[4] = {p[0],p[1],p[2],p[3]};
  double RL[4] = {p[4],p[5],p[6],p[7]};
  double LA[3] = {p[8],p[9],p[10]};
  double RA[3] = {p[11],p[12],p[13]};

  IK_LeftLeg(LL, q,1);
  IK_RightLeg(RL, q,1);
  //IK_RightArm(RA,q);

  // stablish playtime and delay
  q[18] = p[14]; // playtime
  q[19] = p[15]; // delay

  // get the robot current position
  getPositionAll(portHandler,packetHandler, IDs, q0);

  // calculate and set desiered speed
  calcMotorsSpeed(q0, q, vel,1);
  setVelocitySync(groupSyncWriteVel, packetHandler, IDs,vel);

  // set the robot position
  setPositionSync(groupSyncWritePos, packetHandler,IDs, q);
  // stop aftermovement
  //delay(q[19]*1000);
}

void moveRobot_byQVals(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
               dynamixel::GroupSyncWrite groupSyncWritePos, dynamixel::GroupSyncWrite groupSyncWriteVel,
               double IDs[18], double q[20]){
  
  // get the robot current position
  getPositionAll(portHandler,packetHandler, IDs, q0);

  // calculate and set desiered speed
  calcMotorsSpeed(q0, q, vel,1);
  setVelocitySync(groupSyncWriteVel, packetHandler, IDs,vel);

  // set the robot position
  setPositionSync(groupSyncWritePos, packetHandler,IDs, q);

  // stop aftermovement
  //(q[19]*1000);
}

void moveRobot_byJacobian(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
               dynamixel::GroupSyncWrite groupSyncWritePos, dynamixel::GroupSyncWrite groupSyncWriteVel,
               double IDs[18], double setPoint[6], double pt, double giro[2], double correctHip[2]){

  // set points positions for:
  //                           X              Y               Z
  //    LEFT LEG  (LL):    setPoint[0]    setPoint[1]    setPoint[2]
  //    RIGHT LEG (RL):    setPoint[3]    setPoint[4]    setPoint[5]

  // timigs:  
  //    PLAYTIME :  pt

  // angle for leg
  //    ANGLE RIGHT LEG:  giro[0]
  //    ANGLE LEFT LEG:   giro[1]

  // correction for hip
  //    CORRECTION RIGHT LEG: correctHip[0]
  //    CORRECTION LEFT LEG: correctHip[1]

  // define the pose setpoint vectors
  BLA::Matrix<6,1,double> r_Leg_POSE_SETPOINT;
  r_Leg_POSE_SETPOINT(0,0) = setPoint[3]; // x right leg
  r_Leg_POSE_SETPOINT(1,0) = setPoint[4]; // y right leg
  r_Leg_POSE_SETPOINT(2,0) = setPoint[5]; // z right leg
  r_Leg_POSE_SETPOINT(3,0) = 0; // alpha
  r_Leg_POSE_SETPOINT(4,0) = 0; // theta
  r_Leg_POSE_SETPOINT(5,0) = giro[0]; // gamma

  BLA::Matrix<6,1,double> l_Leg_POSE_SETPOINT;
  l_Leg_POSE_SETPOINT(0,0) = setPoint[0]; // x left leg
  l_Leg_POSE_SETPOINT(1,0) = setPoint[1]; // y left leg
  l_Leg_POSE_SETPOINT(2,0) = setPoint[2]; // z left leg
  l_Leg_POSE_SETPOINT(3,0) = 0; // alpha
  l_Leg_POSE_SETPOINT(4,0) = 0; // theta
  l_Leg_POSE_SETPOINT(5,0) = giro[1]; // gamma

  // get the robot motor values from DXL
  getPositionAll(portHandler,packetHandler, IDs, q0); // read the robot's current position
  double qR[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // array to store the robots current position in radians
  for (int i = 0; i<18; i = i+1){qR[i] = bits2rad(q0[i], configPos_QVals[i]);} // transform the readings from bits to radians 

  // divide the qR values for each leg
  double qRight_Leg[6] = {qR[6],qR[8],qR[10],qR[12],qR[14],qR[16]}; //q7 q9 q11q q13 q15 q17
  double qLeft_Leg[6]  = {qR[7],qR[9],qR[11],qR[13],qR[15],qR[17]}; //q8 q10 q12q q14 q16 q18

  BLA::Matrix<6,1,double> r_Leg_qNOW;
  r_Leg_qNOW(0,0) = qR[6];
  r_Leg_qNOW(1,0) = qR[8];
  r_Leg_qNOW(2,0) = qR[10];
  r_Leg_qNOW(3,0) = qR[12];
  r_Leg_qNOW(4,0) = qR[14];
  r_Leg_qNOW(5,0) = qR[16];

  BLA::Matrix<6,1,double> l_Leg_qNOW;
  l_Leg_qNOW(0,0) = qR[7];
  l_Leg_qNOW(1,0) = qR[9];
  l_Leg_qNOW(2,0) = qR[11];
  l_Leg_qNOW(3,0) = qR[13];
  l_Leg_qNOW(4,0) = qR[15];
  l_Leg_qNOW(5,0) = qR[17];

  // use the forward kinematics to determine the current pose of the robot
  BLA::Matrix<6,1,double> r_Leg_POSE = getRightLeg_POSE(qRight_Leg);
  BLA::Matrix<6,1,double> l_Leg_POSE = getLeftLeg_POSE(qLeft_Leg);

  // get the jacobians for each leg
  BLA::Matrix<6,6,double> r_Leg_JACOBIAN = rightLeg_Jacobian(qRight_Leg);
  BLA::Matrix<6,6,double> l_Leg_JACOBIAN = leftLeg_Jacobian(qLeft_Leg);

  // invert the jacobians
  BLA::Matrix<6,6,double> r_Leg_INV_JACO = Inverse(r_Leg_JACOBIAN);
  BLA::Matrix<6,6,double> l_Leg_INV_JACO = Inverse(l_Leg_JACOBIAN);

  // calculate the error between the setpoint and the current pose
  BLA::Matrix<6,1,double> r_Leg_ERROR = r_Leg_POSE - r_Leg_POSE_SETPOINT;
  BLA::Matrix<6,1,double> l_Leg_ERROR = l_Leg_POSE - l_Leg_POSE_SETPOINT;

  // Serial.println("Current Values for Right Leg Error");
  // for (int i = 0; i<6; i = i+1){Serial.println(r_Leg_ERROR(i,0));}

  // Serial.println("Calculated Jacobian");
  // printJacobian6x6(r_Leg_JACOBIAN);

  // Serial.println("Inverted Jacobian");
  // printJacobian6x6(r_Leg_INV_JACO); r_Leg_qNOW - 
  
  // get the new values for the robot
  BLA::Matrix<6,1,double> r_Leg_qNEW = r_Leg_qNOW - r_Leg_INV_JACO*r_Leg_ERROR;
  //BLA::Matrix<6,1,double> tmp = r_Leg_INV_JACO*r_Leg_ERROR;
  BLA::Matrix<6,1,double> l_Leg_qNEW = l_Leg_qNOW - l_Leg_INV_JACO*l_Leg_ERROR;

  double qF[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  
  // arms stay the same
  qF[0] = q0[0];    qF[1] = q0[1]; 
  qF[2] = q0[2];    qF[3] = q0[3];
  qF[4] = q0[4];    qF[5] = q0[5];

  // legs change to new pose
  // right leg
  qF[6] = rad2bits(r_Leg_qNEW(0,0), configPos_QVals[6]); // cadera este puede girar
  qF[8] = rad2bits(r_Leg_qNEW(1,0), configPos_QVals[8]) - correctHip[0];
  qF[10] = rad2bits(r_Leg_qNEW(2,0), configPos_QVals[10]);
  qF[12] = rad2bits(r_Leg_qNEW(3,0), configPos_QVals[12]);
  qF[14] = rad2bits(r_Leg_qNEW(4,0), configPos_QVals[14]);
  qF[16] = rad2bits(r_Leg_qNEW(5,0), configPos_QVals[16]) + ceil(correctHip[0]/5);

  // left leg
  qF[7] = rad2bits(l_Leg_qNEW(0,0), configPos_QVals[7]); // cadera este puede girar
  qF[9] = rad2bits(l_Leg_qNEW(1,0), configPos_QVals[9]) + correctHip[1];
  qF[11] = rad2bits(l_Leg_qNEW(2,0), configPos_QVals[11]);
  qF[13] = rad2bits(l_Leg_qNEW(3,0), configPos_QVals[13]);
  qF[15] = rad2bits(l_Leg_qNEW(4,0), configPos_QVals[15]);
  qF[17] = rad2bits(l_Leg_qNEW(5,0), configPos_QVals[17]) - ceil(correctHip[1]/5);

  qF[18] = pt; // playtime
  qF[19] = 0; // delay

  // calculate and set desiered speed
  calcMotorsSpeed(q0, qF, vel,1);
  setVelocitySync(groupSyncWriteVel, packetHandler, IDs,vel);

  // set the robot position
  //Serial.println("New values for right leg");
  // for (int i = 0; i<6; i = i+1){
  //   Serial.print(r_Leg_qNOW(i,0));
  //   Serial.print("  ");
  //   Serial.print(tmp(i,0));
  //   Serial.print(" = ");
  //   Serial.println(r_Leg_qNEW(i,0));
  // }
  setPositionSync(groupSyncWritePos, packetHandler,IDs, qF);
  // stop aftermovement
  //delay(q[19]*1000);
}
#endif
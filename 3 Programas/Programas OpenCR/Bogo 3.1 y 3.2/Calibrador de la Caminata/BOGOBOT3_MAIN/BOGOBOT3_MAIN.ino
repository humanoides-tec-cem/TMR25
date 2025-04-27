/*  BOGOBOT 3 PROGRAM
    Written by: Ana Patricia Islas Mainou
 */
 
#include "DXL_MX_ALL_MOTORS.h"
#include "DXL_MX.h"
#include "B3_POS.h"
#include "INVERSE_KINEMATICS.h"
#include "FORWARD_KINEMATICS.h"
#include "B3_ROBOT_MOVE.h"
#include "ZMP.h"
#include "JACOBIANS.h"
#include <DynamixelSDK.h>
#include <BasicLinearAlgebra.h>
#include "CMD_VEL_MOVE.h"

// Define dynamixel IDs
double IDs[18] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
double torque = 1;

void setup() {

  // Initialize PortHandler instance
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  
  // Initialize PacketHandler instance
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  
  // Initialize GroupSyncWrite instances
  dynamixel::GroupSyncWrite groupSyncWritePos(portHandler, packetHandler, 30, LEN_MX_GOAL_POSITION); // for position
  dynamixel::GroupSyncWrite groupSyncWriteVel(portHandler, packetHandler, 32, LEN_MX_GOAL_POSITION); // for velocity
  
  // Initialize BulkRead instances
  dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler); // present pos
  
  Serial.begin(115200);
  
  // Start Communication and Enable Torque
  if (startCom(portHandler,packetHandler,BAUDRATE)) {
    Serial.println("Start...");
  } else {
    return;
  }

  for (int i=0; i<18; i++){
  pingMotor(portHandler,packetHandler,i);}
  
  setTorqueAll(portHandler,packetHandler,IDs, torque);

  // MAIN ----------------------------------------------------------------------------------  
  //                 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0
  // double step[20] = {1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,3};
  // // moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,configPos_QVals);
  // // delay(2000);
  // moveRobot_byPose(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,walk_TaskS);
  // delay(2000);
  // // //Serial.println("caminar");
  // // for (int i=0;  i<1; i++){
  // // // este ya jala NO MOVER -------------------------------------------------------------------
  // // //walkSeq_Jacobian(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,step,3.5,0.4,9.5,3);
  // // //walkSeq(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,step,3.5,0.4,9.5,10.8);
  // // }

  // // double setPoint[6] = {0, 0, -25, 0, 0, -25};
  // // double pt = 1;
  // // double giro[2] = {0,PI/8};
  // // moveRobot_byJacobian(portHandler, packetHandler, groupSyncWritePos, groupSyncWriteVel, IDs, setPoint, pt, giro);
  // // delay(5000);

  // // giro[0]=PI/8; giro[1]=0;
  // // moveRobot_byJacobian(portHandler, packetHandler, groupSyncWritePos, groupSyncWriteVel, IDs, setPoint, pt, giro);
  // // delay(5000);

  // moveRobot_byPose(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs, walk_TaskS);
  // delay(5000);

  // // CAMINATA NORMAL ------------------------

  // double gamma = 0;//PI/32;
  // for (int i=0;  i<5; i++){
  //   walkSeq_Jacobian(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel, IDs, step, 20, 3.5, 0.4, 7, 8.8, gamma);
  // }

  // gamma = 0;//-PI/32;
  // for (int i=0;  i<5; i++){
  //   walkSeq_Jacobian(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel, IDs, step, 20, 3.5, 0.4, 7, 8.8, gamma);
  // }

  // // delay(5000);

  // // CAMINATA CON CMD VEL ------------------------
  // // for (int i=0;  i<10; i++){
  // //   moveRobot_cmd_vel(portHandler, packetHandler, groupSyncWritePos, groupSyncWriteVel, IDs);
  // // }

  // delay(5000);
  // moveRobot_byPose(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs, walk_TaskS);
  // delay(5000);

  // Stop Communication and Disable Torque ---------------------------------------------------------------------------
  // moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,sitPos_QVals);
  delay(500);
  torque = 0;
  setTorqueAll(portHandler,packetHandler,IDs, torque);
  Serial.println("Torque Off");
  delay(500);
  stopCom(portHandler,packetHandler);

}


void loop() {}

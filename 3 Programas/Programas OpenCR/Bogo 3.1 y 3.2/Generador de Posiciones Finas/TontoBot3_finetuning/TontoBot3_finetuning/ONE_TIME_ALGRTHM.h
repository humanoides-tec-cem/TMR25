#ifndef ONE_TIME_ALGRTHM
#define ONE_TIME_ALGRTHM

#include "B3_POS.h"
#include "B3_ROBOT_MOVE.h"



void Stand_Up_FaceUp(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
                   dynamixel::GroupSyncWrite groupSyncWritePos, dynamixel::GroupSyncWrite groupSyncWriteVel,
                   double IDs[18]){
  //  ------------------------------- P O S E S       P A R A R       B O C A         A R R I B A---------------------------------------------------------------
  // for(int pe = 0; pe < siz_pos_arriba-1 ; pe++){
  //   moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,poses_boca_arriba[pe]);
  // }
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_Acostado);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_Rodillas_Arriba);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_Baja_Rodillas);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_Sube_Torso);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_se_sienta);
  moveRobot_byPose(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,p2);

}

void Stand_Up_FaceDown(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
                   dynamixel::GroupSyncWrite groupSyncWritePos, dynamixel::GroupSyncWrite groupSyncWriteVel,
                   double IDs[18]){
  //--------------------------------- P O S E S       P A R A R         B O C A       A B A J O------------------------------------------------------------
  // for(int p=0; p < siz_pos_abajo-1 ;p++){
  //   moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,poses_boca_abajo[p]);
  // }
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_0);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_1);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_2);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_3);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_4);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_5);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_6);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_7);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_8);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_9);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_10);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_11);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_12);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_13);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_15);
  moveRobot_byPose(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,p2);
}

void kick_der(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
                   dynamixel::GroupSyncWrite groupSyncWritePos, dynamixel::GroupSyncWrite groupSyncWriteVel,
                   double IDs[18]){
  //--------------------------------- P O S E S       P A R A R         P A T E A R     C O N     D E R E C H A------------------------------------------------------------
  // for(int p=0; p < siz_patea_der-1 ;p++){
  //   moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,poses_patea_der[p]);
  // }
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_patea_1);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_patea_2);  
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_patea_3);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_patea_4);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_patea_5);
  moveRobot_byPose(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,p2);
}

#endif

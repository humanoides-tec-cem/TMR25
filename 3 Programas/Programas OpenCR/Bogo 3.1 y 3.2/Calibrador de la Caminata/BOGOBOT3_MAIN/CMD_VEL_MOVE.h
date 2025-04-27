#ifndef CMD_VEL_MOVE
#define CMD_VEL_MOVE

#include "ZMP.h"

void moveRobot_cmd_vel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
               dynamixel::GroupSyncWrite groupSyncWritePos, dynamixel::GroupSyncWrite groupSyncWriteVel, double IDs[18]){
  float Wz = 0.0; // angular.z
  float Vx = 1.0; // linear.x
  float Vy = 0.0; // linear.y
  float t = 0.4;

  double radio = 3.5;
  double x_zmp = 8;//min__(7,max__(Vx*t,-7));
  double y_zmp = 10.2;

  double giro = 0;

  double step[4] = {1,2,2,3}; double s = 4;

  walkSeq_Jacobian(portHandler, packetHandler, groupSyncWritePos, groupSyncWriteVel, IDs, step, s, radio, t, x_zmp, y_zmp, giro);
}
#endif
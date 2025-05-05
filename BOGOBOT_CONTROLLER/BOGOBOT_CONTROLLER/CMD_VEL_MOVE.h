#ifndef CMD_VEL_MOVE
#define CMD_VEL_MOVE

#include "ZMP.h"

void moveRobot_cmd_vel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
               dynamixel::GroupSyncWrite groupSyncWritePos, dynamixel::GroupSyncWrite groupSyncWriteVel, double IDs[18]){

  // float Wz = 0.0; // angular.z
  // float Vx = 1.0; // linear.x
  // float Vy = 0.0; // linear.y
  float t = 0.4;

  double radio = 3.5;
  double x_zmp = min_(8.5,max_(Vx*t*100,-8.5)); // Mover la cadera hacia adelante
  double y_zmp = 8.8; //8.8; // Máximo 10.2 // Mover la cadera hacia los lados

  double giro = min_(PI/18,max_(-PI/18,Wz*t*100)); // Si se pisa ponerle pi/18

  double step[4] = {1, 2, 2, 3}; double s = 4;// Medio paso derecho, pasos completos, medio paso para cerrar (izquierda)

  walkSeq_Jacobian(portHandler, packetHandler, groupSyncWritePos, groupSyncWriteVel, IDs, step, s, radio, 0.4, x_zmp, y_zmp, giro);
}
#endif
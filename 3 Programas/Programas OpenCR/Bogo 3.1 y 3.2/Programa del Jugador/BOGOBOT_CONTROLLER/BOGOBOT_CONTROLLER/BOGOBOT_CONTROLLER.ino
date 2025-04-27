// IMPORTAR LIBRERIAS NECESARIAS
#include "COMMUNICATION.h"
#include "DXL_MX_ALL_MOTORS.h"
#include "DXL_MX.h"
#include "B3_POS.h"
#include "INVERSE_KINEMATICS.h"
#include "FORWARD_KINEMATICS.h"
#include "B3_ROBOT_MOVE.h"
#include "ZMP.h"
#include "JACOBIANS.h"
#include "CMD_VEL_MOVE.h"
#include "ONE_TIME_ALGRTHM.h"
#include <DynamixelSDK.h>
#include <BasicLinearAlgebra.h>
#include <IMU.h>

// DEFINIR INTERVALO DE INTERRUPCION 
#define IMU_RATE 500000     
HardwareTimer Timer(TIMER_CH1);

// DEFINIR VARIABLES GLOBALES
double IDs[18] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18}; double torque = 1;

int roll, pitch, yaw;

cIMU    IMU;


void setup() {
  // INICIALIZAR TIMER DE INTERRUPCION  
  Timer.stop();
  Timer.setPeriod(IMU_RATE);    
  Timer.attachInterrupt(handler_IMU);
  Timer.start();


  pinMode(led_pin, OUTPUT);
  // INICIALIZAR VARIABLES PARA COMUNICACION CON LOS DXL ***************************************************************************************

  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  dynamixel::GroupSyncWrite groupSyncWritePos(portHandler, packetHandler, 30, LEN_MX_GOAL_POSITION); // for position
  dynamixel::GroupSyncWrite groupSyncWriteVel(portHandler, packetHandler, 32, LEN_MX_GOAL_POSITION); // for velocity
  dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler); // for present pos
  
  // INICIALIZAR SERIAL ************************************************************************************************************************
  Serial.begin(115200);

  // INICIALIZAR IMU ***************************************************************************************************************************
  IMU.begin();
  
  // CALIBRACION DEL IMU ***********************************************************************************************************************
  Serial.println("Calibrating IMU ...");
  IMU.SEN.acc_cali_start();
  while( IMU.SEN.acc_cali_get_done() == false ){
    IMU.update();
  }
  Serial.println("IMU Calibrated Correctly");

  // INICIALIZAR ROBOT *************************************************************************************************************************
  Serial.println("Starting robot...");

  Serial.println("Conecting motors..."); // -----------------------------------------------------------
  if (startCom(portHandler,packetHandler,BAUDRATE))
  {Serial.println("Motors Conected Correclty");} else {return;}
  
  Serial.println("Moving Robot to start position... "); // --------------------------------------------
  setTorqueAll(portHandler,packetHandler,IDs, torque);
  // cambiar la siguiente linea por una linea para levantar el robot 
  Stand_Up_FaceDown(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs);
  delay(5000);

  Serial.println("Robot started correctly");

  // CONTROLADOR GENERAL DEL ROBOT ************************************************************************************************************
  int standing  = 1;
  while (true){
    // Revisar la IMU para determinar como queda standing 
    if(pitch < 15){
      standing = 2;
      state = 6;

    }else if(pitch > 165){
      standing = 3;
      state = 6;

    }else{
      standing = 1;
    }

    // Segun lo que queda en standing seleccionar
    if (standing == 2){
      Serial.println("El robot esta boca abajo");
      Stand_Up_FaceDown(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs);

    } 
    if (standing == 3){
      Serial.println("El robot esta boca arriba");
      Stand_Up_FaceUp(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs);

    } 

    if (standing == 1){ // Cuando el robot esta erguido se revisa el valor del estado en el serial
      Serial.println("El robot esta erguido");
      if(Serial.available() > 0){
      readFromSerial(); // revisar comando en el serial para ver que debe hacer el robot
      //printMessage();
      }
    }
    // Segun el comando en el serial el robot hace lo siguiente

    if (state == 2){ // **************** CAMINAR USANDO CMD_VEL
    Serial.println("Caminar hacia la pelota usar CMD_VEL");
      moveRobot_cmd_vel(portHandler, packetHandler, groupSyncWritePos, groupSyncWriteVel, IDs);
    }

    if (state == 3){  // **************** PATEAR LA PELOTA
      Serial.println("Patear la pelota");
      kick_der(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs);

    }

    if (state == 0){ // **************** ROBOT ERGUIDO Y QUIETO
      Serial.println("Default erguido, quieto, stop");
      moveRobot_byPose(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,walk_TaskS_2);
    }
    
    if (state == 5){ // **************** APAGAR EL ROBOT 
      Serial.println("Stoppping robot...");
      moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,sitPos_QVals); delay(500);
      torque = 0; setTorqueAll(portHandler,packetHandler,IDs, torque); delay(500);
      stopCom(portHandler,packetHandler);
      Serial.println("Robot stopped correctly");
      return;
    }
  }
}

void handler_IMU(void) { 
  IMU.update();
  pitch = abs(IMU.rpy[0]);
  roll = abs(IMU.rpy[1]);
  yaw = abs(IMU.rpy[2]);  
}


void loop() {}

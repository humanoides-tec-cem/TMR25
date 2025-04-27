
#define IMU_RATE 800000     
HardwareTimer Timer(TIMER_CH1);
 
#include "DXL_MX_ALL_MOTORS.h"
#include "DXL_MX.h"
#include "B3_POS.h"
#include "B3_FK_IK.h"
#include "B3_ROBOT_MOVE.h"
#include "ZMP.h"
#include "ONE_TIME_ALGRTHM.h"
#include <DynamixelSDK.h>
#include <IMU.h>
cIMU    IMU;
uint8_t   err_code;
uint8_t   led_tog = 0;
uint8_t   led_pin = 13;
int pitch, roll, yaw;


// Define dynamixel IDs
double IDs[18] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
double torque = 1;

//variables para debuggear
double PosPosPos[18];  
double pose_recibida[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,10};

//loops para debuggear cosas y asi
/*SI ES FALSE, ESE CICLO ESTA INACTIVO
   SI ES TRUE EL CICLO ESTA ACTIVO*/

bool make_new_pose = true; //loop para capturar nuevas poses
bool abajo = false; //loop para testear OTA para que se pare boca abajo
bool arriba = false; //loop para testear OTA para que se pare boca arriba

bool kick_d = false; //loop de OTA patear con pierna derecha
bool kick_l =false; //loop de OTA patear con pierna izquierda

bool complete_test = false; //loop para testear one time algorithms con IMU

int pos_abajo = 0; //contadores para navegar entre poses 
int pos_arriba = 0;

int pos_kick_d;

void setup() {
  //IMU interrupt timer
  Timer.stop();
  Timer.setPeriod(IMU_RATE);           // in microseconds
  Timer.attachInterrupt(handler_IMU);
  Timer.start();

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
  IMU.begin();
  
  // Start Communication and Enable Torque
  if (startCom(portHandler,packetHandler,BAUDRATE)) {
    Serial.println("Start...");
  } else {
    return;
  }
  setTorqueAll(portHandler,packetHandler,IDs, torque);
/*LA IMU SE CALIBRA CON EL ROBOT ACOSTADO BOCA ABAJO
C A L I B R A R    C O N    E L    R O B O T     A C O S T A D O     B O C A    A B A J O*/
  //calibra la IMU
  Serial.println("ACC Cali Start");
  IMU.SEN.acc_cali_start();
  while( IMU.SEN.acc_cali_get_done() == false )
  {
    IMU.update();
  }
  Serial.print("ACC Cali End ");

  // MAIN ----------------------------------------------------------------------------------  
 //                 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0
  double step[20] = {1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,3};
  delay(2000);

/*SI SE QUIERE UTILIZAR LA IMU, SE TIENE QUE CALIBRAR CON EL ROBOT BOCA ABAJO
ENTONCES PRIMERO SE PARA BOCA ABAJO Y DESPÚES YA ESTÁ LISTO PA CHAMBEAR*/
  //Stand_Up_FaceDown(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs);
  moveRobot_byPose(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,p2);  
  delay(5000);

  //moveRobot_byPose(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,p2);
  //moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_15);

//--- C I C L O     P A R A       D E B U G G E A R      P O S E S-------------------------------------------
  while(make_new_pose){
    if(Serial.available() > 0){
      String data = Serial.readStringUntil('\n');

      if(data == "o"){ // apaga torque momentaneamente (o de off)
        Serial.println("apagando torques pa");
        torque = 0;
        setTorqueAll(portHandler,packetHandler,IDs, torque);
        delay(5000);
        torque = 1;
        setTorqueAll(portHandler,packetHandler,IDs, torque);

      }else if(data == "g"){ // obtiene la posicion actual "g" de GET
        Serial.println(" ");
        getPositionAll(portHandler,packetHandler, IDs, PosPosPos);
        Serial.println(" ");
        Serial.println(" ");

      }else if(data == "e"){ // "e" de END ---> accaba el ciclo para que se cierren las comunicaciones
        make_new_pose = false;
        torque = 0;
        setTorqueAll(portHandler,packetHandler,IDs, torque);

      }else if(data == "fine"){ // "e" de END ---> accaba el ciclo para que se cierren las comunicaciones
        actualizarPoseRecibida(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs);
      }
    }
  }

// T E S T      S E     P A R A     B O C A    A B A J O
  while(abajo){
    if(Serial.available() > 0){
      String data = Serial.readStringUntil('\n');

      if (data == "o"){//apaga torques momentaneamente
        torque = 0;
        setTorqueAll(portHandler,packetHandler,IDs, torque);
        delay(5000);
        torque = 1;
        setTorqueAll(portHandler,packetHandler,IDs, torque);

      }else if(data == "f"){ // se para boca abajo "f" de full asi como en realizalo todo completo
        Stand_Up_FaceDown(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs);

      }else if(data == "0"){ //pose inicial de la secuencia
        pos_abajo = 0;
        moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,poses_boca_abajo[pos_abajo]);
        Serial.println("pose inicial");

      }else if(data == "g"){ // obtiene la posicion actual
        Serial.println(" ");
        getPositionAll(portHandler,packetHandler, IDs, PosPosPos);
        Serial.println(" ");
        Serial.println(" ");

      }else if(data == "p2"){ // por si se requiere parar 
          moveRobot_byPose(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,p2);

      }else if(data == "fine"){
        actualizarPoseRecibida(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs);        


      }else if(data == "e"){ //E N D    W H I L E
        abajo = false;
        torque = 0;
        setTorqueAll(portHandler,packetHandler,IDs, torque);
      }
    }
  }



// T E S T      S E     P A R A     B O C A    A R R I B A 
  while(arriba){
    if(Serial.available() > 0){
      String data = Serial.readStringUntil('\n');

      if (data == "o"){//apaga torques momentaneamente
        torque = 0;
        setTorqueAll(portHandler,packetHandler,IDs, torque);
        delay(5000);
        torque = 1;
        setTorqueAll(portHandler,packetHandler,IDs, torque);

      }else if(data == "f"){ // se para boca arriba
        Stand_Up_FaceUp(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs);

      }else if(data == "0"){ //pose inicial de la secuencia
        pos_arriba = 0;
        moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,poses_boca_arriba[pos_arriba]);
        Serial.print("pose: ");
        Serial.println(pos_arriba);

      }else if(data == "g"){ // obtiene la posicion actual
        Serial.println(" ");
        getPositionAll(portHandler,packetHandler, IDs, PosPosPos);
        Serial.println(" ");
        Serial.println(" ");

      }else if(data == "p2"){
          moveRobot_byPose(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,p2);

      }else if(data == "fine"){
        actualizarPoseRecibida(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs);        

      }else if(data == "e"){ //E N D    W H I L E
        arriba = false;
        torque = 0;
        setTorqueAll(portHandler,packetHandler,IDs, torque);
      }
    }
  }



// T E S T      S E     P A T E A R    C O N      D E R E C H A
  while(kick_d){
    if(Serial.available() > 0){
      String data = Serial.readStringUntil('\n');

      if (data == "o"){//apaga torques momentaneamente
        torque = 0;
        setTorqueAll(portHandler,packetHandler,IDs, torque);
        delay(5000);
        torque = 1;
        setTorqueAll(portHandler,packetHandler,IDs, torque);

      }else if(data == "f"){ // se para boca arriba
        kick_der(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs);

      }else if(data == "0"){ //pose inicial de la secuencia
        pos_kick_d = 0;
        moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_patea_1);

      }else if(data == "g"){ // obtiene la posicion actual
        Serial.println(" ");
        getPositionAll(portHandler,packetHandler, IDs, PosPosPos);
        Serial.println(" ");
        Serial.println(" ");

      }else if(data == "p2"){
          moveRobot_byPose(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,p2);

      }else if(data == "fine"){
        actualizarPoseRecibida(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs);        

      }else if(data == "e"){ //E N D    W H I L E
        kick_d = false;
        torque = 0;
        setTorqueAll(portHandler,packetHandler,IDs, torque);
      }
    }
  }

// C I C L O     P A R A   I M P L E M E N T A R     O N E     T I M E    A L G O R I T H M S    C O N    I M U 
  while(complete_test){

    if((pitch < 15)){
      Stand_Up_FaceDown(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs);


    }else if((pitch > 165)){
      Stand_Up_FaceUp(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs);
    }

    if(Serial.available() > 0){
      String str = Serial.readStringUntil('\n');
      if(str == "e"){ //E N D    W H I L E
        complete_test = false;
        torque = 0;
        setTorqueAll(portHandler,packetHandler,IDs, torque);
      }
    }
  }


  torque = 0;
  setTorqueAll(portHandler,packetHandler,IDs, torque);
  stopCom(portHandler,packetHandler);
}




void handler_IMU(void) { // interrupcion de hardware para actualizar las lecturas de la IMU
  IMU.update();
  pitch = abs(IMU.rpy[0]);
  roll = abs(IMU.rpy[1]);
  yaw = abs(IMU.rpy[2]);  
}


void actualizarPoseRecibida(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
                   dynamixel::GroupSyncWrite groupSyncWritePos, dynamixel::GroupSyncWrite groupSyncWriteVel,
                   double IDs[18]) {
  bool finetuning = true;
  Serial.println("finetuneando ando");

  while(finetuning){

    if(Serial.available() > 0){
      String opt = Serial.readStringUntil('\n');

      if(opt == "e"){
        Serial.println("acabando loop de finetuneo");
        finetuning = false;
      
      }else if(opt == "g"){
        //get position
        Serial.println(" ");
        getPositionAll(portHandler,packetHandler, IDs, PosPosPos);
        Serial.println(" ");

      }else{
        String datos = opt;
        int indice = 0;
        int inicio = 0;

        while (indice < 18) {
          int separador = datos.indexOf(',', inicio);
          String valor_str = (separador == -1) ? datos.substring(inicio) : datos.substring(inicio, separador);

          // Convertir String a char[] para usar atof
          char buffer[20];
          valor_str.toCharArray(buffer, sizeof(buffer));
          pose_recibida[indice] = atof(buffer);

          indice++;
          if (separador == -1) break;
          inicio = separador + 1;
        }

        if (indice == 18) {
          Serial.println("Valores actualizados.");
          //get position
          Serial.println(" ");
          getPositionAll(portHandler,packetHandler, IDs, PosPosPos);
          Serial.println(" ");

          moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_recibida);

        } else {
          Serial.println("Error: faltan valores.");
        }
      }
    }
  } 
}


void loop(){}

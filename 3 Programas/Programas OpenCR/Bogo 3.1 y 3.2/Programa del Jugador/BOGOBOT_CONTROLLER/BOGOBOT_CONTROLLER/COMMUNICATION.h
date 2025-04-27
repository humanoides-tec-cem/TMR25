#ifndef COMMUNICATION
#define COMMUNICATION 

float state = 0.0; // state
float Wz = 0.0; // angular.z
float Vx = 0.0; // linear.x
float Vy = 0.0; // linear.y

int cmd_vel[8] = {0,0,0,0,0,0,0,0};
int led_pin = 22;
// ------------------------------------------------------------
// lee del 1 byte del serial, transforma a DEC y guarda en info
int byteToDec(byte b){
  int b0 = bitRead(b,0) * 1;
  int b1 = bitRead(b,1) * 2;
  int b2 = bitRead(b,2) * 4;
  int b3 = bitRead(b,3) * 8;
  int b4 = bitRead(b,4) * 16;
  int b5 = bitRead(b,5) * 32;
  int b6 = bitRead(b,6) * 64;
  int b7 = bitRead(b,7) * 128;

  return b0 + b1 + b2 + b3 + b4 + b5 + b6 + b7;
}

int readFromSerial(){
  for(int i = 0; i < 8 && Serial.available(); i++){
    cmd_vel[i] = byteToDec(Serial.read());
  }
  if (cmd_vel[7] == 10){

    digitalWrite(led_pin, LOW); // ON 
    state = cmd_vel[0];
    if(cmd_vel[1] == 1){Wz = cmd_vel[2]*-0.01;} else {Wz = cmd_vel[2]*0.01;}
    if(cmd_vel[3] == 1){Vx = cmd_vel[4]*-0.1;} else {Vx = cmd_vel[4]*0.1;}
    if(cmd_vel[5] == 1){Vy = cmd_vel[6]*-0.1;} else {Vy = cmd_vel[6]*0.1;}
  }
}

#endif
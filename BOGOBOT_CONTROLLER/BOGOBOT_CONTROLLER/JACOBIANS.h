#ifndef JACOBIANS
#define JACOBIANS

#include "B3_POS.h"
#include <BasicLinearAlgebra.h>
using namespace BLA;

double L3 = L_DIM[2]; double L4 = L_DIM[3];
double D3 = A_DIM[1]; double D4 = A_DIM[2];

  
// renglon 1
BLA::Matrix<6,6,double> rightLeg_Jacobian(double qJ[6]){

  double q7 = qJ[0]; double q9 = qJ[1]; 
  double q11 = qJ[2]; double q13 = qJ[3]; 
  double q15 = qJ[4]; double q17 = qJ[5]; 

  double c7 = cos(q7); double s7 = sin(q7);
  double c9 = cos(q9); double s9 = sin(q9);
  double c11 = cos(q11); double s11 = sin(q11);
  double c13 = cos(q13); double s13 = sin(q13);
  double c15 = cos(q15); double s15 = sin(q15);
  double c17 = cos(q17); double s17 = sin(q17);

  double c11_13 = cos(q11 + q13); double s11_13 = sin(q11 + q13);
  double c11_13_15 = cos(q11 + q13 - q15); double s11_13_15 = sin(q11 + q13 - q15);

  BLA::Matrix<6,6,double> Jt_RLeg;
  
  // renglon 1
  Jt_RLeg(0,0) = -s7*(L4*s11_13 + L3*s11) - c7*s9*(L4*c11_13 + L3*c11);
  Jt_RLeg(0,1) = -c9*s7*(L4*c11_13 + L3*c11);
  Jt_RLeg(0,2) = c7*(L4*c11_13 + L3*c11) + s7*s9*(L4*s11_13 + L3*s11);
  Jt_RLeg(0,3) = L4*c11_13*c7 + L4*s11_13*s7*s9;
  Jt_RLeg(0,4) = 0;
  Jt_RLeg(0,5) = 0;

  // renglon 2
  Jt_RLeg(1,0) = s7*s9*(L4*c11_13 + L3*c11) - c7*(L4*s11_13 + L3*s11);
  Jt_RLeg(1,1) = -c7*c9*(L4*c11_13 + L3*c11);
  Jt_RLeg(1,2) = c7*s9*(L4*s11_13 + L3*s11) - s7*(L4*c11_13 + L3*c11);
  Jt_RLeg(1,3) = L4*s11_13*c7*s9 - L4*c11_13*s7;
  Jt_RLeg(1,4) = 0;
  Jt_RLeg(1,5) = 0;

  // renglon 3
  Jt_RLeg(2,0) = 0;
  Jt_RLeg(2,1) = s9*(L4*c11_13 + L3*c11);
  Jt_RLeg(2,2) = c9*(L4*s11_13 + L3*s11);
  Jt_RLeg(2,3) = L4*s11_13*c9;
  Jt_RLeg(2,4) = 0;
  Jt_RLeg(2,5) = 0;

  // renglon 4
  Jt_RLeg(3,0) = 0;
  Jt_RLeg(3,1) = (c9*s17 - c11_13_15*c17*s9)*(c17*(s11_13_15*s7 + c11_13_15*c7*s9) - c7*c9*s17) - (c9*c17 + c11_13_15*s9*s17)*(s17*(s11_13_15*s7 + c11_13_15*c7*s9) + c7*c9*c17) + s11_13_15*s9*(c11_13_15*s7 - s11_13_15*c7*s9);
  Jt_RLeg(3,2) = -c9*s7;
  Jt_RLeg(3,3) = -c9*s7;
  Jt_RLeg(3,4) = c9*s7;
  Jt_RLeg(3,5) = c11_13_15*c7 + s11_13_15*s7*s9;

  // renglon 5
  Jt_RLeg(4,0) = (s9*s17 + c11_13_15*c9*c17)*(c17*(s11_13_15*s7 + c11_13_15*c7*s9) - c7*c9*s17) - (c17*s9 - c11_13_15*c9*s17)*(s17*(s11_13_15*s7 + c11_13_15*c7*s9) + c7*c9*c17) - s11_13_15*c9*(c11_13_15*s7 - s11_13_15*c7*s9);
  Jt_RLeg(4,1) = s7;
  Jt_RLeg(4,2) = s17*(c17*s9 - c11_13_15*c9*s17)*(c11_13_15*c7 + s11_13_15*s7*s9) - c17*(s9*s17 + c11_13_15*c9*c17)*(c11_13_15*c7 + s11_13_15*s7*s9) - s11_13_15*c9*(s11_13_15*c7 - c11_13_15*s7*s9);
  Jt_RLeg(4,3) = s17*(c17*s9 - c11_13_15*c9*s17)*(c11_13_15*c7 + s11_13_15*s7*s9) - c17*(s9*s17 + c11_13_15*c9*c17)*(c11_13_15*c7 + s11_13_15*s7*s9) - s11_13_15*c9*(s11_13_15*c7 - c11_13_15*s7*s9);
  Jt_RLeg(4,4) = c7*c9;
  Jt_RLeg(4,5) = s11_13_15*c7*s9 - c11_13_15*s7;

  // renglon 6
  Jt_RLeg(5,0) = - pow((c11_13_15*c7 + s11_13_15*s7*s9),2) - pow((c17*(s11_13_15*c7 - c11_13_15*s7*s9) + c9*s7*s17),2) - pow((s17*(s11_13_15*c7 - c11_13_15*s7*s9) - c9*c17*s7),2);
  Jt_RLeg(5,1) = (s17*(s11_13_15*c7 - c11_13_15*s7*s9) - c9*c17*s7)*(c7*c17*s9 - c11_13_15*c7*c9*s17) - (c17*(s11_13_15*c7 - c11_13_15*s7*s9) + c9*s7*s17)*(c7*s9*s17 + c11_13_15*c7*c9*c17) + s11_13_15*c7*c9*(c11_13_15*c7 + s11_13_15*s7*s9);
  Jt_RLeg(5,2) = (c11_13_15*c7 + s11_13_15*s7*s9)*(s11_13_15*s7 + c11_13_15*c7*s9) - c17*(c17*(s11_13_15*c7 - c11_13_15*s7*s9) + c9*s7*s17)*(c11_13_15*s7 - s11_13_15*c7*s9) - s17*(s17*(s11_13_15*c7 - c11_13_15*s7*s9) - c9*c17*s7)*(c11_13_15*s7 - s11_13_15*c7*s9);
  Jt_RLeg(5,3) = (c11_13_15*c7 + s11_13_15*s7*s9)*(s11_13_15*s7 + c11_13_15*c7*s9) - c17*(c17*(s11_13_15*c7 - c11_13_15*s7*s9) + c9*s7*s17)*(c11_13_15*s7 - s11_13_15*c7*s9) - s17*(s17*(s11_13_15*c7 - c11_13_15*s7*s9) - c9*c17*s7)*(c11_13_15*s7 - s11_13_15*c7*s9);
  Jt_RLeg(5,4) = c17*(c17*(s11_13_15*c7 - c11_13_15*s7*s9) + c9*s7*s17)*(c11_13_15*s7 - s11_13_15*c7*s9) - (c11_13_15*c7 + s11_13_15*s7*s9)*(s11_13_15*s7 + c11_13_15*c7*s9) + s17*(s17*(s11_13_15*c7 - c11_13_15*s7*s9) - c9*c17*s7)*(c11_13_15*s7 - s11_13_15*c7*s9);
  Jt_RLeg(5,5) = s11_13_15*c9;

  return Jt_RLeg;
}

BLA::Matrix<6,6,double> leftLeg_Jacobian(double qJ[6]){
  double q8 = qJ[0]; double q10 = qJ[1];
  double q12 = qJ[2]; double q14 = qJ[3];
  double q16 = qJ[4]; double q18 = qJ[5];

  double c8 = cos(q8); double s8 = sin(q8);
  double c10 = cos(q10); double s10 = sin(q10);
  double c12 = cos(q12); double s12 = sin(q12);
  double c14 = cos(q14); double s14 = sin(q14);
  double c16 = cos(q16); double s16 = sin(q16);
  double c18 = cos(q18); double s18 = sin(q18);

  double c12_14 = cos(q12 + q14); double s12_14 = sin(q12 + q14);
  double c12_14_16 = cos(q12 + q14 - q16); double s12_14_16 = sin(q12 + q14 - q16);
  
  BLA::Matrix<6,6,double> Jt_LLeg;

  // renglon 1
  Jt_LLeg(0,0) = s8*(L4*s12_14 + L3*s12) - c8*s10*(L4*c12_14 + L3*c12);
  Jt_LLeg(0,1) = -c10*s8*(L4*c12_14 + L3*c12);
  Jt_LLeg(0,2) = s8*s10*(L4*s12_14 + L3*s12) - c8*(L4*c12_14 + L3*c12);
  Jt_LLeg(0,3) = L4*s12_14*s8*s10 - L4*c12_14*c8;
  Jt_LLeg(0,4) = 0;
  Jt_LLeg(0,5) = 0;

  // renglon 2
  Jt_LLeg(1,0) = c8*(L4*s12_14 + L3*s12) + s8*s10*(L4*c12_14 + L3*c12);
  Jt_LLeg(1,1) = -c8*c10*(L4*c12_14 + L3*c12);
  Jt_LLeg(1,2) = s8*(L4*c12_14 + L3*c12) + c8*s10*(L4*s12_14 + L3*s12);
  Jt_LLeg(1,3) = L4*c12_14*s8 + L4*s12_14*c8*s10;
  Jt_LLeg(1,4) = 0;
  Jt_LLeg(1,5) = 0;

  // renglon 3
  Jt_LLeg(2,0) = 0;
  Jt_LLeg(2,1) = s10*(L4*c12_14 + L3*c12);
  Jt_LLeg(2,2) = c10*(L4*s12_14 + L3*s12);
  Jt_LLeg(2,3) = L4*s12_14*c10;
  Jt_LLeg(2,4) = 0;
  Jt_LLeg(2,5) = 0;

  // renglon 4
  Jt_LLeg(3,0) = 0;
  Jt_LLeg(3,1) = (c10*c18 + c12_14_16*s10*s18)*(s18*(s12_14_16*s8 - c12_14_16*c8*s10) - c8*c10*c18) - (c10*s18 - c12_14_16*c18*s10)*(c18*(s12_14_16*s8 - c12_14_16*c8*s10) + c8*c10*s18) - s12_14_16*s10*(c12_14_16*s8 + s12_14_16*c8*s10);
  Jt_LLeg(3,2) = c10*s8;
  Jt_LLeg(3,3) = c10*s8;
  Jt_LLeg(3,4) = -c10*s8;
  Jt_LLeg(3,5) = c12_14_16*c8 - s12_14_16*s8*s10;

  // renglon 5
  Jt_LLeg(4,0) = (c18*s10 - c12_14_16*c10*s18)*(s18*(s12_14_16*s8 - c12_14_16*c8*s10) - c8*c10*c18) - (s10*s18 + c12_14_16*c10*c18)*(c18*(s12_14_16*s8 - c12_14_16*c8*s10) + c8*c10*s18) + s12_14_16*c10*(c12_14_16*s8 + s12_14_16*c8*s10);
  Jt_LLeg(4,1) = s8;
  Jt_LLeg(4,2) = c8*c10;
  Jt_LLeg(4,3) = c8*c10;
  Jt_LLeg(4,4) = -c8*c10;
  Jt_LLeg(4,5) = -c12_14_16*s8 - s12_14_16*c8*s10;

  // renglon 6
  Jt_LLeg(5,0) = -pow((c12_14_16*c8 - s12_14_16*s8*s10),2) - pow((s18*(s12_14_16*c8 + c12_14_16*s8*s10) + c10*c18*s8),2) - pow((c18*(s12_14_16*c8 + c12_14_16*s8*s10) - c10*s8*s18),2);
  Jt_LLeg(5,1) = (c18*(s12_14_16*c8 + c12_14_16*s8*s10) - c10*s8*s18)*(c8*s10*s18 + c12_14_16*c8*c10*c18) - (s18*(s12_14_16*c8 + c12_14_16*s8*s10) + c10*c18*s8)*(c8*c18*s10 - c12_14_16*c8*c10*s18) - s12_14_16*c8*c10*(c12_14_16*c8 - s12_14_16*s8*s10);
  Jt_LLeg(5,2) = (c12_14_16*c8 - s12_14_16*s8*s10)*(s12_14_16*s8 - c12_14_16*c8*s10) - c18*(c18*(s12_14_16*c8 + c12_14_16*s8*s10) - c10*s8*s18)*(c12_14_16*s8 + s12_14_16*c8*s10) - s18*(s18*(s12_14_16*c8 + c12_14_16*s8*s10) + c10*c18*s8)*(c12_14_16*s8 + s12_14_16*c8*s10);
  Jt_LLeg(5,3) = (c12_14_16*c8 - s12_14_16*s8*s10)*(s12_14_16*s8 - c12_14_16*c8*s10) - c18*(c18*(s12_14_16*c8 + c12_14_16*s8*s10) - c10*s8*s18)*(c12_14_16*s8 + s12_14_16*c8*s10) - s18*(s18*(s12_14_16*c8 + c12_14_16*s8*s10) + c10*c18*s8)*(c12_14_16*s8 + s12_14_16*c8*s10);
  Jt_LLeg(5,4) = c18*(c18*(s12_14_16*c8 + c12_14_16*s8*s10) - c10*s8*s18)*(c12_14_16*s8 + s12_14_16*c8*s10) - (c12_14_16*c8 - s12_14_16*s8*s10)*(s12_14_16*s8 - c12_14_16*c8*s10) + s18*(s18*(s12_14_16*c8 + c12_14_16*s8*s10) + c10*c18*s8)*(c12_14_16*s8 + s12_14_16*c8*s10);
  Jt_LLeg(5,5) = -s12_14_16*c10;

  return Jt_LLeg;
}

BLA::Matrix<3,3,double> rightArm_Jacobian(double q[18]){
  double q1 = q[0]; double q2 = q[1]; double q3 = q[2]; 
  double q4 = q[3]; double q5 = q[4]; double q6 = q[5];
  double q7 = q[6]; double q8 = q[7]; double q9 = q[8]; 
  double q10 = q[9]; double q11 = q[10]; double q12 = q[11];
  double q13 = q[12]; double q14 = q[13]; double q15 = q[14]; 
  double q16 = q[15]; double q17 = q[16]; double q18 = q[17];

  double c1 = cos(q1); double s1 = sin(q1);
  double c3 = cos(q3); double s3 = sin(q3);
  double c5 = cos(q5); double s5 = sin(q5);

  double c3_5 = cos(q3 - q5); double s3_5 = sin(q3 - q5);

  BLA::Matrix<3,3,double> Jt_RArm;

  // renglon 1
  Jt_RArm(0,0) = c1*(D3*c3 + D4*c3_5);
  Jt_RArm(0,1) = -s1*(D3*s3 + D4*s3_5 );
  Jt_RArm(0,2) = D4*s3_5 *s1;

  // renglon 2
  Jt_RArm(1,0) = 0;
  Jt_RArm(1,1) = -D3*c3 - D4*c3_5 ;
  Jt_RArm(1,2) = D4*c3_5 ;

  // renglon 3
  Jt_RArm(2,0) = s1*(D3*c3 + D4*c3_5);
  Jt_RArm(2,1) = c1*(D3*s3 + D4*s3_5 );
  Jt_RArm(2,2) = -D4*s3_5 *c1;

  return Jt_RArm;
}

BLA::Matrix<3,3,double> leftArm_Jacobian(double q[18]){
  double q1 = q[0]; double q2 = q[1]; double q3 = q[2]; 
  double q4 = q[3]; double q5 = q[4]; double q6 = q[5];
  double q7 = q[6]; double q8 = q[7]; double q9 = q[8]; 
  double q10 = q[9]; double q11 = q[10]; double q12 = q[11];
  double q13 = q[12]; double q14 = q[13]; double q15 = q[14]; 
  double q16 = q[15]; double q17 = q[16]; double q18 = q[17];

  double c2 = cos(q2); double s2 = sin(q2);
  double c4 = cos(q4); double s4 = sin(q4);
  double c6 = cos(q6); double s6 = sin(q6);

  double c4_6 = cos(q4 - q6); double s4_6 = sin(q4 - q6);

  BLA::Matrix<3,3,double> Jt_LArm;

  // renglon 1
  Jt_LArm(0,0) = -c2*(D3*c4 + D4*c4_6);
  Jt_LArm(0,1) = s2*(D3*s4 + D4*s4_6);
  Jt_LArm(0,2) = -D4*s4_6*s2;

  // renglon 2
  Jt_LArm(1,0) = 0;
  Jt_LArm(1,1) = -D3*c4 -D4*c4_6;
  Jt_LArm(1,2) = D4*c4_6;

  // renglon 3
  Jt_LArm(2,0) = s2*(D3*c4 + D4*c4_6);
  Jt_LArm(2,1) = c2*(D3*s4 + D4*s4_6);
  Jt_LArm(2,2) = -D4*s4_6*c2;

  return Jt_LArm;
}

void printJacobian3x3(BLA::Matrix<3,3,double> J){
  for(int i= 0; i < 3; i = i+1){
    for(int j= 0; j < 3; j = j+1){
      Serial.print(J(i,j),4);
      Serial.print(" ");
    }
    Serial.println("");
  }
  Serial.println("");
}

void printJacobian6x6(BLA::Matrix<6,6,double> J){
  for(int i= 0; i < 6; i = i+1){
    for(int j= 0; j < 6; j = j+1){
      Serial.print(J(i,j),4);
      Serial.print("  ");
    }
    Serial.println("");
  }
  Serial.println("");
}

#endif
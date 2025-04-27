#ifndef FK
#define FK

#include "B3_POS.h"
#include "JACOBIANS.h"
#include <BasicLinearAlgebra.h>

BLA::Matrix<6,1,double> getRightLeg_POSE(double qLeg[6]){
  double L3 = L_DIM[2]; double L4 = L_DIM[3];
  double q7 = qLeg[0]; double q9 = qLeg[1]; double q11 = qLeg[2];
  double q13 = qLeg[3]; double q15 = qLeg[4];double q17 = qLeg[5];

  double c7 = cos(q7); double s7 = sin(q7);
  double c9 = cos(q9); double s9 = sin(q9);
  double c11 = cos(q11); double s11 = sin(q11);
  double c13 = cos(q13); double s13 = sin(q13);
  double c15 = cos(q15); double s15 = sin(q15);
  double c17 = cos(q17); double s17 = sin(q17);
  
  // Define the position and orientation vector
  BLA::Matrix<6,1,double> legPose;

  // Calculate de position vector using FK [x;y;z]
  legPose(0,0) = L3*(c7*s11 - c11*s7*s9) + L4*(c13*(c7*s11 - c11*s7*s9) + s13*(c7*c11 + s7*s9*s11)); // x
  legPose(1,0) = - L3*(s7*s11 + c7*c11*s9) - L4*(c13*(s7*s11 + c7*c11*s9) + s13*(c11*s7 - c7*s9*s11)); // y
  legPose(2,0) = -c9*(L4*cos(q11 + q13) + L3*c11); // z

  // Calculate the orientation vector using Kajita's formula

  // rotation matrix
  double r11 = s15*(c13*(c7*s11 - c11*s7*s9) + s13*(c7*c11 + s7*s9*s11)) + c15*(c13*(c7*c11 + s7*s9*s11) - s13*(c7*s11 - c11*s7*s9));
  double r12 = s17*(s15*(c13*(c7*c11 + s7*s9*s11) - s13*(c7*s11 - c11*s7*s9)) - c15*(c13*(c7*s11 - c11*s7*s9) + s13*(c7*c11 + s7*s9*s11))) + c9*c17*s7;
  double r13 = c17*(s15*(c13*(c7*c11 + s7*s9*s11) - s13*(c7*s11 - c11*s7*s9)) - c15*(c13*(c7*s11 - c11*s7*s9) + s13*(c7*c11 + s7*s9*s11))) - c9*s7*s17;

  double r21 = - s15*(c13*(s7*s11 + c7*c11*s9) + s13*(c11*s7 - c7*s9*s11)) - c15*(c13*(c11*s7 - c7*s9*s11) - s13*(s7*s11 + c7*c11*s9));
  double r22 = c7*c9*c17 - s17*(s15*(c13*(c11*s7 - c7*s9*s11) - s13*(s7*s11 + c7*c11*s9)) - c15*(c13*(s7*s11 + c7*c11*s9) + s13*(c11*s7 - c7*s9*s11)));
  double r23 = - c17*(s15*(c13*(c11*s7 - c7*s9*s11) - s13*(s7*s11 + c7*c11*s9)) - c15*(c13*(s7*s11 + c7*c11*s9) + s13*(c11*s7 - c7*s9*s11))) - c7*c9*s17;

  double r31 = sin(q11 + q13 - q15)*c9;
  double r32 = cos(q11 + q13 - q15)*c9*s17 - c17*s9;
  double r33 = s9*s17 + cos(q11 + q13 - q15)*c9*c17;

  // Determine if the matrix is diagonal
  int ctrDiag = 0; // when this counter stays at 0 the matrix is diagonal
  if (r12 != 0){ctrDiag = ctrDiag+1;}
  if (r13 != 0){ctrDiag = ctrDiag+1;}
  if (r21 != 0){ctrDiag = ctrDiag+1;}
  if (r23 != 0){ctrDiag = ctrDiag+1;}
  if (r31 != 0){ctrDiag = ctrDiag+1;}
  if (r32 != 0){ctrDiag = ctrDiag+1;}

  double k1 = 0; double k2 = 0; double k3 = 0;
  if (ctrDiag == 0){
    // the matrix is diagonal
    k1 = (2*PI/2)*(r11 +1);
    k2 = (2*PI/2)*(r22 +1);
    k3 = (2*PI/2)*(r33 +1);
  }
  else{
    // the matrix is not diagonal
    double l1 = r32 - r23;
    double l2 = r13 - r31;
    double l3 = r21 - r12;

    double mag_l = sqrt(pow(l1,2) + pow(l2,2) + pow(l3,2));
    double theta = atan2_(mag_l, r11 + r22 + r33 -1);

    k1 = theta * (l1/mag_l);
    k2 = theta * (l2/mag_l);
    k3 = theta * (l3/mag_l);
  }

  k1 = atan2_(sin(k1), cos(k1));
  k2 = atan2_(sin(k2), cos(k2));
  k3 = atan2_(sin(k3), cos(k3));

  legPose(3,0) = k1;
  legPose(4,0) = k2;
  legPose(5,0) = k3;

  Serial.println("RIGHT LEG POSE");
  for (int i = 0; i<6; i = i+1){
    Serial.println(legPose(i,0));
  }
  return legPose;
}

BLA::Matrix<6,1,double> getLeftLeg_POSE(double qLeg[6]){
  double L3 = L_DIM[2]; double L4 = L_DIM[3];

  double q8 = qLeg[0]; double q10 = qLeg[1]; double q12 = qLeg[2];
  double q14 = qLeg[3]; double q16 = qLeg[4];double q18 = qLeg[5];

  double c8 = cos(q8); double s8 = sin(q8);
  double c10 = cos(q10); double s10 = sin(q10);
  double c12 = cos(q12); double s12 = sin(q12);
  double c14 = cos(q14); double s14 = sin(q14);
  double c16 = cos(q16); double s16 = sin(q16);
  double c18 = cos(q18); double s18 = sin(q18);
  
  // position and orientation vector
  BLA::Matrix<6,1,double> legPose;

  // Calculate de position vector using FK [x;y;z]
  legPose(0,0) = - L3*(c8*s12 + c12*s8*s10) - L4*(c14*(c8*s12 + c12*s8*s10) + s14*(c8*c12 - s8*s10*s12));
  legPose(1,0) = L3*(s8*s12 - c8*c12*s10) + L4*(c14*(s8*s12 - c8*c12*s10) + s14*(c12*s8 + c8*s10*s12));
  legPose(2,0) = -c10*(L4*cos(q12 + q14) + L3*c12);

  // Calculate the orientation vector using Kajita's formula

  // rotation matrix
  double r11 = sin(q16)*(cos(q14)*(cos(q8)*sin(q12) + cos(q12)*sin(q8)*sin(q10)) + sin(q14)*(cos(q8)*cos(q12) - sin(q8)*sin(q10)*sin(q12))) + cos(q16)*(cos(q14)*(cos(q8)*cos(q12) - sin(q8)*sin(q10)*sin(q12)) - sin(q14)*(cos(q8)*sin(q12) + cos(q12)*sin(q8)*sin(q10)));
  double r12 = cos(q10)*cos(q18)*sin(q8) - sin(q18)*(sin(q16)*(cos(q14)*(cos(q8)*cos(q12) - sin(q8)*sin(q10)*sin(q12)) - sin(q14)*(cos(q8)*sin(q12) + cos(q12)*sin(q8)*sin(q10))) - cos(q16)*(cos(q14)*(cos(q8)*sin(q12) + cos(q12)*sin(q8)*sin(q10)) + sin(q14)*(cos(q8)*cos(q12) - sin(q8)*sin(q10)*sin(q12))));
  double r13 = - cos(q18)*(sin(q16)*(cos(q14)*(cos(q8)*cos(q12) - sin(q8)*sin(q10)*sin(q12)) - sin(q14)*(cos(q8)*sin(q12) + cos(q12)*sin(q8)*sin(q10))) - cos(q16)*(cos(q14)*(cos(q8)*sin(q12) + cos(q12)*sin(q8)*sin(q10)) + sin(q14)*(cos(q8)*cos(q12) - sin(q8)*sin(q10)*sin(q12)))) - cos(q10)*sin(q8)*sin(q18);

  double r21 = - sin(q16)*(cos(q14)*(sin(q8)*sin(q12) - cos(q8)*cos(q12)*sin(q10)) + sin(q14)*(cos(q12)*sin(q8) + cos(q8)*sin(q10)*sin(q12))) - cos(q16)*(cos(q14)*(cos(q12)*sin(q8) + cos(q8)*sin(q10)*sin(q12)) - sin(q14)*(sin(q8)*sin(q12) - cos(q8)*cos(q12)*sin(q10)));
  double r22 = sin(q18)*(sin(q16)*(cos(q14)*(cos(q12)*sin(q8) + cos(q8)*sin(q10)*sin(q12)) - sin(q14)*(sin(q8)*sin(q12) - cos(q8)*cos(q12)*sin(q10))) - cos(q16)*(cos(q14)*(sin(q8)*sin(q12) - cos(q8)*cos(q12)*sin(q10)) + sin(q14)*(cos(q12)*sin(q8) + cos(q8)*sin(q10)*sin(q12)))) + cos(q8)*cos(q10)*cos(q18);
  double r23 = cos(q18)*(sin(q16)*(cos(q14)*(cos(q12)*sin(q8) + cos(q8)*sin(q10)*sin(q12)) - sin(q14)*(sin(q8)*sin(q12) - cos(q8)*cos(q12)*sin(q10))) - cos(q16)*(cos(q14)*(sin(q8)*sin(q12) - cos(q8)*cos(q12)*sin(q10)) + sin(q14)*(cos(q12)*sin(q8) + cos(q8)*sin(q10)*sin(q12)))) - cos(q8)*cos(q10)*sin(q18);

  double r31 = -sin(q12 + q14 - q16)*cos(q10);
  double r32 = cos(q12 + q14 - q16)*cos(q10)*sin(q18) - cos(q18)*sin(q10);
  double r33 = sin(q10)*sin(q18) + cos(q12 + q14 - q16)*cos(q10)*cos(q18);

  // Determine if the matrix is diagonal
  int ctrDiag = 0; // when this counter stays at 0 the matrix is diagonal
  if (r12 != 0){ctrDiag = ctrDiag+1;}
  if (r13 != 0){ctrDiag = ctrDiag+1;}
  if (r21 != 0){ctrDiag = ctrDiag+1;}
  if (r23 != 0){ctrDiag = ctrDiag+1;}
  if (r31 != 0){ctrDiag = ctrDiag+1;}
  if (r32 != 0){ctrDiag = ctrDiag+1;}

  double k1 = 0; double k2 = 0; double k3 = 0;
  if (ctrDiag == 0){
    // the matrix is diagonal
    k1 = (2*PI/2)*(r11 +1);
    k2 = (2*PI/2)*(r22 +1);
    k3 = (2*PI/2)*(r33 +1);
  }
  else{
    // the matrix is not diagonal
    double l1 = r32 - r23;
    double l2 = r13 - r31;
    double l3 = r21 - r12;

    double mag_l = sqrt(pow(l1,2) + pow(l2,2) + pow(l3,2));
    double theta = atan2_(mag_l, r11 + r22 + r33 -1);

    k1 = theta * (l1/mag_l);
    k2 = theta * (l2/mag_l);
    k3 = theta * (l3/mag_l);
  }

  k1 = atan2_(sin(k1), cos(k1));
  k2 = atan2_(sin(k2), cos(k2));
  k3 = atan2_(sin(k3), cos(k3));

  legPose(3,0) = k1;
  legPose(4,0) = k2;
  legPose(5,0) = k3;

  return legPose;
}
#endif
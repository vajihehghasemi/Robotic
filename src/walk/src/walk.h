#ifndef WALK_H
#define WALK_H

#include <math.h>
#include "calc.h"

class walk
{
public:
  calc C;
  double t;
  double P_pelvis[3];
  double P_rightFoot[3];
  double P_leftFoot[3];
  double Yaw_pelvis;
  double Yaw_rightFoot;
  double Yaw_leftFoot;

  double l_pelvis=.14;
  double l_thigh=.27;
  double l_leg=.2;
  double y_pelvisMax=.08;
  double stepLength=.08;
  double stepHeight=.03;
  double l_R;
  double l_L;
  double q_H_Y_R=0;
  double q_H_R_R=0;
  double q_H_P_R=0;
  double q_K_P_R=0;
  double q_A_P_R=0;
  double q_A_R_R=0;
  double q_H_Y_L=0;
  double q_H_R_L=0;
  double q_H_P_L=0;
  double q_K_P_L=0;
  double q_A_P_L=0;
  double q_A_R_L=0;

  double T_pelvis=2;
  double T_stepping=2;

  walk();
  void kinematic();
  void inverseKinematic();
  void lower_height(double h,double T_start,double T_end);
  void pelvis2side(bool left, double T_start, double T_end);
  void firstStep(bool left, double T_start, double T_end);
  void stepping(bool left, double T_start, double T_end);
};

#endif // WALK_H

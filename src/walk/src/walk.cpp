#include "walk.h"

walk::walk()
{
  P_leftFoot[0]=0;
  P_leftFoot[1]= l_pelvis/2;
  P_leftFoot[2]=-l_thigh-l_leg;
  P_rightFoot[0]=0;
  P_rightFoot[1]=-l_pelvis/2;
  P_rightFoot[2]=-l_thigh-l_leg;
  P_pelvis[0]=0;
  P_pelvis[1]=0;
  P_pelvis[2]=0;
}

void walk::kinematic(){
  l_L=sqrt(l_thigh*l_thigh+l_leg*l_leg+2*l_thigh*l_leg*cos(q_K_P_L));
  double q_P_corr_L=asin(sin(q_K_P_L)*l_leg/l_L);
  P_leftFoot[0]=P_pelvis[0]-l_L*(cos(q_H_Y_L)*sin(q_H_P_L+q_P_corr_L)+sin(q_H_Y_L)*cos(q_H_P_L+q_P_corr_L)*sin(q_H_R_L));
  P_leftFoot[1]=P_pelvis[1]+l_pelvis/2-l_L*(sin(q_H_Y_L)*sin(q_H_P_L+q_P_corr_L)-cos(q_H_Y_L)*cos(q_H_P_L+q_P_corr_L)*sin(q_H_R_L));
  P_leftFoot[2]=P_pelvis[2]-l_L*(cos(q_H_R_L)*cos(q_H_P_L+q_P_corr_L));

  l_R=sqrt(l_thigh*l_thigh+l_leg*l_leg+2*l_thigh*l_leg*cos(q_K_P_R));
  double q_P_corr_R=asin(sin(q_K_P_R)*l_leg/l_R);
  P_rightFoot[0]=P_pelvis[0]-l_R*(cos(q_H_Y_R)*sin(q_H_P_R+q_P_corr_R)+sin(q_H_Y_R)*cos(q_H_P_R+q_P_corr_R)*sin(q_H_R_R));
  P_rightFoot[1]=P_pelvis[1]-l_pelvis/2-l_R*(sin(q_H_Y_R)*sin(q_H_P_R+q_P_corr_R)-cos(q_H_Y_R)*cos(q_H_P_R+q_P_corr_R)*sin(q_H_R_R));
  P_rightFoot[2]=P_pelvis[2]-l_R*(cos(q_H_R_R)*cos(q_H_P_R+q_P_corr_R));
}

void walk::inverseKinematic(){
double P_leftFoot_aux[2];
double P_rightFoot_aux[2];

P_leftFoot_aux[0]=P_pelvis[0]+(P_leftFoot[0]-P_pelvis[0])*cos(Yaw_pelvis)+(P_leftFoot[1]-P_pelvis[1])*sin(Yaw_pelvis);
P_leftFoot_aux[1]=P_pelvis[1]+(P_leftFoot[1]-P_pelvis[1])*cos(Yaw_pelvis)-(P_leftFoot[0]-P_pelvis[0])*sin(Yaw_pelvis);

P_rightFoot_aux[0]=P_pelvis[0]+(P_rightFoot[0]-P_pelvis[0])*cos(Yaw_pelvis)+(P_rightFoot[1]-P_pelvis[1])*sin(Yaw_pelvis);
P_rightFoot_aux[1]=P_pelvis[1]+(P_rightFoot[1]-P_pelvis[1])*cos(Yaw_pelvis)-(P_rightFoot[0]-P_pelvis[0])*sin(Yaw_pelvis);

  q_H_Y_L=Yaw_leftFoot-Yaw_pelvis;
  //l_L=sqrt(pow(P_leftFoot[0]-(P_pelvis[0]+l_pelvis/2*sin(Yaw_pelvis)),2)+pow(P_leftFoot[1]-(P_pelvis[1]+l_pelvis/2*cos(Yaw_pelvis)),2)+pow(P_leftFoot[2]-P_pelvis[2],2));
 l_L=sqrt(pow(P_leftFoot_aux[0]-(P_pelvis[0]),2)+pow(P_leftFoot_aux[1]-(P_pelvis[1]+l_pelvis/2),2)+pow(P_leftFoot[2]-P_pelvis[2],2));
  q_K_P_L=acos((l_L*l_L-l_thigh*l_thigh-l_leg*l_leg)/(2*l_thigh*l_leg));
  if(fabs((l_L*l_L-l_thigh*l_thigh-l_leg*l_leg)/(2*l_thigh*l_leg))>1){
    q_K_P_L=0;
    qDebug()<<"left leg IK err";
  }
  double q_P_corr_L=asin(sin(q_K_P_L)*l_leg/l_L);
  double a=-(P_leftFoot_aux[0]-(P_pelvis[0]))/l_L;
  double b=-(P_leftFoot_aux[1]-(P_pelvis[1]+l_pelvis/2))/l_L;
  q_H_P_L=asin(a*cos(q_H_Y_L)+b*sin(q_H_Y_L))-q_P_corr_L;
  q_H_R_L=-asin((-a*sin(q_H_Y_L)+b*cos(q_H_Y_L))/cos(q_H_P_L+q_P_corr_L));
  q_A_P_L=-q_K_P_L-q_H_P_L;
  q_A_R_L=-q_H_R_L;

  q_H_Y_R=Yaw_rightFoot-Yaw_pelvis;
  //l_R=sqrt(pow(P_rightFoot[0]-(P_pelvis[0]-l_pelvis/2*sin(Yaw_pelvis)),2)+pow(P_rightFoot[1]-(P_pelvis[1]-l_pelvis/2*cos(Yaw_pelvis)),2)+pow(P_rightFoot[2]-P_pelvis[2],2));
   l_R=sqrt(pow(P_rightFoot_aux[0]-(P_pelvis[0]),2)+pow(P_rightFoot_aux[1]-(P_pelvis[1]-l_pelvis/2),2)+pow(P_rightFoot[2]-P_pelvis[2],2));
  q_K_P_R=acos((l_R*l_R-l_thigh*l_thigh-l_leg*l_leg)/(2*l_thigh*l_leg));
  if(fabs((l_R*l_R-l_thigh*l_thigh-l_leg*l_leg)/(2*l_thigh*l_leg))>1){
    q_K_P_R=0;
    qDebug()<<"right leg IK err";
  }
  double q_P_corr_R=asin(sin(q_K_P_R)*l_leg/l_R);
  a=-(P_rightFoot_aux[0]-(P_pelvis[0]))/l_R;
  b=-(P_rightFoot_aux[1]-(P_pelvis[1]-l_pelvis/2))/l_R;
  q_H_P_R=asin(a*cos(q_H_Y_R)+b*sin(q_H_Y_R))-q_P_corr_R;
  q_H_R_R=-asin((-a*sin(q_H_Y_R)+b*cos(q_H_Y_R))/cos(q_H_P_R+q_P_corr_R));
  q_A_P_R=-q_K_P_R-q_H_P_R;
  q_A_R_R=-q_H_R_R;

}

void walk::lower_height(double h,double T_start,double T_end){
  P_pelvis[2]=C.move2pose(h,t,T_start,T_end);
}

void walk::pelvis2side(bool left,double T_start,double T_end){
  if (left) {
    P_pelvis[1]=C.move2pose(y_pelvisMax,t,T_start,T_end);
  }
  else{
    P_pelvis[1]=C.move2pose(-y_pelvisMax,t,T_start,T_end);
  }
}

void walk::firstStep(bool left,double T_start,double T_end){
  double duration=T_end-T_start;
  double alpha=.5;
  double beta=.25;
  pelvis2side(!left,T_start,T_start+duration*alpha);
  if(left){
   P_leftFoot[0]= C.move2pose(stepLength/2,t,T_start+duration*alpha,T_end);
   P_leftFoot[2]=-l_thigh-l_leg+C.move2pose(stepHeight,t,T_start+duration*alpha,T_start+duration*(alpha+beta))-C.move2pose(stepHeight,t,T_start+duration*(alpha+beta),T_end);

  }
  else{
   P_rightFoot[0]= C.move2pose(stepLength/2,t,T_start+duration*alpha,T_end);
   P_rightFoot[2]=-l_thigh-l_leg+C.move2pose(stepHeight,t,T_start+duration*alpha,T_start+duration*(alpha+beta))-C.move2pose(stepHeight,t,T_start+duration*(alpha+beta),T_end);

  }
   P_pelvis[0]= C.move2pose(stepLength/2,t,T_start+duration*alpha,T_end);


}


void walk::stepping(bool left,double T_start,double T_end){
   double duration=T_end-T_start;
   double alpha=.5;
   double beta=.25;


   if(left){
     P_rightFoot[0]=
     P_leftFoot[0]=-stepLength/2+C.move2pose(stepLength,t,T_start+duration*alpha,T_end);
     P_leftFoot[2]=-l_thigh-l_leg+C.move2pose(stepHeight,t,T_start+duration*alpha,T_start+duration*(alpha+beta))-C.move2pose(stepHeight,t,T_start+duration*(alpha+beta),T_end);
     P_pelvis[1]=-y_pelvisMax+C.move2pose(2*y_pelvisMax,t,T_start,T_start+duration*alpha);
   }

   else{

    P_rightFoot[0]==-stepLength/2+C.move2pose(stepLength,t,T_start+duration*alpha,T_end);
    P_rightFoot[2]=-l_thigh-l_leg+C.move2pose(stepHeight,t,T_start+duration*alpha,T_start+duration*(alpha+beta))-C.move2pose(stepHeight,t,T_start+duration*(alpha+beta),T_end);
    P_pelvis[1]=y_pelvisMax+C.move2pose(-2*y_pelvisMax,t,T_start,T_start+duration*alpha);
   }



    P_pelvis[0]=-stepLength/2+C.move2pose(stepLength,t,T_start+duration*alpha,T_end);


}








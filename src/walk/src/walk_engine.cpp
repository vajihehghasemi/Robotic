#include <ros/ros.h>

#include "std_msgs/String.h"
#include <move_base/move_base.h>

#include "calc.h"
#include "walk.h"

#include<std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf/transform_listener.h>

#include <QFile>
#include <QTime>
#include <nav_msgs/Path.h>


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf/tf.h>

//#include <moveit_visual_tools/moveit_visual_tools.h>
static const std::string PLANNING_GROUP = "rightarm";



int walk_state=0; //0:stop 1:forward -1:bacward
int turn_state=0;//0:straght 1:lefr -1:right
void TwistCallback(const geometry_msgs::Twist& msg){
  walk_state=msg.linear.x;
  turn_state=msg.angular.z;
}

MatrixXd quater2rot(double w,double x,double y, double z){
  MatrixXd R(3,3);
  R<<w*w+x*x-y*y-z*z,2*x*y-2*w*z,2*x*z+2*w*y,
      2*x*y+2*w*z,w*w-x*x+y*y-z*z,2*y*z-2*w*x,
      2*x*z-2*w*y,2*y*z+2*w*x,w*w-x*x-y*y+z*z;
  return R;
}

double zmin_l_foot=0;
double zmin_r_foot=0;
double teta_motor_L,teta_motor_R;//pitch
double phi_motor_L,phi_motor_R;//roll
void ankle_states(const gazebo_msgs::LinkStates& msg){
  double x_left, x_right,y_left, y_right,z_left, z_right;
  Vector3d vec_A;
  Vector3d vec_B;
  Vector3d vec_C;
  Vector3d vec_D;
  Vector3d vec_E;
  Vector3d vec_F;
  Vector3d vec_G;
  Vector3d vec_H;
  vec_A<<.15,.05,-.07;
  vec_B<<.15,-.07,-.07;
  vec_C<<-.05,-.07,-.07;
  vec_D<<-.05,.05,-.07;

  vec_E<<.15,.07,-.07;
  vec_F<<.15,-.05,-.07;
  vec_G<<-.05,-.05,-.07;
  vec_H<<-.05,.07,-.07;


  int r=1,l=3;


  for (int i = 0; i < msg.name.size(); ++i) {
    if (msg.name[i]=="robot::left_foot") {
      l=i;
    }

    if (msg.name[i]=="robot::right_foot") {
      r=i;
    }
  }


  x_left=msg.pose[l].position.x;
  x_right=msg.pose[r].position.x;
  y_left=msg.pose[l].position.y;
  y_right=msg.pose[r].position.y;
  z_left=msg.pose[l].position.z;
  z_right=msg.pose[r].position.z;




  MatrixXd R_left(3,3);
  MatrixXd R_right(3,3);
  R_left=quater2rot(msg.pose[l].orientation.w,msg.pose[l].orientation.x,msg.pose[l].orientation.y,msg.pose[l].orientation.z);
  R_right=quater2rot(msg.pose[r].orientation.w,msg.pose[r].orientation.x,msg.pose[r].orientation.y,msg.pose[r].orientation.z);
  Vector3d temp;
  temp=R_left*vec_A;    double A=temp(2)+z_left;
  temp=R_left*vec_B;    double B=temp(2)+z_left;
  temp=R_left*vec_C;    double C=temp(2)+z_left;
  temp=R_left*vec_D;    double D=temp(2)+z_left;

  temp=R_right*vec_E;   double E=temp(2)+z_right;
  temp=R_right*vec_F;   double F=temp(2)+z_right;
  temp=R_right*vec_G;   double G=temp(2)+z_right;
  temp=R_right*vec_H;   double H=temp(2)+z_right;


  teta_motor_L=(A+B-C-D)/.2;
  teta_motor_R=(E+F-G-H)/.2;
  phi_motor_L=(A-B-C+D)/.12;
  phi_motor_R=(E-F-D+H)/.12;



  zmin_l_foot=min(min(A,B),min(C,D));
  zmin_r_foot=min(min(E,F),min(G,H));



}




QVector<double> X(1),Y(1),X_l(1),Y_l(1),X_r(1),Y_r(1),th(1);
walk W;
calc C;
QVector<double> steps_p_x(1);
QVector<double> steps_p_y(1);
QVector<double> steps_p_th(1);
QVector<double> steps_l_x(1);
QVector<double> steps_l_y(1);
QVector<double> steps_l_th(1);
QVector<double> steps_r_x(1);
QVector<double> steps_r_y(1);
QVector<double> steps_r_th(1);

int p;
int l;
int r;

double t0=2;
double t1=5;




void pathplanning(const nav_msgs::Path& msg){
if(msg.poses.size()>0 /*&& W.stepHeight==0*/){

  qDebug()<<"New Path Data recieved";
W.t=t1;
W.stepHeight=0.03;



  X.clear();
  Y.clear();
  X_l.clear();
  Y_l.clear();
  X_r.clear();
  Y_r.clear();
  th.clear();
//  X.append(0);
//  Y.append(0);
//  X_l.append(0);
//  Y_l.append(W.l_pelvis/2);
//  X_r.append(0);
//  Y_r.append(-W.l_pelvis/2);
//  th.append(0);


//  steps_l_x[0]=0;
//  steps_l_y[0]=W.l_pelvis/2;
//  steps_l_th[0]=0;

//  steps_r_x[0]=0;
//  steps_r_y[0]=-W.l_pelvis/2;
//  steps_r_th[0]=0;

//  steps_p_x[0] =0;
//  steps_p_y[0] =0;
//  steps_p_th[0]=0;


//  th.append(th.last()+dth);
//  X.append(X.last()+v*cos(th.last())*dt);
//  Y.append(Y.last()+v*sin(th.last())*dt);

//  X_l.append(X.last()-W.l_pelvis/2*sin(th.last()));
//  Y_l.append(Y.last()+W.l_pelvis/2*cos(th.last()));
//  X_r.append(X.last()+W.l_pelvis/2*sin(th.last()));
//  Y_r.append(Y.last()-W.l_pelvis/2*cos(th.last()));


  for (int i = 0; i < msg.poses.size(); ++i) {
X.append(msg.poses[i].pose.position.x);
Y.append(msg.poses[i].pose.position.y);

  }

  for (int i = 0; i<X.size()-1; ++i) {
    th.append(atan2(Y[i+1]-Y[i],X[i+1]-X[i]));
//    if(th.last()>M_PI){
//      th[th.size()-1]=2*M_PI-th.last();
//    }
  }

  th.append(th.last());

  for (int i = 0; i < msg.poses.size(); ++i){
    X_l.append(X[i]-W.l_pelvis/2*sin(th[i]));
    Y_l.append(Y[i]+W.l_pelvis/2*cos(th[i]));
    X_r.append(X[i]+W.l_pelvis/2*sin(th[i]));
    Y_r.append(Y[i]-W.l_pelvis/2*cos(th[i]));
  }
qDebug()<<"path updated";

  W.P_rightFoot[0]=X_r[0];
  W.P_rightFoot[1]=Y_r[0];
  W.Yaw_rightFoot=th[0];
  W.P_leftFoot[0]=X_l[0];
  W.P_leftFoot[1]=Y_l[0];
  W.Yaw_leftFoot=th[0];
  W.P_pelvis[0]=X[0];
  W.P_pelvis[1]=Y[0];
  W.Yaw_pelvis=th[0];
qDebug()<<"model updated";
//  qDebug()<<"Data recieved";
//  QByteArray log_data;
//  for (int i = 0; i < X.size(); ++i) {
//    log_data.append(QString::number(X[i])+","+
//                    QString::number(Y[i])+","+
//                    QString::number(X_l[i])+","+
//                    QString::number(Y_l[i])+","+
//                    QString::number(X_r[i])+","+
//                    QString::number(Y_r[i])+"\n");

//  }
//  QTime pc_time;
//QString name="Desktop/log/logW"+QString::number(pc_time.currentTime().hour())+"_"+QString::number(pc_time.currentTime().minute())+"_"+QString::number(pc_time.currentTime().second())+".txt";
//  QFile myfile(name);
//  myfile.remove();
//  myfile.open(QFile::ReadWrite);
//  myfile.write(log_data);
//  myfile.close();

//qDebug()<<"Log saved to "<<name;


steps_l_x.clear();
steps_l_y.clear();
steps_l_th.clear();
steps_r_x.clear();
steps_r_y.clear();
steps_r_th.clear();
steps_p_x.clear();
steps_p_y.clear();
steps_p_th.clear();

steps_l_x.append(X_l[0]);
steps_l_y.append(Y_l[0]);
steps_l_th.append(th[0]);

steps_r_x.append(X_r[0]);
steps_r_y.append(Y_r[0]);
steps_r_th.append(th[0]);

steps_p_x.append(X[0]);
steps_p_y.append(Y[0]);
steps_p_th.append(th[0]);




  double d_l=0;

  for (int i = 0; i < X_l.size()-1; i++) {

//    d_l+=C.distance(X_l[i],Y_l[i],X_l[i+1],Y_l[i+1]);
    d_l+=C.distance(X[i],Y[i],X[i+1],Y[i+1]);
    if(steps_l_x.size()==1 && d_l>W.stepLength){
      steps_l_x.append(X_l[i]);
      steps_l_y.append(Y_l[i]);
      steps_l_th.append(th[i]);
      d_l=0;
    }
    if(d_l>2*W.stepLength){
      steps_l_x.append(X_l[i]);
      steps_l_y.append(Y_l[i]);
      steps_l_th.append(th[i]);
      d_l=0;
    }

  }
//  qDebug()<<"left foot Stepping pattern generated"<<steps_l_x.size();
//  for (int i = 0; i < X_l.size()-m; i+=2*m) {
//    steps_l_x.append(X_l[m+i]);
//    steps_l_y.append(Y_l[m+i]);
//    steps_l_th.append(th[m+i]);
//  }

  steps_l_x.append(X_l.last());
  steps_l_y.append(Y_l.last());
  steps_l_th.append(th.last());


  double d_r=0;
  for (int i = 0; i < X_r.size()-1; i++) {

//    d_r+=C.distance(X_r[i],Y_r[i],X_r[i+1],Y_r[i+1]);
    d_r+=C.distance(X[i],Y[i],X[i+1],Y[i+1]);
    if(d_r>2*W.stepLength){
      steps_r_x.append(X_r[i]);
      steps_r_y.append(Y_r[i]);
      steps_r_th.append(th[i]);
      d_r=0;
    }

  }

//  for (int i = 0; i < X_r.size()-2*m; i+=2*m) {
//    steps_r_x.append(X_r[2*m+i]);
//    steps_r_y.append(Y_r[2*m+i]);
//    steps_r_th.append(th[2*m+i]);

//  }



  steps_r_x.append(X_r.last());
  steps_r_y.append(Y_r.last());
  steps_r_th.append(th.last());

//  qDebug()<<"right foot Stepping pattern generated"<<steps_r_x.size();




  for (int i = 0; i < steps_l_x.size()+steps_r_x.size()-1; ++i) {
    if(fmod(i,2)==0){
      steps_p_x.append(steps_r_x[i/2]);
      steps_p_y.append(steps_r_y[i/2]);
      steps_p_th.append(steps_r_th[i/2]);
    }
    else{
      steps_p_x.append(steps_l_x[(i+1)/2]);
      steps_p_y.append(steps_l_y[(i+1)/2]);
      steps_p_th.append(steps_l_th[(i+1)/2]);
    }
  }

  //  steps_p_x.append((steps_l_x.last()+steps_r_x.last())/2);
  //  steps_p_y.append((steps_l_y.last()+steps_r_y.last())/2);
  //  steps_p_th.append((steps_l_th.last()+steps_r_th.last())/2);

  steps_p_x[steps_p_x.size()-1]=((steps_l_x.last()+steps_r_x.last())/2);
  steps_p_y[steps_p_y.size()-1]=((steps_l_y.last()+steps_r_y.last())/2);
  steps_p_th[steps_p_th.size()-1]=((steps_l_th.last()+steps_r_th.last())/2);

   qDebug()<<"Stepping pattern generated";



//  QByteArray log_dataSP;
//  for (int i = 0; i < steps_p_x.size(); ++i) {
//    log_dataSP.append(QString::number(steps_p_x[i])+","+
//                     QString::number(steps_p_y[i])+","+
//                     QString::number(steps_p_th[i])+"\n");

//  }
//  QByteArray log_dataSR;
//  for (int i = 0; i < steps_r_x.size(); ++i) {
//    log_dataSR.append(QString::number(steps_r_x[i])+","+
//                     QString::number(steps_r_y[i])+","+
//                     QString::number(steps_r_th[i])+"\n");

//  }
//  QByteArray log_dataSL;
//  for (int i = 0; i < steps_l_x.size(); ++i) {
//    log_dataSL.append(QString::number(steps_l_x[i])+","+
//                     QString::number(steps_l_y[i])+","+
//                     QString::number(steps_l_th[i])+"\n");

//  }


//  name="Desktop/log/logSP"+QString::number(pc_time.currentTime().hour())+"_"+QString::number(pc_time.currentTime().minute())+"_"+QString::number(pc_time.currentTime().second())+".txt";
//  QFile myfileSP(name);
//  myfileSP.remove();
//  myfileSP.open(QFile::ReadWrite);
//  myfileSP.write(log_dataSP);
//  myfileSP.close();

//  name="Desktop/log/logSL"+QString::number(pc_time.currentTime().hour())+"_"+QString::number(pc_time.currentTime().minute())+"_"+QString::number(pc_time.currentTime().second())+".txt";
//  QFile myfileSL(name);
//  myfileSL.remove();
//  myfileSL.open(QFile::ReadWrite);
//  myfileSL.write(log_dataSL);
//  myfileSL.close();

//  name="Desktop/log/logSR"+QString::number(pc_time.currentTime().hour())+"_"+QString::number(pc_time.currentTime().minute())+"_"+QString::number(pc_time.currentTime().second())+".txt";
//  QFile myfileSR(name);
//  myfileSR.remove();
//  myfileSR.open(QFile::ReadWrite);
//  myfileSR.write(log_dataSR);
//  myfileSR.close();
}

}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "walk_engine");
  ros::NodeHandle nh;
  ros::Rate rr(200);

  ros::Publisher pubH = nh.advertise<std_msgs::Float64>("/human_model/base_link_to_human_head_position_controller/command",100);
//  ros::Publisher pubArm_L = nh.advertise<std_msgs::Float64>("/human_model/left_arm_to_forearm_position_controller/command",100);
//  ros::Publisher pubFArm_L = nh.advertise<std_msgs::Float64>("/human_model/left_forearm_to_base_link_position_controller/command",100);
//  ros::Publisher pubArm_R = nh.advertise<std_msgs::Float64>("/human_model/right_arm_to_forearm_position_controller/command",100);
//  ros::Publisher pubFArm_R = nh.advertise<std_msgs::Float64>("/human_model/right_forearm_to_base_link_position_controller/command",100);


  ros::Publisher pub_K_P_L = nh.advertise<std_msgs::Float64>("/human_model/left_leg_to_left_thigh_position_controller/command",100);
  ros::Publisher pub_H_R_L = nh.advertise<std_msgs::Float64>("/human_model/left_thigh_to_base_linkRoll_position_controller/command",100);
  ros::Publisher pub_H_Y_L = nh.advertise<std_msgs::Float64>("/human_model/left_thigh_to_base_linkYaw_position_controller/command",100);
  ros::Publisher pub_H_P_L = nh.advertise<std_msgs::Float64>("/human_model/left_thigh_to_base_link_position_controller/command",100);
  ros::Publisher pub_A_R_L = nh.advertise<std_msgs::Float64>("/human_model/left_foot_to_left_ankle_position_controller/command",100);
  ros::Publisher pub_A_P_L = nh.advertise<std_msgs::Float64>("/human_model/left_leg_to_left_ankle_position_controller/command",100);


  ros::Publisher pub_K_P_R = nh.advertise<std_msgs::Float64>("/human_model/right_leg_to_right_thigh_position_controller/command",100);
  ros::Publisher pub_H_R_R = nh.advertise<std_msgs::Float64>("/human_model/right_thigh_to_base_linkRoll_position_controller/command",100);
  ros::Publisher pub_H_Y_R = nh.advertise<std_msgs::Float64>("/human_model/right_thigh_to_base_linkYaw_position_controller/command",100);
  ros::Publisher pub_H_P_R = nh.advertise<std_msgs::Float64>("/human_model/right_thigh_to_base_link_position_controller/command",100);
  ros::Publisher pub_A_R_R = nh.advertise<std_msgs::Float64>("/human_model/right_foot_to_right_ankle_position_controller/command",100);
  ros::Publisher pub_A_P_R = nh.advertise<std_msgs::Float64>("/human_model/right_leg_to_right_ankle_position_controller/command",100);

  ros::Subscriber ankleStates = nh.subscribe("/gazebo/link_states", 100, ankle_states);
  ros::Subscriber pathplanningsub= nh.subscribe("/move_base/NavfnROS/plan", 100, pathplanning);



  X.clear();
  Y.clear();
  X_l.clear();
  Y_l.clear();
  X_r.clear();
  Y_r.clear();
  th.clear();
  X.append(0);
  Y.append(0);
  X_l.append(0);
  Y_l.append(W.l_pelvis/2);
  X_r.append(0);
  Y_r.append(-W.l_pelvis/2);
  th.append(0);


  steps_l_x[0]=0;
  steps_l_y[0]=W.l_pelvis/2;
  steps_l_th[0]=0;

  steps_r_x[0]=0;
  steps_r_y[0]=-W.l_pelvis/2;
  steps_r_th[0]=0;

  steps_p_x[0] =0;
  steps_p_y[0] =0;
  steps_p_th[0]=0;


/// define a point base path planning
if(!true){
  QVector<double> P_x(1),P_y(1);
  P_x[0]=0; P_y[0]=0;
    //P=[0,0;0,1.5;1.5,2;1.5,-2;-2,-2];
  P_x.append(0);    P_y.append(1.5);
  P_x.append(1.5);  P_y.append(2);
  P_x.append(1.5);    P_y.append(-2);
  P_x.append(-2);  P_y.append(-2);


  int i_p=0;
  double d=C.distance(X.last(),Y.last(),P_x[i_p+1],P_y[i_p+1]);
  double th_des=atan2(P_y[i_p+1]-Y.last(),P_x[i_p+1]-X.last());
  double th_des_past=th_des;
  double d_min=.01;
  double len=W.stepLength;
  double v=len/9;
  double dt=.005;
  double radius=.4;
  double d_th_max=v/radius*dt;
  double dth;

  while (d>d_min) {
    th_des=atan2(P_y[i_p+1]-Y.last(),P_x[i_p+1]-X.last());
    while (th_des-th_des_past>M_PI){
      th_des=th_des-M_PI*2;
    }
    while (th_des-th_des_past<-M_PI){
      th_des=th_des+M_PI*2;
    }
    th_des_past=th_des;

    dth=th_des-th.last();
    if (dth<-d_th_max){
      dth=-d_th_max;
    }
    else if (dth>d_th_max){
      dth=d_th_max;
    }

    th.append(th.last()+dth);
    X.append(X.last()+v*cos(th.last())*dt);
    Y.append(Y.last()+v*sin(th.last())*dt);

    X_l.append(X.last()-W.l_pelvis/2*sin(th.last()));
    Y_l.append(Y.last()+W.l_pelvis/2*cos(th.last()));
    X_r.append(X.last()+W.l_pelvis/2*sin(th.last()));
    Y_r.append(Y.last()-W.l_pelvis/2*cos(th.last()));

    d=C.distance(X.last(),Y.last(),P_x[i_p+1],P_y[i_p+1]);


    if (d<d_min){
      i_p=i_p+1;
      if (i_p>P_x.size()-2){
        break;
      }
      d=C.distance(X.last(),Y.last(),P_x[i_p+1],P_y[i_p+1]);
    }
    if (X.size()>1e6){
      qDebug()<<"pattern generation error!";
      return 0;
    }

  }
  double m=len/v/dt;

  for (int i = 0; i < X_l.size()-m; i+=2*m) {
    steps_l_x.append(X_l[m+i]);
    steps_l_y.append(Y_l[m+i]);
    steps_l_th.append(th[m+i]);
  }
  steps_l_x.append(X_l.last());
  steps_l_y.append(Y_l.last());
  steps_l_th.append(th.last());
  for (int i = 0; i < X_r.size()-2*m; i+=2*m) {
    steps_r_x.append(X_r[2*m+i]);
    steps_r_y.append(Y_r[2*m+i]);
    steps_r_th.append(th[2*m+i]);

  }
  steps_r_x.append(X_r.last());
  steps_r_y.append(Y_r.last());
  steps_r_th.append(th.last());





  for (int i = 0; i < steps_l_x.size()+steps_r_x.size()-1; ++i) {
    if(fmod(i,2)==0){
      steps_p_x.append(steps_r_x[i/2]);
      steps_p_y.append(steps_r_y[i/2]);
      steps_p_th.append(steps_r_th[i/2]);
    }
    else{
      steps_p_x.append(steps_l_x[(i+1)/2]);
      steps_p_y.append(steps_l_y[(i+1)/2]);
      steps_p_th.append(steps_l_th[(i+1)/2]);
    }
  }

  //  steps_p_x.append((steps_l_x.last()+steps_r_x.last())/2);
  //  steps_p_y.append((steps_l_y.last()+steps_r_y.last())/2);
  //  steps_p_th.append((steps_l_th.last()+steps_r_th.last())/2);

  steps_p_x[steps_p_x.size()-1]=((steps_l_x.last()+steps_r_x.last())/2);
  steps_p_y[steps_p_y.size()-1]=((steps_l_y.last()+steps_r_y.last())/2);
  steps_p_th[steps_p_th.size()-1]=((steps_l_th.last()+steps_r_th.last())/2);



}



  ros::Subscriber walk_sub = nh.subscribe("/humanTwist", 1000, TwistCallback);

  double zsave_l_foot=-W.l_thigh-W.l_leg;
  double zsave_r_foot=-W.l_thigh-W.l_leg;

  double zpast_l_foot=-W.l_thigh-W.l_leg;
  double zpast_r_foot=-W.l_thigh-W.l_leg;


  bool l_down;
  bool r_down;

  bool l_up;
  bool r_up;

  bool l_fix;
  bool r_fix;

  bool l_hit=true;
  bool r_hit=true;

  double t_hit_l;
  double t_hit_r;
  double z_hit_l;
  double z_hit_r;
  W.P_leftFoot[0]=0;
  W.P_leftFoot[1]=W.l_pelvis/2;
  W.P_leftFoot[2]=-W.l_thigh-W.l_leg;
  W.P_rightFoot[0]=0;
  W.P_rightFoot[1]=-W.l_pelvis/2;
  W.P_rightFoot[2]=-W.l_thigh-W.l_leg;
  W.P_pelvis[0]=0;
  W.P_pelvis[1]=0;
  W.P_pelvis[2]=0;
  W.Yaw_pelvis=0;
  W.Yaw_rightFoot=0;
  W.Yaw_leftFoot=0;




//  // move it
//  // The :move_group_interface:`MoveGroup` class can be easily
//    // setup using just the name of the planning group you would like to control and plan for.
//    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  
//    // We will use the :planning_scene_interface:`PlanningSceneInterface`
//    // class to add and remove collision objects in our "virtual world" scene
//    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
//    // Raw pointers are frequently used to refer to the planning group for improved performance.
//    const robot_state::JointModelGroup* joint_model_group =
//        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
  
  
  
//    // Planning to a Pose goal
//      // ^^^^^^^^^^^^^^^^^^^^^^^
//      // We can plan a motion for this group to a desired pose for the
//      // end-effector.
//      geometry_msgs::Pose target_pose1;
//      target_pose1.orientation.w = 1.0;
//      target_pose1.position.x = 0.28;
//      target_pose1.position.y = -0.2;
//      target_pose1.position.z = 0.5;
//      move_group.setPoseTarget(target_pose1);
    
//      // Now, we call the planner to compute the plan and visualize it.
//      // Note that we are just planning, not asking move_group
//      // to actually move the robot.
//      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
//      bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
      
      



  QByteArray log_data;

  while(ros::ok()){

    std_msgs::Float64 data;


    data.data=0;

    pubH.publish(data);
//    pubArm_L.publish(data);
//    pubFArm_L .publish(data);
//    pubArm_R.publish(data);
//    pubFArm_R .publish(data);


    W.t+=.005;



    W.lower_height(-.02, t0, t1);
    p=int((W.t-t1)/(W.T_pelvis+W.T_stepping))+1;
    l=int((W.t-t1-W.T_pelvis)/(2*(W.T_pelvis+W.T_stepping)))+1;
    r=max(int((W.t-t1-W.T_pelvis*2-W.T_stepping)/(2*(W.T_pelvis+W.T_stepping)))+1,0);


    if(p>=steps_p_x.size() && l>=steps_l_x.size()&& r>=steps_r_x.size()){
      W.stepHeight=0;
    }


    if(p>=steps_p_x.size()){
      p=steps_p_x.size()-1;
    }

    if(l>=steps_l_x.size()){
      l=steps_l_x.size()-1;
    }

    if(r>=steps_r_x.size()){
      r=steps_r_x.size()-1;
    }





    W.P_pelvis[0]=steps_p_x[p]+C.move2zero(W.P_pelvis[0]-steps_p_x[p],W.t-t1-(W.T_pelvis+W.T_stepping)*(p-1),W.T_pelvis);
    W.P_pelvis[1]=steps_p_y[p]+C.move2zero(W.P_pelvis[1]-steps_p_y[p],W.t-t1-(W.T_pelvis+W.T_stepping)*(p-1),W.T_pelvis);
    W.Yaw_pelvis=steps_p_th[p]+C.move2zero(W.Yaw_pelvis-steps_p_th[p],W.t-t1-(W.T_pelvis+W.T_stepping)*(p-1),W.T_pelvis);

    W.P_rightFoot[0]=steps_r_x[r]+C.move2zero(W.P_rightFoot[0]-steps_r_x[r],W.t-t1-W.T_pelvis-(W.T_pelvis+W.T_stepping)*(1+2*(r-1))-W.T_stepping/10,W.T_stepping-W.T_stepping/5);
    W.P_rightFoot[1]=steps_r_y[r]+C.move2zero(W.P_rightFoot[1]-steps_r_y[r],W.t-t1-W.T_pelvis-(W.T_pelvis+W.T_stepping)*(1+2*(r-1))-W.T_stepping/10,W.T_stepping-W.T_stepping/5);
    W.P_rightFoot[2]=-W.l_thigh-W.l_leg+C.move2pose(W.stepHeight,W.t-t1-W.T_pelvis-(W.T_pelvis+W.T_stepping)*(1+2*(r-1)),0,W.T_stepping/2)-C.move2pose(W.stepHeight,W.t-t1-W.T_pelvis-(W.T_pelvis+W.T_stepping)*(1+2*(r-1)),W.T_stepping/2,W.T_stepping);
    W.Yaw_rightFoot=steps_r_th[r]+C.move2zero(W.Yaw_rightFoot-steps_r_th[r],W.t-t1-W.T_pelvis-(W.T_pelvis+W.T_stepping)*(1+2*(r-1))-W.T_stepping/10,W.T_stepping-W.T_stepping/5);

    W.P_leftFoot[0]=steps_l_x[l]+C.move2zero(W.P_leftFoot[0]-steps_l_x[l],W.t-t1-W.T_pelvis-(W.T_pelvis+W.T_stepping)*2*(l-1)-W.T_stepping/10,W.T_stepping-W.T_stepping/5);
    W.P_leftFoot[1]=steps_l_y[l]+C.move2zero(W.P_leftFoot[1]-steps_l_y[l],W.t-t1-W.T_pelvis-(W.T_pelvis+W.T_stepping)*2*(l-1)-W.T_stepping/10,W.T_stepping-W.T_stepping/5);
    W.P_leftFoot[2]=-W.l_thigh-W.l_leg+C.move2pose(W.stepHeight,W.t-t1-W.T_pelvis-(W.T_pelvis+W.T_stepping)*2*(l-1),0,W.T_stepping/2)-C.move2pose(W.stepHeight,W.t-t1-W.T_pelvis-(W.T_pelvis+W.T_stepping)*2*(l-1),W.T_stepping/2,W.T_stepping);
    W.Yaw_leftFoot=steps_l_th[l]+C.move2zero(W.Yaw_leftFoot-steps_l_th[l],W.t-t1-W.T_pelvis-(W.T_pelvis+W.T_stepping)*2*(l-1)-W.T_stepping/10,W.T_stepping-W.T_stepping/5);


    ////////////////******************************

    l_down=(W.P_leftFoot[2]<zpast_l_foot-1e-8);
    r_down=(W.P_rightFoot[2]<zpast_r_foot-1e-8);

    l_up=(W.P_leftFoot[2]>zpast_l_foot+1e-8);
    r_up=(W.P_rightFoot[2]>zpast_r_foot+1e-8);

    l_fix=(!l_down)&&(!l_up);
    r_fix=(!r_down)&&(!r_up);

    //  qDebug()<<l_down<<"\t"<<l_fix<<"\t"<<l_up<<"\n"<<r_down<<"\t"<<r_fix<<"\t"<<r_up<<"\n**************************";

    //qDebug()<<zpast_l_foot-W.P_leftFoot[2];
    zpast_l_foot=W.P_leftFoot[2];
    zpast_r_foot=W.P_rightFoot[2];

    if(l_up){l_hit=true;}
    if(r_up){r_hit=true;}

    if(zmin_l_foot<.001 && l_hit &&l_down ){
      t_hit_l=W.t;
      l_hit=false;
      z_hit_l=zsave_l_foot;
      //        qDebug()<<"left foot hit at t="<<t_hit_l<<"  and z="<<z_hit_l+W.l_thigh+W.l_leg;
      qDebug()<<"x= "<<(W.P_leftFoot[0]+W.P_rightFoot[0])/2<<"\ty= "<<(W.P_leftFoot[1]+W.P_rightFoot[1])/2;
    }

    if(zmin_r_foot<.001 && r_hit &&r_down ){
      t_hit_r=W.t;
      r_hit=false;
      z_hit_r=zsave_r_foot;
      //        qDebug()<<"right foot hit at t="<<t_hit_r<<"  and z="<<z_hit_r+W.l_thigh+W.l_leg;
      qDebug()<<"x= "<<(W.P_leftFoot[0]+W.P_rightFoot[0])/2<<"\ty= "<<(W.P_leftFoot[1]+W.P_rightFoot[1])/2;
    }





    if(!l_hit){
      zsave_l_foot= (-W.l_thigh-W.l_leg)+C.move2zero(zsave_l_foot-(-W.l_thigh-W.l_leg),W.t-t_hit_l,1.5);
      W.P_leftFoot[2]=zsave_l_foot;

    }
    if(!r_hit){
      zsave_r_foot=(-W.l_thigh-W.l_leg)+C.move2zero(zsave_r_foot-(-W.l_thigh-W.l_leg),W.t-t_hit_r,1.5);
      W.P_rightFoot[2]=zsave_r_foot;

    }



    zsave_l_foot=W.P_leftFoot[2];
    zsave_r_foot=W.P_rightFoot[2];

    W.inverseKinematic();



    data.data=W.q_K_P_L;  pub_K_P_L.publish(data);
    data.data=W.q_H_R_L;  pub_H_R_L.publish(data);
    data.data=W.q_H_Y_L;  pub_H_Y_L.publish(data);
    data.data=W.q_H_P_L;  pub_H_P_L.publish(data);
    data.data=W.q_A_R_L;  pub_A_R_L.publish(data);
    data.data=W.q_A_P_L;  pub_A_P_L.publish(data);

    data.data=W.q_K_P_R;  pub_K_P_R.publish(data);
    data.data=W.q_H_R_R;  pub_H_R_R.publish(data);
    data.data=W.q_H_Y_R;  pub_H_Y_R.publish(data);
    data.data=W.q_H_P_R;  pub_H_P_R.publish(data);
    data.data=W.q_A_R_R;  pub_A_R_R.publish(data);
    data.data=W.q_A_P_R;  pub_A_P_R.publish(data);

    //    qDebug()<<"pelvis:";
    //    qDebug()<<"x= "<<W.P_pelvis[0];
    //    qDebug()<<"y= "<<W.P_pelvis[1];
    //    qDebug()<<"z= "<<W.P_pelvis[2];
    //    qDebug()<<"Yaw= "<<W.Yaw_pelvis;
    //    qDebug()<<"leftFoot:";
    //    qDebug()<<"x= "<<W.P_leftFoot[0];
    //    qDebug()<<"y= "<<W.P_leftFoot[1];
    //    qDebug()<<"z= "<<W.P_leftFoot[2];
    //    qDebug()<<"Yaw="<<W.Yaw_leftFoot;
    //    qDebug()<<"rightFoot:";
    //    qDebug()<<"x= "<<W.P_rightFoot[0];
    //    qDebug()<<"y= "<<W.P_rightFoot[1];
    //    qDebug()<<"z= "<<W.P_rightFoot[2];
    //    qDebug()<<"Yaw="<<W.Yaw_rightFoot;
    //    qDebug()<<"       ";

    //    qDebug()<<"K_P  L: "<<W.q_K_P_L<<"\tR: "<<W.q_K_P_R;
    //    qDebug()<<"H_R  L: "<<W.q_H_R_L<<"\tR: "<<W.q_H_R_R;
    //    qDebug()<<"H_Y  L: "<<W.q_H_Y_L<<"\tR: "<<W.q_H_Y_R;
    //    qDebug()<<"H_P  L: "<<W.q_H_P_L<<"\tR: "<<W.q_H_P_R;
    //    qDebug()<<"A_R  L: "<<W.q_A_R_L<<"\tR: "<<W.q_A_R_R;
    //    qDebug()<<"A_P  L: "<<W.q_A_P_L<<"\tR: "<<W.q_A_P_R;

    //    qDebug()<<"*****************************************";



    log_data.append(QString::number(W.t)+","+QString::number(W.P_pelvis[0])+","+QString::number(W.P_pelvis[1])+","+QString::number(W.P_pelvis[2])+","+QString::number(W.Yaw_pelvis)+","
        +QString::number(W.P_leftFoot[0])+","+QString::number(W.P_leftFoot[1])+","+QString::number(W.P_leftFoot[2])+","+QString::number(W.Yaw_leftFoot)+","
        +QString::number(W.P_rightFoot[0])+","+QString::number(W.P_rightFoot[1])+","+QString::number(W.P_rightFoot[2])+","+QString::number(W.Yaw_rightFoot)+"\n");



    ros::spinOnce();
    rr.sleep();
  }

  QFile myfile("/home/milad/Desktop/walklog.txt");
  myfile.remove();
  myfile.open(QFile::ReadWrite);
  myfile.write(log_data);
  myfile.close();

}

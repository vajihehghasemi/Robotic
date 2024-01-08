#include <ros/ros.h>
#include "std_msgs/String.h"
#include <move_base/move_base.h>

#include "calc.h"
#include "walk.h"

#include<std_msgs/Float64.h>

#include <tf/transform_listener.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_test");
  ros::NodeHandle nh;
  ros::Rate r(200);

  ros::Publisher pubH = nh.advertise<std_msgs::Float64>("/human_model/base_link_to_human_head_position_controller/command",100);
  ros::Publisher pubArm_L = nh.advertise<std_msgs::Float64>("/human_model/left_arm_to_forearm_position_controller/command",100);
  ros::Publisher pubFArm_L = nh.advertise<std_msgs::Float64>("/human_model/left_forearm_to_base_link_position_controller/command",100);
  ros::Publisher pubArm_R = nh.advertise<std_msgs::Float64>("/human_model/right_arm_to_forearm_position_controller/command",100);
  ros::Publisher pubFArm_R = nh.advertise<std_msgs::Float64>("/human_model/right_forearm_to_base_link_position_controller/command",100);


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

  walk W;
  calc C;

  //  /human_model/base_link_to_human_head_position_controller/command
  //  /human_model/camera_joint_position_controller/command
  //  /human_model/hokuyo_joint_position_controller/command
  //  /human_model/left_arm_to_forearm_position_controller/command
  //  /human_model/left_foot_to_left_ankle_position_controller/command
  //  /human_model/left_forearm_to_base_link_position_controller/command
  //  /human_model/left_leg_to_left_ankle_position_controller/command
  //  /human_model/left_leg_to_left_thigh_position_controller/command
  //  /human_model/left_thigh_to_base_linkRoll_position_controller/command
  //  /human_model/left_thigh_to_base_linkYaw_position_controller/command
  //  /human_model/left_thigh_to_base_link_position_controller/command
  //  /human_model/right_arm_to_forearm_position_controller/command
  //  /human_model/right_foot_to_right_ankle_position_controller/command
  //  /human_model/right_forearm_to_base_link_position_controller/command
  //  /human_model/right_leg_to_right_ankle_position_controller/command
  //  /human_model/right_leg_to_right_thigh_position_controller/command
  //  /human_model/right_thigh_to_base_linkRoll_position_controller/command
  //  /human_model/right_thigh_to_base_linkYaw_position_controller/command
  //  /human_model/right_thigh_to_base_link_position_controller/command





  while(ros::ok()){

    std_msgs::Float64 data;
    W.t+=.01;

    data.data=0;

    pubH.publish(data);
    pubArm_L.publish(data);
    pubFArm_L .publish(data);
    pubArm_R.publish(data);
    pubFArm_R .publish(data);



    W.P_leftFoot[0]=0;
    W.P_leftFoot[1]=W.l_pelvis/2;
    W.P_leftFoot[2]=-W.l_thigh-W.l_leg;
    W.P_rightFoot[0]=0;
    W.P_rightFoot[1]=-W.l_pelvis/2;
    W.P_rightFoot[2]=-W.l_thigh-W.l_leg;
    W.P_pelvis[0]=0;
    W.P_pelvis[1]=0;
    W.P_pelvis[2]=0;
    double t0=2;
    double t1=5;
    W.lower_height(-.02, t0, t1);
    //    W.P_pelvis[1]+=C.move2pose(.11,W.t,5,8);
    //    W.P_rightFoot[0]+=C.move2pose(.05,W.t,8.5,11.5);
    //    W.P_rightFoot[2]+=C.move2pose(.02,W.t,8,9.5)-C.move2pose(.02,W.t,11.5,15);


    //    W.P_pelvis[1]+=C.move2pose(-.22,W.t,15.5,20);
    //    W.P_pelvis[0]+=C.move2pose(.05,W.t,15.5,20);
    //    W.P_leftFoot[0]+=C.move2pose(.1,W.t,20.5,23.5);
    //    W.P_leftFoot[2]+=C.move2pose(.02,W.t,20.5,22)-C.move2pose(.02,W.t,23.5,26);

    //    W.P_pelvis[1]+=C.move2pose(.22,W.t,26,30.5);
    //    W.P_pelvis[0]+=C.move2pose(.1,W.t,26,30.5);
    //    W.P_rightFoot[0]+=C.move2pose(.1,W.t,31,34);
    //    W.P_rightFoot[2]+=(C.move2pose(.02,W.t,31,32.5)-C.move2pose(.02,W.t,34,37.5));

    //    W.P_pelvis[1]+=C.move2pose(-.22,W.t,26+11.5,30.5+11.5);
    //    W.P_pelvis[0]+=C.move2pose(.1,W.t,26+11.5,30.5+11.5);
    //    W.P_leftFoot[0]+=C.move2pose(.1,W.t,31+11.5,34+11.5);
    //    W.P_leftFoot[2]+=(C.move2pose(.02,W.t,31+11.5,32.5+11.5)-C.move2pose(.02,W.t,34+11.5,37.5+11.5));

    //    W.P_pelvis[1]+=C.move2pose(.22,W.t,26+23,30.5+23);
    //    W.P_pelvis[0]+=C.move2pose(.1,W.t,26+23,30.5+23);
    //    W.P_rightFoot[0]+=C.move2pose(.1,W.t,31+23,34+23);
    //    W.P_rightFoot[2]+=(C.move2pose(.02,W.t,31+23,32.5+23)-C.move2pose(.02,W.t,34+23,37.5+23));
    double ym=.06;
    double zm=.02;
    double l=.15;
    double T1=3;
    double T2=3;
    double T3=3;
    W.P_pelvis[1]+=C.move2pose(ym,W.t-t1,0,T1);
    W.P_rightFoot[0]+=C.move2pose(l/2,W.t-t1-T1,0,T2);
    W.P_rightFoot[2]+=C.move2pose(zm,W.t-t1-T1,0,T2/2)-C.move2pose(zm,W.t-t1-T1,T2/2,T2);


    for (int i = 0; i < 10; ++i) {

      W.P_pelvis[0]+=C.move2pose(l/2,W.t-t1-(T1+T2)-2*i*(T1+T2+T3),0,T3+T1);
      W.P_pelvis[1]+=C.move2pose(-ym,W.t-t1-(T1+T2)-2*i*(T1+T2+T3),0,T3);

      W.P_pelvis[1]+=C.move2pose(-ym,W.t-t1-(T1+T2+T3)-2*i*(T1+T2+T3),0,T1);
      W.P_leftFoot[0]+=C.move2pose(l,W.t-t1-T1-(T1+T2+T3)-2*i*(T1+T2+T3),0,T2);
      W.P_leftFoot[2]+=C.move2pose(zm,W.t-t1-T1-(T1+T2+T3)-2*i*(T1+T2+T3),0,T2/2)-C.move2pose(zm,W.t-t1-T1-(T1+T2+T3)-2*i*(T1+T2+T3),T2/2,T2);

      W.P_pelvis[0]+=C.move2pose(l/2,W.t-t1-(T1+T2)-(2*i+1)*(T1+T2+T3),0,T3+T1);
      W.P_pelvis[1]+=C.move2pose(ym,W.t-t1-(T1+T2)-(2*i+1)*(T1+T2+T3),0,T3);

      W.P_pelvis[1]+=C.move2pose(ym,W.t-t1-(T1+T2+T3)-(2*i+1)*(T1+T2+T3),0,T1);
      W.P_rightFoot[0]+=C.move2pose(l,W.t-t1-T1-(T1+T2+T3)-(2*i+1)*(T1+T2+T3),0,T2);
      W.P_rightFoot[2]+=C.move2pose(zm,W.t-t1-T1-(T1+T2+T3)-(2*i+1)*(T1+T2+T3),0,T2/2)-C.move2pose(zm,W.t-t1-T1-(T1+T2+T3)-(2*i+1)*(T1+T2+T3),T2/2,T2);



    }


    //W.firstStep(false,5,8);
    //    if(t<=8&t>=5){
    //      W.P_pelvis[1]=C.polynomial(t,5,8,0,0,0,W.y_pelvisMax,0,.1);
    //      W.P_rightFoot[2]=
    //    }


    //    double w=sqrt(10/.7);
    //    if(W.t>=6){
    //    W.P_pelvis[1]=W.y_pelvisMax*sin(w*W.t-6)*C.move2pose(1,W.t,6,6.5);
    //}
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




    qDebug()<<W.P_leftFoot[0];
    qDebug()<<W.P_leftFoot[1];
    qDebug()<<W.P_leftFoot[2];
    qDebug()<<W.P_rightFoot[0];
    qDebug()<<W.P_rightFoot[1];
    qDebug()<<W.P_rightFoot[2];
    qDebug()<<W.P_pelvis[0];
    qDebug()<<W.P_pelvis[1];
    qDebug()<<W.P_pelvis[2];
    qDebug()<<"       ";

    qDebug()<<W.q_K_P_L<<"\t"<<W.q_K_P_R;
    qDebug()<<W.q_H_R_L<<"\t"<<W.q_H_R_R;
    qDebug()<<W.q_H_Y_L<<"\t"<<W.q_H_Y_R;
    qDebug()<<W.q_H_P_L<<"\t"<<W.q_H_P_R;
    qDebug()<<W.q_A_R_L<<"\t"<<W.q_A_R_R;
    qDebug()<<W.q_A_P_L<<"\t"<<W.q_A_P_R;

    qDebug()<<"*****************************************";







    ros::spinOnce();
    r.sleep();
  }
}

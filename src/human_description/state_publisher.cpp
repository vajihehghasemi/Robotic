#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include<std_msgs/Float64.h>


int main(int argc, char** argv) {
ros::init(argc, argv, "state_publisher");
ros::NodeHandle n;
ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
tf::TransformBroadcaster broadcaster;
ros::Rate loop_rate(30);

const double degree = M_PI/180;
double rot4 = 90;


    // robot state
    double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
//    odom_trans.header.frame_id = "hokuyo_link";

    odom_trans.child_frame_id = "base_link";
  //  odom_trans.child_frame_id = "left_foot";



//joint_state.name.resize(14);
//joint_state.position.resize(14);
//joint_state.name[0] ="base_link_to_human_head";
//joint_state.name[1] ="right_forearm_to_base_link";
//joint_state.name[2] ="right_arm_to_forearm";
//joint_state.name[3] ="left_forearm_to_base_link";
//joint_state.name[4] ="left_arm_to_forearm";
//joint_state.name[5] ="right_thigh_to_base_link";
//joint_state.name[6] ="left_thigh_to_base_link";
//joint_state.name[7] ="right_leg_to_right_thigh";
//joint_state.name[8] ="left_leg_to_left_thigh";
//joint_state.name[9] ="right_leg_to_right_ankle";
//joint_state.name[10] ="left_leg_to_left_ankle";
//joint_state.name[11] ="right_foot_to_right_ankle";
//joint_state.name[12] ="left_foot_to_left_ankle";
//joint_state.name[13] ="base_laser_to_base_link";

joint_state.name.resize(23);
joint_state.position.resize(23);
joint_state.name[0] ="base_link_to_human_head";
joint_state.name[1] ="right_forearm_to_base_linkRoll";
joint_state.name[2] ="right_forearm_to_base_linkPitch";
joint_state.name[3] ="right_forearm_to_base_linkYaw";
joint_state.name[4] ="right_arm_to_forearm";
joint_state.name[5] ="left_forearm_to_base_linkRoll";
joint_state.name[6] ="left_forearm_to_base_linkPitch";
joint_state.name[7] ="left_forearm_to_base_linkYaw";
joint_state.name[8] ="left_arm_to_forearm";
joint_state.name[9] ="right_thigh_to_base_linkYaw";
joint_state.name[10] ="right_thigh_to_base_linkRoll";
joint_state.name[11] ="right_thigh_to_base_link";
joint_state.name[12] ="left_thigh_to_base_linkYaw";
joint_state.name[13] ="left_thigh_to_base_linkRoll";
joint_state.name[14] ="left_thigh_to_base_link";
joint_state.name[15] ="right_leg_to_right_thigh";
joint_state.name[16] ="left_leg_to_left_thigh";
joint_state.name[17] ="right_leg_to_right_ankle";
joint_state.name[18] ="left_leg_to_left_ankle";
joint_state.name[19] ="right_foot_to_right_ankle";
joint_state.name[20] ="left_foot_to_left_ankle";
joint_state.name[21] ="right_foot_to_right_ankle";
joint_state.name[22] ="left_foot_to_left_ankle";
//joint_state.name[23] ="base_laser_to_base_link";




ros::Publisher pubH = n.advertise<std_msgs::Float64>("/human_model/base_link_to_human_head_position_controller/command",100);
ros::Publisher pubArm_L = n.advertise<std_msgs::Float64>("/human_model/left_arm_to_forearm_position_controller/command",100);
ros::Publisher pubFArm_L = n.advertise<std_msgs::Float64>("/human_model/left_forearm_to_base_link_position_controller/command",100);
ros::Publisher pubArm_R = n.advertise<std_msgs::Float64>("/human_model/right_arm_to_forearm_position_controller/command",100);
ros::Publisher pubFArm_R = n.advertise<std_msgs::Float64>("/human_model/right_forearm_to_base_link_position_controller/command",100);


ros::Publisher pub_K_P_L = n.advertise<std_msgs::Float64>("/human_model/left_leg_to_left_thigh_position_controller/command",100);
ros::Publisher pub_H_R_L = n.advertise<std_msgs::Float64>("/human_model/left_thigh_to_base_linkRoll_position_controller/command",100);
ros::Publisher pub_H_Y_L = n.advertise<std_msgs::Float64>("/human_model/left_thigh_to_base_linkYaw_position_controller/command",100);
ros::Publisher pub_H_P_L = n.advertise<std_msgs::Float64>("/human_model/left_thigh_to_base_link_position_controller/command",100);
ros::Publisher pub_A_R_L = n.advertise<std_msgs::Float64>("/human_model/left_foot_to_left_ankle_position_controller/command",100);
ros::Publisher pub_A_P_L = n.advertise<std_msgs::Float64>("/human_model/left_leg_to_left_ankle_position_controller/command",100);


ros::Publisher pub_K_P_R = n.advertise<std_msgs::Float64>("/human_model/right_leg_to_right_thigh_position_controller/command",100);
ros::Publisher pub_H_R_R = n.advertise<std_msgs::Float64>("/human_model/right_thigh_to_base_linkRoll_position_controller/command",100);
ros::Publisher pub_H_Y_R = n.advertise<std_msgs::Float64>("/human_model/right_thigh_to_base_linkYaw_position_controller/command",100);
ros::Publisher pub_H_P_R = n.advertise<std_msgs::Float64>("/human_model/right_thigh_to_base_link_position_controller/command",100);
ros::Publisher pub_A_R_R = n.advertise<std_msgs::Float64>("/human_model/right_foot_to_right_ankle_position_controller/command",100);
ros::Publisher pub_A_P_R = n.advertise<std_msgs::Float64>("/human_model/right_leg_to_right_ankle_position_controller/command",100);

std_msgs::Float64 data;
data.data=0;

int joint_init=0;

while (ros::ok()) {
  if(joint_init<100){
  pubH.publish(data);
  pubArm_L.publish(data);
  pubFArm_L .publish(data);
  pubArm_R.publish(data);
  pubFArm_R .publish(data);
  pub_K_P_L.publish(data);
  pub_H_R_L.publish(data);
  pub_H_Y_L.publish(data);
  pub_H_P_L.publish(data);
  pub_A_R_L.publish(data);
  pub_A_P_L.publish(data);
  pub_K_P_R.publish(data);
  pub_H_R_R.publish(data);
  pub_H_Y_R.publish(data);
  pub_H_P_R.publish(data);
  pub_A_R_R.publish(data);
  pub_A_P_R.publish(data);
  joint_init++;


  }

  //update joint_state

//    joint_state.header.stamp = ros::Time::now();
//    joint_state.position[0] = 0;
//    joint_state.position[1] = 0;
//    joint_state.position[2] = 0;
//    joint_state.position[3] = 0;
//    joint_state.position[4] = 0;
//    joint_state.position[5] = 0;
//    joint_state.position[6] = 0;
//    joint_state.position[7] = 0;
//    joint_state.position[8] = 0;
//    joint_state.position[9] = 0;
//    joint_state.position[10] = 0;
//    joint_state.position[11] = 0;
//    joint_state.position[12] = 0;
//    joint_state.position[13] = 0;



    // update transform
    // (moving in a circle with radius=2)
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x =0.01*cos(rot4/5);
    odom_trans.transform.translation.y =0.01*sin(rot4/5);
    odom_trans.transform.translation.z = -0.2;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

    //send the joint state and transform
    joint_pub.publish(joint_state);
    broadcaster.sendTransform(odom_trans);

    rot4 += 1;
    if (rot4 > 90) rot4 = 0;

    loop_rate.sleep();
}
return 0;
}

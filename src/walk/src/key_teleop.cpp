#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include<termios.h>
#include <qdebug.h>

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "key_teleop");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<geometry_msgs::Twist>("/humanTwist", 1000);

  ros::Rate loop_rate(10);
  int num;
  geometry_msgs::Twist msg;

  while (ros::ok())
  {

    num=getch();
    qDebug()<<"\n"<<num;

    switch (num) {
    case 50:
      msg.linear.x=-1;
      break;
    case 52:
      msg.angular.z=-1;
      break;
    case 53:
      msg.angular.z=0;
      break;
    case 54:
      msg.angular.z=-1;
      break;
    case 56:
      msg.linear.x=1;
      break;
    default:
      msg.linear.x=0;
      msg.linear.y=0;
      break;
    }



    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

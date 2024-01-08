#include <ros/ros.h>
#include "calc.h"
#include <nav_msgs/Path.h>
#include <QFile>
#include <QTime>




void dataSave(const nav_msgs::Path& msg){
  qDebug()<<"Data recieved";
  QByteArray log_data;
  for (int i = 0; i < msg.poses.size(); ++i) {
    log_data.append(QString::number(msg.poses[i].pose.position.x)+","+
                    QString::number(msg.poses[i].pose.position.y)+","+
                    QString::number(msg.poses[i].pose.orientation.z)+","+
                    QString::number(msg.poses[i].pose.orientation.w)+"\n");

  }
  QTime pc_time;
QString name="Desktop/log/log"+QString::number(pc_time.currentTime().hour())+"_"+QString::number(pc_time.currentTime().minute())+"_"+QString::number(pc_time.currentTime().second())+".txt";
  QFile myfile(name);
  myfile.remove();
  myfile.open(QFile::ReadWrite);
  myfile.write(log_data);
  myfile.close();

qDebug()<<"Log saved to "<<name;


}

int main(int argc, char **argv)
{



  ros::init(argc, argv, "datalogger");
  ros::NodeHandle nh;
  ros::Rate r(200);
  ros::Subscriber pathplanning= nh.subscribe("/move_base/NavfnROS/plan", 100, dataSave);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();

  }



}





#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "geometry_msgs/Twist.h"
int main(int argc, char **argv)
{
ros::init(argc, argv, "motor");
ros::NodeHandle n;
ros::Publisher motor_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
ros::Rate loop_rate(10);
int count = 0;
  while (ros::ok())
  {
    geometry_msgs::Twist msg;
    msg.linear.x = 10;
    msg.angular.z = 5;
    motor_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}


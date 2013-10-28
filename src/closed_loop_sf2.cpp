/*************************************************************
Localization- Executable = Triangulation,
Take in the two center points and compute the distance to the cone
 ************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <rflex/atrvjr_driver.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>
#include <tf/transform_datatypes.h>


//#define DEBUG

using namespace std;


class ClosedLoop {
protected:
  
  ros::NodeHandle n_;

  ros::Subscriber heading_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher command_pub_;

  bool new_heading; 		//whether or not there's been a new command
  char heading_topic[100];
  char odom_topic[100];
  char command_topic[100]; 
  float dist_cmd;		//distance from current_heading
  float angle_cmd;		//angle from current_heading
  float odomX;			//x given by odometry
  float odomY;			//y given by odometry
  float odomA;			//angle given by odometry 
  float startX;			//starting point x coord when command is given
  float startY;			//starting point y coord when command is given
  float destX;			//destination x coord
  float destY;			//destination y coord
  int count;			//what number point on the reference path from start to finish we are trying to go to next
  float deltaDist;		//distance from one point to the next on the reference path
  float pathX;			//next point on reference path x coord
  float pathY;			//next piont on reference path y coord
  float pathAngle;		//angle reference path makes with positive x axis
  float distToRef;		//distance from current point to reference point on path
  float angleToRef;		//angle needed to turn for robot to be on the path from its position to the reference point
				//calculated as angle between heading of robot and the path from the postion of the robot to the reference point
  float travelPathAngle;	//angle of path (from position of robot to reference point) from x axis
  float k;	     		// "Spring" Constant (N/m)
  float B;	     		// "Damping" Constant
  float r;	     		// Radius of rotation of robot (m)
  float dt;	 		//time to step between iterations
  float m;    			// Mass of robot (kg)
  float I;    			// Moment of Inertia of robot
  float vMax;			//maximum translational velocity
  float tranVel;		//translational velocity
  float rotVel;			//angular velocity
  int left_time;		
  int right_time;



public:
  
  ClosedLoop(ros::NodeHandle &n, char **argv) :
    n_(n) {
    sprintf(heading_topic, "/current_heading");		//output topic
    sprintf(odom_topic, "/odom");
    sprintf(command_topic, "/cmd_vel");

   
    //Gets the most recently published image from the topic and activates the callback method
    heading_sub_ = n_.subscribe<geometry_msgs::Pose>(heading_topic, 1, &ClosedLoop::headingCallback, this);
    odom_sub_ = n_.subscribe<nav_msgs::Odometry>(odom_topic, 1, &ClosedLoop::odomCallback, this);
    command_pub_ = n_.advertise<geometry_msgs::Twist>(command_topic, 1);
    
    deltaDist = 1.0;
    k = 20;     		// "Spring" Constant (N/m)
    B = 5;     		// "Damping" Constant
    r = .5;     		// Radius of rotation of robot (m)
    dt = .1; 		//time to step between iterations
    m = 50;    		// Mass of robot (kg)
    I = 10;    		// Moment of Inertia of robot

  }

  ~ClosedLoop() {
	
  }
  void headingCallback(const geometry_msgs::PoseConstPtr& msg_ptr){
	dist_cmd = msg_ptr->position.x;
	angle_cmd = msg_ptr->orientation.z;
	new_heading = true;
        sendCommand();
  }
  void odomCallback(const nav_msgs::OdometryConstPtr& msg_ptr){
  	odomX = msg_ptr->pose.pose.position.x;
  	odomY = msg_ptr->pose.pose.position.y;
  	odomA = tf::getYaw(tf::Quaternion(
  	     	msg_ptr->pose.pose.orientation.x,
    	     	msg_ptr->pose.pose.orientation.y,
 		msg_ptr->pose.pose.orientation.z,
        	msg_ptr->pose.pose.orientation.w));	
  }
  float getPathX(float x1, float y1, float x2, float y2, int increment){
	float angleFromXAxis = atan((y2-y1)/(x2-x1));
	float newX = x1 + deltaDist*increment*cos(angleFromXAxis);
	return newX;;
  }
  float getPathY(float x1, float y1, float x2, float y2, int increment){
	float angleFromXAxis = atan((y2-y1)/(x2-x1));
	float newY = y1 + deltaDist*increment*sin(angleFromXAxis);
	return newY;;
  }
  float getDist(float x1, float y1, float x2, float y2){
	return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
  }
  float getAngle(float x1, float y1, float x2, float y2){
	return atan((y2-y1)/(x2-x1));
  }
	
  void sendCommand(){
	geometry_msgs::Twist msgOut;
        ros::Rate loop_rate(10);
	ros::spinOnce();
	//get new path if new command is given
	if (new_heading){
		startX = odomX;
		startY = odomY;
		destX = odomX + (dist_cmd*cos(angle_cmd));
		destY = odomY + (dist_cmd*sin(angle_cmd));
		ROS_INFO("%6.4f, %6.4f", destX, destY);
		count = 1;
		new_heading = false;	
	}
	
	while ((!new_heading)&&(getDist(odomX, odomY, destX, destY) > 0.5)){
		ros::spinOnce();
	//determine reference point on the given path
		pathX = getPathX(startX, startY, destX, destY, count);
		pathY = getPathY(startX, startY, destX, destY, count);
		pathAngle = getAngle(startX, startY, destX, destY);
		if ((count+1)*deltaDist < dist_cmd){
			count ++;
		}
	//determine robot's position and heading in relation to reference path
		distToRef = getDist(odomX, odomY, pathX, pathY);
		travelPathAngle = getAngle(odomX, odomY, pathX, pathY);
		angleToRef = travelPathAngle - odomA;
	//Compute the "forces" on the robot
		float FSpring = distToRef * k;
	        float FDamper = -B * (tranVel * cos(angleToRef) + rotVel *r *cos(travelPathAngle - angleToRef));
	//Compute the Torque and angular displacement 
		float torque = r * (FSpring + FDamper) * sin(angleToRef);
		rotVel = torque / I * dt ;
		tranVel = (FSpring + FDamper)*cos(angleToRef)/m*dt;
		if (rotVel > 0.7){
			rotVel = 0.7;
		}else if (rotVel < -0.7){
			rotVel = -0.7;
		}
		vMax = 1.0;
		if (tranVel >  vMax){ 
			tranVel =  vMax;
		}else if (tranVel < -vMax){ 
			tranVel = -vMax;
		}
		msgOut.linear.x = tranVel;
		msgOut.angular.z = rotVel;
        	command_pub_.publish(msgOut);
		loop_rate.sleep();
	}
	msgOut.linear.x = 0.0;
	msgOut.angular.z = 0.0;
	command_pub_.publish(msgOut);
  }

};



	//Make visible as a ROS node
int main(int argc, char** argv) {
  ros::init(argc, argv, "ClosedLoop");
  ros::NodeHandle n ("~");

  ClosedLoop node(n, argv);
  ros::spin();

  return 0;
}

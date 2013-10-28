#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <rflex/atrvjr_driver.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <math.h>

#define PI 3.1415926536

using namespace std;
//Global compass position

float compassAngle;



float odomX, odomY, odomA;
//Distance and heading to current destination
float heading_distance, heading_angle; 
bool updateHeading = true;

void headingCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  heading_distance = msg->position.x;
  heading_angle = msg->orientation.z;
  updateHeading = true;
} 

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  odomX = msg->pose.pose.position.x;
  odomY = msg->pose.pose.position.y;
  odomA = tf::getYaw(tf::Quaternion(
         msg->pose.pose.orientation.x,
         msg->pose.pose.orientation.y,
         msg->pose.pose.orientation.z,
         msg->pose.pose.orientation.w));
}


// Get the corresponding Y value for an x value on 
// a linear path given an initial x, y, and angle
float getY (float x, float a, float x0, float y0) {
  if (a != 0) {
    float m = -1/tan(a);
    return m * (x - x0) + y0;
  } else {
    return y0;
  }
}



 // Find the X coordinate of the intersection of two
// lines given two points and a two angles
float getIntersectX (float a1, float a2, float x1, float x2, float y1, float y2) {
  if (a1==0 || a1==PI || a1==-PI) {
    return x1;
  } else if (a2==0 || a2==PI || a2==-PI) {
    return x2;
  } else if (a1 != a2) {
    float m1 = -1/tan(a1);
    float m2 = -1/tan(a2);
    return (m1*x1 - m2*x2 + y2 - y1) / (m1 - m2);
  } else {
    return x1;
  }
}

// Find the angle between two points
float getSlope (float x1, float x2, float y1, float y2) {
  if (x2-x1 > .1)
    return -PI/2 + atan( (y2-y1) / (x2-x1) );
  else if (x1-x2 > .1)
    return PI/2 + atan( (y2-y1) / (x2-x1) );
  else if (y2 > y1)
    return 0;
  else
    return -PI;
}

// Find the distance between two points
float getDist (float x1, float x2, float y1, float y2) {
  return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "controller");

    ros::NodeHandle n;
    ros::Subscriber heading_sub = n.subscribe<geometry_msgs::Pose>("/current_heading",1,headingCallback);
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom",1,odomCallback);
    //ros::Subscriber compass_sub = n.subscribe("/android/imu",1,compassCallback);

    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
 

  /* ------------------------- *
   * INITIAL STATE INFORMATION *
   * ------------------------- */
  geometry_msgs::Twist msgOut;
    int hz = 10;
    ros::Rate loop_rate(hz);
  float theta = 0;  //Current Orientation measured from North (radians)
  float x = 0; //heading_distance * sin( heading_angle );        //Current X Position (m)
  float y = 0; //-heading_distance * cos( heading_angle );       //Current Y Position (m)
  float vTrans = 0;   // Forward (translational) velocity (m/s)
  float w = 0;         // Angular Speed (rad/s)
  float vRot;         // Linear Rotational speed (m/s)
  float thetaRot;    // Direction of vRot (radians)
  float r = .5;     // Radius of rotation of robot (m)
  float m = 50;    // Mass of robot (kg)f
  float I = 10;    // Moment of Inertia

  /* ----------------------- *
   * ENVIRONMENT INFORMATION *
   * ----------------------- */
  float k = 7;     // Spring Constant (N/m)
  float B = 9;     // Damping Constant
  float d;        // Distance from current destination (m)
  float dp;        // Distance from reference point
  float ds = 2;   // Distance ahead on path to ref point
  float dpAngle; // Slope of dp

  float deltaTheta;  // Difference between heading of robot and direction of dp
  float fT;         // Translational force

  float dt = .1;   //time to step each iteration (s)


   
    /* ------------------------------------ *
     * START OF CLOSED LOOP CONTROL PROGRAM *
     * ------------------------------------ */
	    float xPath, yPath;
      float xDest, yDest = 0;
      float pathSlope = 0;
		while (ros::ok()) {
      
  
	    ros::spinOnce();
      if (updateHeading) {
		  xDest = x - (heading_distance * sin( heading_angle ));        //Current X Position (m)
		  yDest = y + (heading_distance * cos( heading_angle ));       //Current Y Position (m)

			  // Compute the distance to the destination
			  d = heading_distance;
			  // Find the reference point
			  pathSlope = heading_angle;
		  } else {
        d = 0;
        msgOut.linear.x = 0;
        msgOut.angular.z = 0;
        chatter_pub.publish(msgOut);
      }
  	  //Go until the robot is within .1 meters of the destination
  	  int i = 0;

      updateHeading = false;
  	  while (d > .1 && !updateHeading) {
			 // Process a round of subscription messages
		      ros::spinOnce();


		    d = getDist(x,xDest,y,yDest);
		
		    xPath = getIntersectX(pathSlope, pathSlope+PI/2, xDest, x, yDest, y);
		    if (xPath > xDest)
		      xPath -= ds * abs(sin(pathSlope));
		    else
		      xPath += ds * abs(sin(pathSlope));
		    
		    if (pathSlope == 0 || pathSlope == PI || pathSlope == -PI) {
		      if (y < yDest) {
		        yPath = y + ds;
		      } else {
		        yPath = y - ds;
		      }
		    } else {
		      yPath = getY(xPath, pathSlope, 0, 0);
		    }      
		    dp = getDist(xPath,x,yPath,y);

		    dpAngle = getSlope(x,xPath,y,yPath);

		    //Compute Delta Theta
		    deltaTheta = dpAngle - theta;
		    if (deltaTheta >  PI) deltaTheta -= 2*PI;
		    if (deltaTheta < -PI) deltaTheta += 2*PI;

		    //Command Line Output
		    if (i%1 == 0 || d<1) {
		      cout << x <<" " << y << " "<< xDest << " " << yDest  << " "<< vTrans << " " << theta <<" "<<w<< " "<<d<<"\n";
		    }

		    //Compute the forces on the robot
		    float FSpring = dp * k;
	            float FDamper = -B * ( vTrans * cos( deltaTheta ) + vRot * cos(dpAngle - thetaRot) );

		    //Compute the Torque and angular displacement
		    float torque = r * (FSpring + FDamper) * sin( deltaTheta );
		    w = 2* torque / I * dt;
		    if (w >  .1) w =  .1;
                    if (w < -.1) w = -.1;
                    //theta += w;
                    theta = odomA;
		    if (theta >  PI) theta -= 2*PI;
		    if (theta < -PI) theta += 2*PI;
		    vRot = r * w;
		    if (vRot > 0)
		      thetaRot = theta + PI/2;
		    else
		      thetaRot = theta - PI/2;
		          

		    //Compute the translational force
		    fT = ( FSpring + FDamper ) * (cos( deltaTheta ));
		    vTrans += fT/m;
		    //Speed Limiter      
		    float vMax;      
		    if (d < 1) {
		      vMax = .5*(d);
		    } else {
		      vMax = .5;
		    }
		    if (vTrans >  vMax) vTrans =  vMax;
		    if (vTrans < -vMax) vTrans = -vMax;

		    //Update position
		    //x -= vTrans*dt*sin(theta);
		    //y += vTrans*dt*cos(theta);
		    
		   //NOTE:  The coordinate system used by Odometry is different than the one used in the program
	    //       0 Degrees aligns with North and the Y Axis, Counter-Clockwise is positive
   
			y = odomX;
		    	x = -odomY;



		    i++;
		    //if (i>2000) return 0;
	 
		         msgOut.linear.x = vTrans;
		         msgOut.angular.z = w/0.35;
		         chatter_pub.publish(msgOut);

		     
		  
		  //Stop when we exit the while loop
		  //       msgOut.linear.x = 0;
		   //      msgOut.angular.z = 0;
		   //      chatter_pub.publish(msgOut);

		// This will adjust as needed per iteration
		      loop_rate.sleep();
	}
}
	   msgOut.linear.x = 0;
           msgOut.angular.z = 0;
           chatter_pub.publish(msgOut);

  return 0;
}


#include <string>
#include <ros/ros.h>
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

//included to take in a Pose msg
#include <geometry_msgs/Pose.h>

//includes and using statement added to control motors
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
using namespace std;


//Input Message object
geometry_msgs::Pose msg;        
//Odometry input Message object
nav_msgs::Odometry msgOdom;
//Compass input message object
sensor_msgs::Imu msgCompass;
//Global x and y position values for odometry
float odomX, odomY;


#include <math.h>
#include <cmath>

#define PI 3.1415926536
//const float pi 3.1415926536;

/**
 *  \brief ATRV-JR Node for ROS
 *  By Mikhail Medvedev 02/2012
 *  Modified from B21 node originally written by David Lu
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
class ATRVJRNode {
    private:
        ATRVJR driver;

        ros::Subscriber subs[4];			///< Subscriber handles (cmd_vel, cmd_accel, cmd_sonar_power, cmd_brake_power)
        ros::Publisher base_sonar_pub;		///< Sonar Publisher for Base Sonars (sonar_cloud_base)
        ros::Publisher body_sonar_pub;		///< Sonar Publisher for Body Sonars (sonar_cloud_body)
        ros::Publisher voltage_pub;			///< Voltage Publisher (voltage)
        ros::Publisher brake_power_pub;		///< Brake Power Publisher (brake_power)
        ros::Publisher sonar_power_pub;		///< Sonar Power Publisher (sonar_power)
        ros::Publisher odom_pub;			///< Odometry Publisher (odom)
        ros::Publisher plugged_pub;			///< Plugged In Publisher (plugged_in)
        ros::Publisher joint_pub; ///< Joint State Publisher (state)
        ros::Publisher bump_pub; ///< Bump Publisher (bumps)
        tf::TransformBroadcaster broadcaster; ///< Transform Broadcaster (for odom)

        bool isSonarOn, isBrakeOn;
        float acceleration;
        float last_distance, last_bearing;
        float x_odo, y_odo, a_odo;
        float cmdTranslation, cmdRotation;
        bool brake_dirty, sonar_dirty;
        bool initialized;
        float first_bearing;
        int updateTimer;
        int prev_bumps;
        bool sonar_just_on;

        void publishOdometry();
        void publishSonar();
        void publishBumps();

   


    public:
        ros::NodeHandle n;
        ATRVJRNode();
        ~ATRVJRNode();
        int initialize(const char* port);
        void spinOnce();

        // Message Listeners
        void NewCommand      (const geometry_msgs::Twist::ConstPtr& msg);
        void SetAcceleration (const std_msgs::Float32   ::ConstPtr& msg);
        void ToggleSonarPower(const std_msgs::Bool      ::ConstPtr& msg);
        void ToggleBrakePower(const std_msgs::Bool      ::ConstPtr& msg);

        float getY (float x, float a, float x0, float y0);
        float getIntersectX (float a1, float a2, float x1, float x2, float y1, float y2);
        float getSlope (float x1, float x2, float y1, float y2);
        float getDist (float x1, float x2, float y1, float y2);
        float getOdomAngle();
        float setOdomToCompass();

        /*void headingCallback(const geometry_msgs::Pose msgIn);
        void odomCallback(const nav_msgs::Odometry msgIn);
        void compassCallback(const sensor_msgs::Imu msgIn);*/
};

ATRVJRNode::ATRVJRNode() : n ("~") {
    isSonarOn = isBrakeOn = false;
    brake_dirty = sonar_dirty = false;
    sonar_just_on = false;
    cmdTranslation = cmdRotation = 0.0;
    updateTimer = 99;
    initialized = false;
    prev_bumps = 0;
    subs[0] = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1,   &ATRVJRNode::NewCommand, this);
    subs[1] = n.subscribe<std_msgs::Float32>("cmd_accel", 1,     &ATRVJRNode::SetAcceleration, this);
    subs[2] = n.subscribe<std_msgs::Bool>("cmd_sonar_power", 1, &ATRVJRNode::ToggleSonarPower, this);
    subs[3] = n.subscribe<std_msgs::Bool>("cmd_brake_power", 1, &ATRVJRNode::ToggleBrakePower, this);
    acceleration = 0.7;

    base_sonar_pub = n.advertise<sensor_msgs::PointCloud>("sonar_cloud_base", 50);
    body_sonar_pub = n.advertise<sensor_msgs::PointCloud>("sonar_cloud_body", 50);
    sonar_power_pub = n.advertise<std_msgs::Bool>("sonar_power", 1);
    brake_power_pub = n.advertise<std_msgs::Bool>("brake_power", 1);
    voltage_pub = n.advertise<std_msgs::Float32>("voltage", 1);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    plugged_pub = n.advertise<std_msgs::Bool>("plugged_in", 1);
    joint_pub = n.advertise<sensor_msgs::JointState>("state", 1);
    bump_pub = n.advertise<sensor_msgs::PointCloud>("bump", 5);
}

int ATRVJRNode::initialize(const char* port) {
    int ret = driver.initialize(port);
    if (ret < 0)
        return ret;

    driver.setOdometryPeriod (100000);
    driver.setDigitalIoPeriod(100000);
    driver.motionSetDefaults();
    return 0;
}

ATRVJRNode::~ATRVJRNode() {
    driver.motionSetDefaults();
    driver.setOdometryPeriod(0);
    driver.setDigitalIoPeriod(0);
    driver.setSonarPower(false);
    driver.setIrPower(false);
}

/// cmd_vel callback
void ATRVJRNode::NewCommand(const geometry_msgs::Twist::ConstPtr& msg) {
    cmdTranslation = msg->linear.x;
    cmdRotation = msg->angular.z;
}

/// cmd_acceleration callback
void ATRVJRNode::SetAcceleration (const std_msgs::Float32::ConstPtr& msg) {
    acceleration = msg->data;
}

/// cmd_sonar_power callback
void ATRVJRNode::ToggleSonarPower(const std_msgs::Bool::ConstPtr& msg) {
    isSonarOn=msg->data;
    sonar_dirty = true;
}

/// cmd_brake_power callback
void ATRVJRNode::ToggleBrakePower(const std_msgs::Bool::ConstPtr& msg) {
    isBrakeOn = msg->data;
    brake_dirty = true;
}

void ATRVJRNode::spinOnce() {
    // Sending the status command too often overwhelms the driver
    if (updateTimer>=100) {
        driver.sendSystemStatusCommand();
        updateTimer = 0;
    }
    updateTimer++;

    if (cmdTranslation != 0 || cmdRotation != 0)
        driver.setMovement(cmdTranslation, cmdRotation, acceleration);

    if (sonar_dirty) {
        driver.setSonarPower(isSonarOn);
        sonar_dirty = false;
        driver.sendSystemStatusCommand();
    }
    if (brake_dirty) {
        driver.setBrakePower(isBrakeOn);
        brake_dirty = false;
        updateTimer = 99;
    }

    std_msgs::Bool bmsg;
    bmsg.data = isSonarOn;
    sonar_power_pub.publish(bmsg);
    bmsg.data = driver.getBrakePower();
    brake_power_pub.publish(bmsg);
    bmsg.data = driver.isPluggedIn();
    plugged_pub.publish(bmsg);
    std_msgs::Float32 vmsg;
    vmsg.data = driver.getVoltage();
    voltage_pub.publish(vmsg);

    publishOdometry();
    publishSonar();
    publishBumps();
}

/** Integrates over the lastest raw odometry readings from
 * the driver to get x, y and theta */
void ATRVJRNode::publishOdometry() {
    if (!driver.isOdomReady()) {
        return;
    }

    float distance = driver.getDistance();
    float true_bearing = angles::normalize_angle(driver.getBearing());

    if (!initialized) {
        initialized = true;
        first_bearing = true_bearing;
	last_bearing = 0;
        x_odo = 0;
        y_odo = 0;
        //a_odo = 0*true_bearing;
    } else {
        float bearing = true_bearing - first_bearing;
        float d_dist = distance-last_distance;
        float d_bearing = bearing - last_bearing;

        if (d_dist > 50 || d_dist < -50)
            return;
//cout << a_odo<<"\n";
        a_odo += d_bearing;
//cout << a_odo<<"\n";
        a_odo = angles::normalize_angle(a_odo);
//cout << a_odo<<"\n";

        //integrate latest motion into odometry
        x_odo += d_dist * cos(a_odo);
        y_odo += d_dist * sin(a_odo);
    }
    last_distance = distance;
    last_bearing = true_bearing - first_bearing;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(last_bearing);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x_odo;
    odom_trans.transform.translation.y = y_odo;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x_odo;
    odom.pose.pose.position.y = y_odo;
    odom.pose.pose.orientation = odom_quat;

    //update odom variables
    odomX = x_odo;
    odomY = y_odo;

    //set the velocity
    odom.child_frame_id = "base_link";
    float tvel = driver.getTranslationalVelocity();
    odom.twist.twist.linear.x = tvel*cos(a_odo);
    odom.twist.twist.linear.y = tvel*sin(a_odo);
    odom.twist.twist.angular.z = driver.getRotationalVelocity();

    //publish the message
    odom_pub.publish(odom);

    // finally, publish the joint state
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.set_name_size(1);
    joint_state.set_position_size(1);
    joint_state.name[0] = "lewis_twist";
    joint_state.position[0] = true_bearing;

    joint_pub.publish(joint_state);

}

void ATRVJRNode::publishSonar() {
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "base_link";

    if (isSonarOn) {
        driver.getBaseSonarPoints(&cloud);
        base_sonar_pub.publish(cloud);

        driver.getBodySonarPoints(&cloud);
        cloud.header.frame_id = "body";
        body_sonar_pub.publish(cloud);

    } else if (sonar_just_on) {
        base_sonar_pub.publish(cloud);
        cloud.header.frame_id = "body";
        body_sonar_pub.publish(cloud);
    }
}

void ATRVJRNode::publishBumps() {
    sensor_msgs::PointCloud cloud1, cloud2;
    cloud1.header.stamp = ros::Time::now();
    cloud2.header.stamp = ros::Time::now();
    cloud1.header.frame_id = "base_link";
    cloud2.header.frame_id = "body";
    int bumps = driver.getBaseBumps(&cloud1) +
                driver.getBodyBumps(&cloud2);

    if (bumps>0 || prev_bumps>0) {
        bump_pub.publish(cloud1);
        bump_pub.publish(cloud2);
    }
    prev_bumps = bumps;
}

void headingCallback(const geometry_msgs::Pose msgIn) {
  msg=msgIn;
} 

void odomCallback(const nav_msgs::Odometry msgIn) {
  msgOdom=msgIn;
}

void compassCallback(const sensor_msgs::Imu msgIn) {
  msgCompass=msgIn;
}


 // Get the corresponding Y value for an x value on 
// a linear path given an initial x, y, and angle
float ATRVJRNode::getY (float x, float a, float x0, float y0) {
  if (a != 0) {
    float m = -1/tan(a);
    return m * (x - x0) + y0;
  } else {
    return y0;
  }
}

 // Find the X coordinate of the intersection of two
// lines given two points and a two angles
float ATRVJRNode::getIntersectX (float a1, float a2, float x1, float x2, float y1, float y2) {
  if (a1==0 || a1==2*PI || a1==PI) {
    return x1;
  } else if (a2==0 || a2==2*PI || a2==PI) {
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
float ATRVJRNode::getSlope (float x1, float x2, float y1, float y2) {
  if (x2 > x1)
    return -PI/2 + atan( (y2-y1) / (x2-x1) );
  else if (x2 < x1)
    return PI/2 + atan( (y2-y1) / (x2-x1) );
  else if (y2 > y1)
    return 0;
  else
    return -PI;
}

// Find the distance between two points
float ATRVJRNode::getDist (float x1, float x2, float y1, float y2) {
  return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

float ATRVJRNode::getOdomAngle() {
//w orientaion of the odometry data
  float w = msgOdom.pose.pose.orientation.w;
  float z = msgOdom.pose.pose.orientation.z;
  float angle = 2*acos(w);
  if (w == 1 || z == 0)
    return angle;
  else
    return angle * z / sin( angle / 2 );
}

float ATRVJRNode::setOdomToCompass() {
    //Get the Yaw component of compass data as an angle
    /*float theta = tf::getYaw(tf::Quaternion(
         msgCompass.orientation.x,
         msgCompass.orientation.y,
         msgCompass.orientation.z,
         msgCompass.orientation.w));*/
    float w = msgCompass.orientation.w;
    float z = msgCompass.orientation.z;
    float theta = 2*acos(w);
      if (!(w == 1 || z == 0))
        theta *= z / sin( theta / 2 );
    //Set odometer angle to compass angle    
	cout << theta <<", "<<a_odo<<"\n";
    a_odo = theta;
    return theta;
}
 

int main(int argc, char** argv) {
  /* ------------------------- *
   * INITIAL STATE INFORMATION *
   * ------------------------- */

  float theta;  //Current Orientation measured from North (radians)
  float x=0;        //Current X Position (m)
  float y=0;       //Current Y Position (m)
  float vTrans = 0;   // Forward (translational) velocity (m/s)
  float w = 0;         // Angular Speed (rad/s)
  float vRot;         // Linear Rotational speed (m/s)
  float thetaRot;    // Direction of vRot (radians)
  float r = .5;     // Radius of rotation of robot (m)
  float m = 50;    // Mass of robot (kg)
  float I = 10;    // Moment of Inertia

  /* ----------------------- *
   * ENVIRONMENT INFORMATION *
   * ----------------------- */
  float k = 3;     // Spring Constant (N/m)
  float B = 9;     // Damping Constant
  float d;        // Distance from current destination (m)
  float dp;        // Distance from reference point
  float ds = 2;   // Distance ahead on path to ref point
  float dpAngle; // Slope of dp

  float deltaTheta;  // Diference between heading of robot and direction of dp
  float fT;         // Translational force

  float dt = .1;   //time to step each iteration (s)


    ros::init(argc, argv, "atrvjr");

    ros::NodeHandle n;

    ros::Subscriber heading_sub = n.subscribe("current_heading",1,headingCallback);
    ros::Subscriber odom_sub = n.subscribe("odom",1,odomCallback);
    ros::Subscriber compass_sub = n.subscribe("android/imu",1,compassCallback);
    
    //FOR TESTING PURPOSES ONLY
    float xDest, yDest;

    ATRVJRNode node;
    std::string port;
    node.n.param<std::string>("port", port, "/dev/ttyUSB0");
    ROS_INFO("Attempting to connect to %s", port.c_str());
    if (node.initialize(port.c_str())<0) {
        ROS_ERROR("Could not initialize RFLEX driver!\n");
        return 0;
    }
    ROS_INFO("Connected!");

    //added from motor.cpp This line should make this into a publisher that publishes geomety_msgs
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    int hz;
    node.n.param("rate", hz, 10);
    ros::Rate loop_rate(hz); 

    //FOR TESTING PURPOSES ONLY
    cout << "Please enter x"<<endl;
    cin >> xDest;
    cout << "Please enter a y"<<endl;
    cin >> yDest;


    //Message object
    geometry_msgs::Twist msgOut;

    //print out the inputed variables, JUST FOR TESTING IF THE OTHER PROGRAM IS COMMUNICATING
//    cout << " position.x: " << msg.position.x << endl;
//    cout << " orientation.z: " << msg.orientation.z <<endl;

    //cout<<"Odometry x: " <<msgOdom.pose.pose.position.x <<endl;

    //GET GPS/COMPASS DATA
    //float GPS_DIST = msg.position.x;

    //TO USE COMPASS UNCOMMENT THE FOLLOWING LINE
    //theta = node.setOdomToCompass();

    //AND COMMENT THIS OUT
    theta = 0;

//---------------------------------------------------------------------------

    /* ------------------------------------ *
     * START OF CLOSED LOOP CONTROL PROGRAM *
     * ------------------------------------ */

    float xPath, yPath;
    // Compute the distance to the destination
    d = node.getDist(x,xDest,y,yDest);

    // Find the reference point
    float pathSlope = node.getSlope(0, xDest, 0, yDest);
    
    //Go until the robot is within 1 meters of the destination
    int i = 0;
    while (d > 1 && ros::ok()) {
	 // Process a round of subscription messages
        ros::spinOnce();
        node.spinOnce();


      d = node.getDist(x,xDest,y,yDest);
  
      xPath = node.getIntersectX(pathSlope, pathSlope+PI/2, 0, x, 0, y);
      if (xPath > xDest)
        xPath -= ds * abs(sin(pathSlope));
      else
        xPath += ds * abs(sin(pathSlope));
      
      if (pathSlope == 0 || pathSlope == PI || pathSlope == -PI) {
        if (yDest > y) {
          yPath = y + ds;
        } else {
          yPath = y - ds;
        }
      } else {
        yPath = node.getY(xPath, pathSlope, 0, 0);
      }      
      dp = node.getDist(xPath,x,yPath,y);

      dpAngle = node.getSlope(x,xPath,y,yPath);

      //Compute Delta Theta
      deltaTheta = dpAngle - theta;
      if (deltaTheta >  PI) deltaTheta -= 2*PI;
      if (deltaTheta < -PI) deltaTheta += 2*PI;

      //Command Line Output
      if (i%1 == 0 || d<1) {
        cout << x << " " << y << " "<< xPath << " " << yPath  << " "<< vTrans << " " << theta <<" "<<w<< " "<<d<<"\n";
      }

      //Compute the forces on the robot
      float FSpring = dp * k;
      float FDamper = -B * ( vTrans * cos( deltaTheta ) + vRot * cos(dpAngle - thetaRot) );

      //Compute the Torque and angular displacement
      float torque = r * (FSpring + FDamper) * sin( deltaTheta );
      w = torque / I * dt;
      //if (w >  .1) w =  .1;
      //if (w < -.1) w = -.1;
      //theta += w;
      theta = node.getOdomAngle();
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
      if (d < 2.9) {
        vMax = .25*(d-.9);
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
      y = msgOdom.pose.pose.position.x;
      x = -msgOdom.pose.pose.position.y;
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
	   msgOut.linear.x = 0;
           msgOut.angular.z = 0;
           chatter_pub.publish(msgOut);

ros::spinOnce();
        node.spinOnce();
loop_rate.sleep();

    return 0;
}

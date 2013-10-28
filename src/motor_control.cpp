/*
 *  Motor Controller using keyboard inputs
 *  Keval Patel - September 26, 2013
 *
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *
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


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
using namespace std;

/**
 * This file is just a simple motor control command file
 * Usage: The keys W,S,A,D serve as input keys to move the rover
 * Forward, Backward, Left and Right respectively.
 *   *"W"*
 *  *A*S*D*
 *  Thus:-
 *  W = FORWARD
 *  S = BACKWARD
 *  A = LEFT
 *  D = RIGHT
 */
#define MOVE_FORWARD 			keyboardInput == 'w'
#define MOVE_BACKWARD 			keyboardInput == 's'
#define TURN_LEFT			keyboardInput == 'a'
#define TURN_RIGHT			keyboardInput == 'd'

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_control");
  // Variable in which the keyboard input character is stored
  char keyboardInput;
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

/**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher command_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
	// As user to provide with the input
    cout << "Please enter a the direction you want to move \nForward- W \nBackward- S\nLeft - A\nRight- D\nStop-H\n";
    cin >> keyboardInput;

   geometry_msgs::Twist velocityMessage;

    /* Check to see which command to execute, then input numbers accordingly
    * We care about linear.x, positive- forward negative- backward
    * We care about angular.z, positive- left negative-right
	*/

    if(MOVE_FORWARD)
    {
      velocityMessage.linear.x = 0.2;
      velocityMessage.angular.z = 0;
    }
    else if(MOVE_BACKWARD)
    {
      velocityMessage.linear.x = -0.2;
      velocityMessage.angular.z = 0;
    }
    else if(TURN_LEFT)
    {
      velocityMessage.linear.x = 0;
      velocityMessage.angular.z = 0.2;
    }
    else if(TURN_RIGHT)
    {
      velocityMessage.linear.x = 0;
      velocityMessage.angular.z = -0.2;
    }
    //if we receive an H or any non-valid letter we stop the rover
    else
     {
       velocityMessage.linear.x = 0;
       velocityMessage.angular.z = 0;
     }

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */

    command_pub.publish(velocityMessage);
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}


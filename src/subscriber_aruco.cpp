/*
 * subscriber_aruco.cpp
 *
 *  Created on: 4 Jun 2013
 *      Author: Miguel A. Olivares-Mendez
 *      SnT - University of Luxembourg
 *      Automation Research Group
 *
 *      This code is for get the information of the ArUco publisher
 *
 *		This is a version of ArUco software for ROS
 *		The ArUco software is a C++ library developed by Rafael Muñoz Salinas (2011).
 *		http://sourceforge.net/projects/aruco/files/1.2.4/
 *		This library let you create augmented reality codes (and board codes) and detect
 *		them using a monocular camera.
 *
 *		An application of this AR code detection for control an UAV for See and Avoid task
 *		has done by Miguel A. Olivares-Mendez
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "aruco/marker_info.h"
#include <cstdlib>
float alpha, beta, heading, azimuth;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const aruco::marker_info& msg)
{
  ROS_INFO("I heard: [#%d]. ", msg.numberOfMarkers);
  for (int i=0; i<msg.numberOfMarkers; ++i){
	  alpha = atan(msg.trsX[i]/ msg.distance[i])*(180/3.1416);
	  beta = atan(msg.trsY[i]/ msg.distance[i])*(180/3.1416);
	  heading = msg.rotY[i]*(180/3.1416);
	  azimuth = msg.rotX[i]*(180/3.1416)+90.0;
	  ROS_INFO("alpha: %f, beta: %f, heading: %f, azimuth: %f",alpha-heading,beta-azimuth,heading,azimuth);
	//  ROS_INFO("Id: %d, Area: %f, Perimeter: %f, Center.X: %f, Center.Y: %f, RotX: %f, RotY: %f, RotZ: %f, Distance: %f, Xtras: %f, Ytras: %f", msg.id[i], msg.area[i], msg.perimeter[i], msg.centerX[i], msg.centerY[i], msg.rotX[i], msg.rotY[i], msg.rotZ[i], msg.distance[i], msg.trsX[i], msg.trsY[i]);
	  ROS_INFO("Id: %d, Center.X: %f, Center.Y: %f, RotX: %f, RotY: %f, RotZ: %f, Distance: %f, Xtras: %f, Ytras: %f", msg.id[i], msg.centerX[i], msg.centerY[i], msg.rotX[i], msg.rotY[i], msg.rotZ[i], msg.distance[i], msg.trsX[i], msg.trsY[i]);

  }
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("marker_info", 1000, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}

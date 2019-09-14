#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <cstdio>
#include "mars_controller/qnode.h"

#include <QDebug>

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

void QNode::robotposCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	// checking frame id format
	if (msg->header.frame_id.find("youbot") != 0)
		return;
	// get robot number by "youbotN/base_footprint" frame id
	int robot_number = std::stoi(
		msg->header.frame_id.substr(6, msg->header.frame_id.find("/"))
	);
	robot_number--;
	
	// saving
	this->odom_msgs[robot_number] = *msg;
}

bool QNode::init(int robots_num) {
	ros::init(init_argc,init_argv,"mars_controller_node");
	if ( ! ros::master::check() ) {
		return false;
	}

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	
	// setting size (WTF?)
	nav_msgs::Odometry odom_null;

	for(int i=0;i < robots_num;i++) {
		// increase odom_msgs object size
		odom_msgs.append(odom_null);

		std::string robot_name = "youbot" + std::to_string(i+1);

		// Init ROS velocity commands publishers
		cmd_vel_pubs.append(
			n.advertise<geometry_msgs::Twist>(
				robot_name + "/cmd_vel",
				1000
			)
		);

		// Init ROS subscriber for current robots positions
		odom_subs.append(
			n.subscribe(
				robot_name + "/odom_abs",
				1,
				&QNode::robotposCallback, this
			)
		);
	}

	start();
	return true;
}

void QNode::sendGoal(int robot_id, double vx, double w, QPointF goal_pos, long int rel_time) {
	
	// Target velocity vector
	geometry_msgs::Twist cmd_vel_msg;
	cmd_vel_msg.linear.x = vx;
	cmd_vel_msg.angular.z = w;

	// Current velocity vector
	/*this->odom_msgs.at(robot_id).twist.twist.linear.x;
	this->odom_msgs.at(robot_id).twist.twist.linear.y; // this may be non-zero (this expected)
	this->odom_msgs.at(robot_id).twist.twist.angular.z;

	// Target position (without orientation)
	goal_pos.x();
	goal_pos.y();

	// Current position (without orientation)
	this->odom_msgs.at(robot_id).pose.pose.position.x;
	this->odom_msgs.at(robot_id).pose.pose.position.y;*/


	/**
	 * How calculate velocity commad on robot? (by this data)
	 * */


	cmd_vel_pubs[robot_id].publish(cmd_vel_msg);
}

void QNode::setRealGroupPos(GroupPos *gpos) {
	if (this->odom_msgs.isEmpty()) return;
	
	int robots_num = gpos->robots_pos.size();

	for (int robot_id=0;robot_id < robots_num;robot_id++) {
		// vels
		gpos->robots_pos[robot_id].vel_real.x = this->odom_msgs.at(robot_id).twist.twist.linear.x;
		gpos->robots_pos[robot_id].vel_real.w = this->odom_msgs.at(robot_id).twist.twist.angular.z;
		// poses (with convert from m to cm and revert y axis)
		gpos->robots_pos[robot_id].pos_real.x = 100*(this->odom_msgs.at(robot_id).pose.pose.position.x);
		gpos->robots_pos[robot_id].pos_real.y = -100*(this->odom_msgs.at(robot_id).pose.pose.position.y);
	}
}

void QNode::run() {
	ros::Rate loop_rate(50);

	while ( ros::ok() ) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	emit rosShutdown(); // used to signal for a shutdown (useful to roslaunch)
}


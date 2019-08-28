#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/Float64.h>
#include <sstream>
#include "mars_controller/qnode.h"

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

bool QNode::init() {
	ros::init(init_argc,init_argv,"mars_controller_node");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	
	// Add your ros communications here.
	cmd_vel_pub = n.advertise<std_msgs::Float64>("cmd_vel", 1000);
	
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"mars_controller_node");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	
	// Add your ros communications here.
	
	start();
	return true;
}

void QNode::sendGoal(double vx, double w, QPointF goal_pos) {
	std_msgs::Float64 cmd_vel_msg;
	cmd_vel_msg.data = vx;
	cmd_vel_pub.publish(cmd_vel_msg);
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {

		// Do something with ros

		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


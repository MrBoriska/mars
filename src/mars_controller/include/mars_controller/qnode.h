#ifndef QNODE_H_
#define QNODE_H_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QVector3D>
#include <QList>
#include <nav_msgs/Odometry.h>

#include "modelconfig.h"


/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv);
	virtual ~QNode();

	// Init ROS node and loop
	bool init(int robots_num);
	
	// Built-in method whitch calling in new thread from start() function (into init() function)
	void run();

	// Send command to move goal
	void sendGoal(int robot_id, double vx, double w, QVector3D goal_pos, long int rel_time);

	// Set to gpos real robots position and velocity (with convert dimensions)
	void setRealGroupPos(GroupPos *gpos);
	
	// Callback function by robot position and velocity
	void robotposCallback(const nav_msgs::Odometry::ConstPtr& msg);

Q_SIGNALS:
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;

	// ROS objects for communication
	QList<ros::Publisher> cmd_vel_pubs;
	QList<ros::Subscriber> odom_subs;

	// Current robots position and velocity
	QList<nav_msgs::Odometry> odom_msgs;

	// accumulating coefficients for PID regulator 
	double ey_p;
	double ey_pp;
	double U_p;

};

#endif /* QNODE_HPP_ */

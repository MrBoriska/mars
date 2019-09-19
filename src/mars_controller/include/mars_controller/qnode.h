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
	// calling in new thread from start() function (into init() function)
	void run();

	// sender command to move goal
	void sendGoal(int robot_id, double vx, double w, QVector3D goal_pos, long int rel_time);

	void setRealGroupPos(GroupPos *gpos);
	// Callbacks
	void robotposCallback(const nav_msgs::Odometry::ConstPtr& msg);

Q_SIGNALS:
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	QList<ros::Publisher> cmd_vel_pubs;
	QList<ros::Subscriber> odom_subs;
	QList<nav_msgs::Odometry> odom_msgs;

	double ey_p;
	double ey_pp;
	double U_p;

};

#endif /* QNODE_HPP_ */

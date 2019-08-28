#ifndef QNODE_H_
#define QNODE_H_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QPointF>


/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
	void sendGoal(double vx, double w, QPointF goal_pos);

Q_SIGNALS:
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher cmd_vel_pub;
};

#endif /* QNODE_HPP_ */

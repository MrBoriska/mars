#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <cstdio>
#include "mars_controller/qnode.h"

#include <QDebug>

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{
		ros::init(init_argc,init_argv,"mars_controller_node");
		if ( ! ros::master::check() ) {
			ROS_ERROR("Do not start node. ROS Master process not found!");
			return;
		}

		ros::start();
		nh = new ros::NodeHandle();

		start();
	}

QNode::~QNode() {
    if(ros::isStarted()) {
		ros::shutdown();

		qDebug() << "~QNode() Wait for shutdown ROS!";
		ros::waitForShutdown();
    }
	qDebug() << "QNode::~QNode() (ros not start)";
	wait();
}
/**
 * @breif QNode::robotposCallback
 * 		  Принимает данные о реальном состоянии системы (положение и скорость)
 * @param msg
 */
void QNode::robotposCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	// checking frame id format
	if (msg->header.frame_id.find("youbot") != 0)
		return;

	// get robot number by "youbot{N}/****" frame id
	int robot_number = std::stoi(
		msg->header.frame_id.substr(6, msg->header.frame_id.find("/"))
	);
	robot_number--;
	
	// saving
	this->odom_msgs[robot_number] = *msg;
}

/**
 * @breif QNode::init
 * 		  Осуществляет инициализацию коммуникации ноды с топиками ROS
 */
void QNode::init() {
	int robots_num = ModelConfig::Instance()->getRobotsNum();
	
	// setting size
	nav_msgs::Odometry odom_null;
	ey_p = 0.0;
	ey_pp = 0.0;
	U_p = 0.0;
	
	cmd_vel_pubs.clear();
	odom_subs.clear();
	odom_msgs.clear();

	for(int i=0;i < robots_num;i++) {
		// increase odom_msgs object size
		odom_msgs.append(odom_null);

		std::string robot_name = "youbot" + std::to_string(i+1);

		// Init ROS velocity commands publishers
		cmd_vel_pubs.append(
			nh->advertise<geometry_msgs::Twist>(
				robot_name + "/cmd_vel",
				1000
			)
		);

		// Init ROS subscriber for current robots position
		odom_subs.append(
			nh->subscribe(
				robot_name + "/odom_abs",
				1,
				&QNode::robotposCallback, this
			)
		);
	}
}

/**
 * @brief QNode::sendGoal
 *        Функция для генерации управляющего воздействия на роботы
 *        с учетом обратной связи по положению и эталонного вектора управления
 * @param robot_id - 
 * @param vx - целевая линейная скорость
 * @param w - целевая угловая скорость
 * @param goal_pos - целевое положение в текущий момент времени
 * @param rel_time - текущее время (с)
 */ 
void QNode::sendGoal(int robot_id, double vx, double w, QVector3D goal_pos, long int rel_time) {
	
	// Target velocity vector
	geometry_msgs::Twist cmd_vel_msg;
	cmd_vel_msg.linear.x = vx;
	cmd_vel_msg.angular.z = w;

	if (vx == 0 && rel_time < 100) {
		ey_pp = 0.0;
		ey_p = 0.0;
		U_p = 0.0;
	}

	ModelConfig* config = ModelConfig::Instance();	

	if (vx != 0 || w != 0) {
		// Current velocity vector (in global basis)
		double cvx_x = this->odom_msgs.at(robot_id).twist.twist.linear.x;
		double cvx_y = this->odom_msgs.at(robot_id).twist.twist.linear.y;
		double cw = this->odom_msgs.at(robot_id).twist.twist.angular.z;

		// convert to robot basis (get "x" component, where "y" is zero)
		double cvx = sqrt(cvx_x*cvx_x + cvx_y*cvx_y);

		if (cvx == 0) {
			cmd_vel_pubs[robot_id].publish(cmd_vel_msg);
			return;
		}

		// Target position (with orientation)
		double gx = goal_pos.x();
		double gy = goal_pos.y();

		// Current position (with orientation)
		double cx = 100.0*(this->odom_msgs.at(robot_id).pose.pose.position.x);
		double cy = -100.0*(this->odom_msgs.at(robot_id).pose.pose.position.y);

		double ccos = cvx_x/cvx;
		double csin = cvx_y/cvx;

		//Calculate error vector
		double ex = (gx-cx)*ccos-(gy-cy)*csin;
		double ey = (gx-cx)*csin+(gy-cy)*ccos;
		ey = -ey; //revert for y axis

		// set error treshold
		ex /= 100.0;
		ey /= 100.0;

		// Regulator settings
		double T = double(config->interval)/1000.0;
		double Kpvx = config->trajectory_v_P;
		double Kp = config->trajectory_w_P;
		double Ki = config->trajectory_w_I;
		double Kd = config->trajectory_w_D;
		double KI = Kp*Ki*T;
		double KD = (Kp*Kd)/T;

		// Simple P regulator
		cmd_vel_msg.linear.x += Kpvx*(ex - 0.15);

		// Requrrent PID regulator
		double U = U_p + Kp*(ey-ey_p) + KI*ey + KD*(ey-2*ey_p+ey_pp);
		ey_pp = ey_p;
		ey_p = ey;
		U_p = U;

		// Small timeout for stability
		if (rel_time > 1000) cmd_vel_msg.angular.z = U;

		// set absolute restrictions
		if (cmd_vel_msg.linear.x < 0)
			cmd_vel_msg.linear.x = 0.0;

		// set thresholds for lenear velocity
		double thres = config->trajectory_v_thres;
		if (cmd_vel_msg.linear.x > vx*(thres+1))
			cmd_vel_msg.linear.x = vx*(thres+1);
		if (cmd_vel_msg.linear.x < vx*(1.0-thres))
			cmd_vel_msg.linear.x = vx*(1.0-thres);
		
		// set thresholds for angular velocity
		double thres_if_w_0 = 3.0*cmd_vel_msg.linear.x;
		if (cmd_vel_msg.angular.z > thres_if_w_0)
			cmd_vel_msg.angular.z = thres_if_w_0;
		if (cmd_vel_msg.angular.z < -thres_if_w_0)
			cmd_vel_msg.angular.z = -thres_if_w_0;
	}

	cmd_vel_pubs[robot_id].publish(cmd_vel_msg);
}

void QNode::setRealGroupPos(GroupPos *gpos) {
	if (this->odom_msgs.isEmpty()) return;
	
	int robots_num = gpos->robots_pos.size();

	for (int robot_id=0;robot_id < robots_num;robot_id++) {
		// vels
		double cvx_x = this->odom_msgs.at(robot_id).twist.twist.linear.x;
		double cvx_y = this->odom_msgs.at(robot_id).twist.twist.linear.y;
		
		gpos->robots_pos[robot_id].vel_real.x = sqrt(cvx_x*cvx_x + cvx_y*cvx_y);
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
	
	qDebug() << "Shutdown by ROS thread";
	emit rosShutdown();
}


#include <QApplication>

//#include <ros/ros.h>
//#include <std_msgs/Float64.h>

#include "mars_controller/mainlogic.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainLogic *model = new MainLogic(argc, argv);
    
    /* 
    need open and close by mainlogic class and etc.
    ros::init(argc,argv,"mars_controller");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
	ros::Publisher chatter_publisher = nh.advertise<std_msgs::Float64>("test_topic", 1000);

    ros::Rate rate(25);
    while (nh.ok())
    {
        rate.sleep();
    }
    */

    return a.exec();
}



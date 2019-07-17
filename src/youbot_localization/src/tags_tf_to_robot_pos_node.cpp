#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>


/*!
 * Creates and runs the robot_pose_publisher node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char ** argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "tags_tf_to_robot_pos");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  //double middle = 0;
  //double sqr_middle = 0;
  //long int n = 0;

  // configuring parameters
  std::string map_frame, robot_frame;
  double publish_frequency;
  ros::Publisher p_pub;

  nh_priv.param<std::string>("map_frame",map_frame,"/map");
  nh_priv.param<std::string>("robot_frame",robot_frame,"base_link");
  nh_priv.param<double>("publish_frequency",publish_frequency,10);

  p_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot_pose", 1);

  // create the listener
  tf::TransformListener listener;
  listener.waitForTransform(map_frame, robot_frame, ros::Time(), ros::Duration(1.0));

  ros::Rate rate(publish_frequency);
  while (nh.ok())
  {
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform(map_frame, robot_frame, ros::Time(0), transform);

      // construct a pose message
      geometry_msgs::PoseWithCovarianceStamped pose_stamped;
      pose_stamped.header.frame_id = map_frame;
      pose_stamped.header.stamp = ros::Time::now();

      pose_stamped.pose.pose.orientation.x = transform.getRotation().getX();
      pose_stamped.pose.pose.orientation.y = transform.getRotation().getY();
      pose_stamped.pose.pose.orientation.z = transform.getRotation().getZ();
      pose_stamped.pose.pose.orientation.w = transform.getRotation().getW();

      pose_stamped.pose.pose.position.x = transform.getOrigin().getX();
      pose_stamped.pose.pose.position.y = transform.getOrigin().getY();
      pose_stamped.pose.pose.position.z = transform.getOrigin().getZ();

      //n++;
      //middle = (middle*(n-1)+pose_stamped.pose.pose.orientation.z)/n;
      //sqr_middle = (sqr_middle*(n-1)+pow(pose_stamped.pose.pose.orientation.z-middle,2))/(n);
      
      // todo: need dynamical calculation
      pose_stamped.pose.covariance = {.0,.0,.0,.0,.0,.0,
                                      .0,.0,.0,.0,.0,.0,
                                      .0,.0,.0,.0,.0,.0,
                                      .0,.0,.0,.0,.0,.0,
                                      .0,.0,.0,.0,.0,.0,
                                      .0,.0,.0,.0,.0,.0};

      p_pub.publish(pose_stamped);
    }
    catch (tf::TransformException &ex)
    {
      // just continue on
    }

    rate.sleep();
  }

  return EXIT_SUCCESS;
}

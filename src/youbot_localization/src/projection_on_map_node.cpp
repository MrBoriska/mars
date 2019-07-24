#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


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
  ros::init(argc, argv, "projection_on_map_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // configuring parameters
  std::string map_frame,
              robot_frame,
              camera_frame;
  
  double publish_frequency;
  ros::Publisher p_pub;
  tf::TransformBroadcaster robot2d_tf;

  nh_priv.param<std::string>("camera_frame",camera_frame,"/camera");
  nh_priv.param<std::string>("map_frame",map_frame,"/map");
  nh_priv.param<std::string>("robot_frame",robot_frame,"base_link");
  nh_priv.param<double>("publish_frequency",publish_frequency,10);

  geometry_msgs::TransformStamped tf_msg_rob2d;
  tf_msg_rob2d.header.frame_id = camera_frame;
  tf_msg_rob2d.child_frame_id = robot_frame + "_2d";

  // create the listener
  tf::TransformListener listener;
  //listener.waitForTransform(map_frame, camera_frame, ros::Time(), ros::Duration(1.0));

  ros::Rate rate(publish_frequency);
  while (nh.ok())
  {
    tf::StampedTransform tr_to_map;
    tf::StampedTransform tr_to_rob;
    try
    {
      listener.lookupTransform(camera_frame, map_frame, ros::Time(0), tr_to_map);
      listener.lookupTransform(camera_frame, robot_frame, ros::Time(0), tr_to_rob);

      tf_msg_rob2d.header.stamp = ros::Time::now();

      // point on surface
      tf::Vector3 p = tr_to_map.getOrigin();
      // vectors on surface
      tf::Vector3 v = tr_to_map.getBasis().getColumn(0);
      tf::Vector3 w = tr_to_map.getBasis().getColumn(1);
      // ray to robot position
      tf::Vector3 ray = tr_to_rob.getOrigin().normalize();

      // line parameter (xyz=ray*t)
      double t = (v[0]*(w[1]*p[2]-w[2]*p[1]) +
                  v[1]*(w[2]*p[0]-w[0]*p[2]) +
                  v[2]*(w[0]*p[1]-w[1]*p[0])) / (v[0]*(w[1]*ray[2]-w[2]*ray[1]) +
                                                 v[1]*(w[2]*ray[0]-w[0]*ray[2]) +
                                                 v[2]*(w[0]*ray[1]-w[1]*ray[0]));
      
      // position on surface
      tf_msg_rob2d.transform.translation.x = ray[0]*t;
      tf_msg_rob2d.transform.translation.y = ray[1]*t;
      tf_msg_rob2d.transform.translation.z = ray[2]*t;
      
      // orientation on surface (estimate)
      double roll, pitch, yaw;
      tr_to_map.getBasis().getRPY(roll, pitch, yaw);
      tf_msg_rob2d.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(
        roll,
        pitch,
        tf::getYaw(tr_to_rob.getRotation())
      );

      robot2d_tf.sendTransform(tf_msg_rob2d);
    }
    catch (tf::TransformException &ex)
    {
      // just continue on
    }

    rate.sleep();
  }

  return EXIT_SUCCESS;
}

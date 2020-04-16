#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"

class PickAndPlace
{
public:
  PickAndPlace()
  {
    pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    sub_ = n_.subscribe("/odom", 1000, &PickAndPlace::odom_callback, this);
  

    // Variables
    pose1x = 2.0 + 1.93;  // odom - rviz map pose1x coords = 1.93 mt diff 
    pose1y = 2.0 + 0.77;  // odom - rviz map pose1y coords = 0.77 mt diff
    pose2x = -1.0 + 1.76; // odom - rviz map pose2x coords = 1.76 mt diff
    pose2y = 0.0 + 0.57;  // odom - rviz map pose2y coords = 0.57 mt diff
    threshold = 0.15;
    picked = false;
  
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 2.5;
    marker.pose.position.y = 2;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- .5x.5x.5 here means 50cm on a side
    marker.scale.x = .5;
    marker.scale.y = .5;
    marker.scale.z = .5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (pub_.getNumSubscribers() < 1)
    {
      if (!ros::ok()){}
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    pub_.publish(marker);
    ROS_INFO("marker created !");
  }

  void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    float pose_x = msg->pose.pose.position.x;
    float pose_y = msg->pose.pose.position.y;
    float or_w = msg->pose.pose.orientation.w;

    if(pose_x >= pose1x-threshold && pose_x <= pose1x+threshold && picked == false)
    {
      if(pose_y >= pose1y-threshold && pose_y <= pose1y+threshold && picked == false)
      {
        ros::Duration(1.0).sleep(); // wait for threshold value 
        marker.action = visualization_msgs::Marker::DELETE;
        pub_.publish(marker);
        ROS_INFO("marker deleted !");
        ros::Duration(5.0).sleep();
        picked = true;
      }
    }

    if(pose_x >= pose2x-threshold && pose_x <= pose2x+threshold && picked == true)
    {
      if(pose_y >= pose2y-threshold && pose_y <= pose2y+threshold && picked ==true)
      {
        ros::Duration(1.0).sleep(); // wait for threshold value
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = -1;
        marker.pose.position.y = 0;
        pub_.publish(marker);
        ROS_INFO("marker created !");
        picked = false;
      }
    }
  }
private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  visualization_msgs::Marker marker;
  // Variables
  float pose1x;
  float pose1y;
  float pose2x;
  float pose2y;
  float threshold;
  bool picked;
};

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  PickAndPlace PPObject;
  ros::spin();
  return 0;
}
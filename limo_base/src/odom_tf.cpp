#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class OdomToTF
{
public:
  OdomToTF()
  : nh_("~"), last_stamp_(0)
  {
    // Parameters (with defaults)
    nh_.param<std::string>("odom_topic", odom_topic_, std::string("/odom"));
    nh_.param<std::string>("parent_frame", parent_frame_, std::string("odom"));
    nh_.param<std::string>("child_frame", child_frame_, std::string("base_footprint"));

    sub_ = nh_.subscribe(odom_topic_, 10, &OdomToTF::odomCallback, this);

    ROS_INFO_STREAM("OdomToTF listening on " << odom_topic_
                    << " publishing TF " << parent_frame_
                    << " -> " << child_frame_);
  }

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    // Avoid repeated data: only publish if timestamp is strictly newer
    if (!last_stamp_.isZero() && msg->header.stamp <= last_stamp_)
    {
      // Optionally uncomment for debugging:
      // ROS_DEBUG("Skipping TF broadcast due to non-increasing timestamp");
      return;
    }
    last_stamp_ = msg->header.stamp;

    geometry_msgs::TransformStamped ts;
    ts.header.stamp = msg->header.stamp;
    ts.header.frame_id = parent_frame_;
    ts.child_frame_id = child_frame_;

    ts.transform.translation.x = msg->pose.pose.position.x;
    ts.transform.translation.y = msg->pose.pose.position.y;
    ts.transform.translation.z = msg->pose.pose.position.z;

    ts.transform.rotation = msg->pose.pose.orientation;

    br_.sendTransform(ts);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  tf2_ros::TransformBroadcaster br_;

  std::string odom_topic_;
  std::string parent_frame_;
  std::string child_frame_;

  ros::Time last_stamp_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_to_tf");
  OdomToTF node;
  ros::spin();
  return 0;
}


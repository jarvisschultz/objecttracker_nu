#include <nodelet_topic_tools/nodelet_throttle.h>
#include <sensor_msgs/PointCloud2.h>
#include <pluginlib/class_list_macros.h>
#include <message_filters/subscriber.h>

typedef nodelet_topic_tools::NodeletThrottle<sensor_msgs::PointCloud2> NodeletThrottleCloud;

PLUGINLIB_DECLARE_CLASS (objecttracker_nu, NodeletThrottleCloud, NodeletThrottleCloud, nodelet::Nodelet);

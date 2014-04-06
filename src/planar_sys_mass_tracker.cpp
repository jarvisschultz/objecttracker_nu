// Jarvis Schultz
// September 2012

//---------------------------------------------------------------------------
// Notes
// ---------------------------------------------------------------------------
// This node is for tracking a the suspended mass robot using the
// nodelet launch files to downsample, transform, and pass-through
// filter the PC's.  I am originally using it to send on to the
// planar_coordinator node for filtering and controlling the
// closed-loop planar mass system.

// ---------------------------------------------------------------------------
// Includes
// ---------------------------------------------------------------------------
#include <iostream>
#include <sstream>
#include <fstream>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <puppeteer_msgs/PointPlus.h>
#include <puppeteer_msgs/Robots.h>
#include <puppeteer_msgs/OperatingConditionChange.h>
#include <geometry_msgs/Point.h>

// PCL stuff:
#include <pcl/pcl_base.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h> 
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>


//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------
typedef pcl::PointXYZ PointT;
#define POINT_THRESHOLD (5)
#define MAX_CONSECUTIVE_ERRORS (10)

//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------

class MassTracker
{

private:
    ros::NodeHandle n_;
    ros::Subscriber cloud_sub;
    ros::Publisher mass_pub;
    tf::Transform tf;
    unsigned int error_count;

public:
    MassTracker()
	{
	    ROS_DEBUG("Creating subscribers and publishers");
	    // all publishers and subscribers:
	    cloud_sub = n_.subscribe("mass_box_filter/psz/output", 10,
	    			     &MassTracker::cloudcb, this);
	    mass_pub = n_.advertise<puppeteer_msgs::PointPlus>
	    	("object1_position", 100);
	    
	    tf::StampedTransform t;
	    tf::TransformListener listener;

	    ROS_DEBUG("Looking up transform from kinect to optimization frame");
	    try
	    {
		ros::Time now=ros::Time::now();
		listener.waitForTransform("/camera_depth_optical_frame",
					  "/oriented_optimization_frame",
					  now, ros::Duration(1.0));
		listener.lookupTransform("/oriented_optimization_frame",
					 "/camera_depth_optical_frame",
					 ros::Time(0), t);
		tf = t;	    

	    }
	    catch (tf::TransformException ex)
	    {
		ROS_ERROR("%s", ex.what());
		ros::shutdown();
	    }

	    // set a parameter telling the world that I am tracking the robots
	    ros::param::set("tracking_mass", true);
	    error_count = 0;

	    // wait for operating condition service:
	    ROS_INFO("Waiting for operating_condition_change service:");
	    ROS_INFO("Will not continue until available...");
	    if(ros::service::waitForService("operating_condition_change"))
		ROS_INFO("Service available!");
	    else
	    {
		ROS_ERROR("Could not get service, shutting down");
		ros::shutdown();
	    }

	    
	    return;
	}

    // this function gets called every time new pcl data comes in
    void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan)
	{
	    ROS_DEBUG("Filtered and downsampled cloud receieved in robot tracker");

	    Eigen::Vector4f cent;
	    puppeteer_msgs::PointPlus pt;
	    pcl::PointCloud<pcl::PointXYZ>::Ptr
		cloud (new pcl::PointCloud<pcl::PointXYZ>);

	    // Convert to pcl
	    ROS_DEBUG("Convert incoming cloud to pcl cloud");
	    pcl::fromROSMsg(*scan, *cloud);

	    // now we can find the cent, and determine if we have
	    // an error.  If we do, we can pass that along to the
	    // coordinator node.
	    pt.header.frame_id = scan->header.frame_id;
	    pt.header.stamp = scan->header.stamp;
	    if (cloud->points.size() > POINT_THRESHOLD)
	    {
		pcl::compute3DCentroid(*cloud, cent);
		// then we have successfully found the cent of a
		// valid cloud:
		pt.x = cent(0); pt.y = cent(1); pt.z = cent(2);
		pt.error = false;
		error_count = 0;
	    }
	    else
	    {
		pt.x = 0; pt.y = 0; pt.z = 0;
		pt.error = true;
		error_count++;
	    }

	    
	    if (error_count > MAX_CONSECUTIVE_ERRORS)
	    {
		puppeteer_msgs::OperatingConditionChange::Request req;
		puppeteer_msgs::OperatingConditionChange::Response resp;
		req.state.state = req.state.EMERGENCY;
		ros::service::call("operating_condition_change", req, resp);
		ROS_ERROR("Lost mass too many times!");
		ros::shutdown();
	    }
	    // finally publish the results:
	    mass_pub.publish(pt);
	}
}; // End of MassTracker class


//---------------------------------------------------------------------------
// Main
//---------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planar_mass_tracker");

    // turn on debugging
    // log4cxx::LoggerPtr my_logger =
    // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    // my_logger->setLevel(
    // ros::console::g_level_lookup[ros::console::levels::Debug]);

    ros::NodeHandle n;

    ROS_INFO("Starting planar mass tracker...\n");
    MassTracker tracker;
  
    ros::spin();
  
    return 0;
}

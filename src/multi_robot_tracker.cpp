// Jake Ware and Jarvis Schultz
// September 13, 2011

//---------------------------------------------------------------------------
// Notes
// ---------------------------------------------------------------------------
// This node is for tracking a robot.  It should be launched from a
// launch file that defines a coordinate transform from the kinect's
// depth frame to a frame aligned with my optimization code.

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include <iostream>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <puppeteer_msgs/PointPlus.h>
#include <puppeteer_msgs/Robots.h>
#include <geometry_msgs/Point.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/feature.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/pcl_base.h>
#include <pcl/common/transform.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>


//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------
#define POINT_THRESHOLD (5)
#define MAX_CLUSTERS 5
typedef pcl::PointXYZ PointT;


//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------

class RobotTracker
{

private:
    ros::NodeHandle n_;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub[MAX_CLUSTERS];
    ros::Publisher robots_pub;
    bool locate;
    Eigen::Affine3f const_transform;
    tf::Transform tf;

public:
    RobotTracker()
	{
	    ROS_DEBUG("Creating subscribers and publishers");
	    cloud_sub = n_.subscribe("/camera/depth/points", 1,
				     &RobotTracker::cloudcb, this);
	    robots_pub = n_.advertise<puppeteer_msgs::Robots>
		("robot_positions", 100);
	    cloud_pub[0] = n_.advertise<sensor_msgs::PointCloud2>
		("filtered_cloud", 1);
	    cloud_pub[1] = n_.advertise<sensor_msgs::PointCloud2>
		("cluster_1_cloud", 1);
	    cloud_pub[2] = n_.advertise<sensor_msgs::PointCloud2>
		("cluster_2_cloud", 1);
	    cloud_pub[3] = n_.advertise<sensor_msgs::PointCloud2>
		("cluster_3_cloud", 1);
	    cloud_pub[4] = n_.advertise<sensor_msgs::PointCloud2>
		("cluster_4_cloud", 1);
  
	    locate = true;
	    tf::StampedTransform t;
	    tf::TransformListener listener;

	    ROS_DEBUG("Looking up transform from kinect to optimization frame");
	    try
	    {
		ros::Time now=ros::Time::now();
		listener.waitForTransform("/openni_depth_optical_frame",
					  "/oriented_optimization_frame",
					  now, ros::Duration(1.0));
		listener.lookupTransform("/oriented_optimization_frame",
					 "/openni_depth_optical_frame",
					 ros::Time(0), t);
		tf = t;	    

	    }
	    catch (tf::TransformException ex)
	    {
		ROS_ERROR("%s", ex.what());
		ros::shutdown();
	    }
	}

    // this function gets called every time new pcl data comes in
    void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan)
	{
	    ROS_DEBUG("cloudcb started");
	    ros::Time time = ros::Time::now();
	    // Eigen::Vector4f centroid;
	    // float D_sphere = 0.05; //meters
	    // float R_search = 2.0*D_sphere;
	    geometry_msgs::Point point;
	    pcl::PassThrough<pcl::PointXYZ> pass;
	    Eigen::VectorXf lims(6);
	    sensor_msgs::PointCloud2::Ptr
		ros_cloud (new sensor_msgs::PointCloud2 ()),
		ros_cloud_filtered (new sensor_msgs::PointCloud2 ());    

	    pcl::PointCloud<pcl::PointXYZ>::Ptr
		cloud (new pcl::PointCloud<pcl::PointXYZ> ()),
		cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());

	    // set a parameter telling the world that I am tracking the robot
	    ros::param::set("/tracking_robot", true);

	    ROS_DEBUG("About to transform cloud");
	    // New sensor message for holding the transformed data
	    sensor_msgs::PointCloud2::Ptr scan_transformed
		(new sensor_msgs::PointCloud2());
	    try{
		pcl_ros::transformPointCloud("/oriented_optimization_frame",
					     tf, *scan, *scan_transformed);
	    }
	    catch(tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
	    }
	    scan_transformed->header.frame_id = "/oriented_optimization_frame";

	    // Convert to pcl
	    ROS_DEBUG("Convert cloud to pcd from rosmsg");
	    pcl::fromROSMsg(*scan_transformed, *cloud);

	    // set time stamp and frame id
	    ros::Time tstamp = ros::Time::now();

	    // run through pass-through filter to eliminate tarp and below robots.
	    ROS_DEBUG("Pass-through filter");
	    lims << -1.0, 1.0, -0.1, 1.0, 0.0, 3.5;
	    pass_through(cloud, cloud_filtered, lims);

	    // now let's publish that filtered cloud
	    ROS_DEBUG("Converting and publishing cloud");		      
	    pcl::toROSMsg(*cloud_filtered, *ros_cloud_filtered);
	    ros_cloud_filtered->header.frame_id =
		"/oriented_optimization_frame";
	    cloud_pub[0].publish(ros_cloud_filtered);

	    // create a pcd file:
	    pcl::PCDWriter writer;
	    std::stringstream ss;
	    ss << "cloud_filtered.pcd";
	    static bool create = true;
	    if (create)
	    {
		create = false;
		writer.write<pcl::PointXYZ> (ss.str (), *cloud_filtered, false); //*
	    }
	    
	    // std::cout << "Original size = " << cloud->points.size() << std::endl;
	    // std::cout << "Filtered size = " << cloud_filtered->points.size() << std::endl;
	    
	    // ROS_DEBUG("Begin extraction filtering");
	    // // build a KdTree object for the search method of the extraction
	    // pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree
	    // 	(new pcl::KdTreeFLANN<pcl::PointXYZ> ());
	    // tree->setInputCloud (cloud_filtered);

	    // // create a vector for storing the indices of the clusters
	    // std::vector<pcl::PointIndices> cluster_indices;

	    // // setup extraction:
	    // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	    // ec.setClusterTolerance (0.02); // 2cm
	    // ec.setMinClusterSize (30);
	    // ec.setMaxClusterSize (1000);
	    // ec.setSearchMethod (tree);
	    // ec.setInputCloud (cloud_filtered);

	    // // now we can perform cluster extraction
	    // ec.extract (cluster_indices);

	    // // run through the indices, create new clouds, and then publish them
	    // int j=1;
	    // for (std::vector<pcl::PointIndices>::const_iterator
	    // 	     it=cluster_indices.begin();
	    // 	 it!=cluster_indices.end (); ++it)
	    // {
	    // 	ROS_DEBUG_THROTTLE(1,"Number of clusters found: %d",
	    // 			   (int) cluster_indices.size());
	    // 	pcl::PointCloud<pcl::PointXYZ>::Ptr
	    // 	    cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	    // 	for (std::vector<int>::const_iterator
	    // 		 pit = it->indices.begin ();
	    // 	     pit != it->indices.end (); pit++)
	    // 	{
	    // 	    cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
	    // 	}
	    // 	cloud_cluster->width = cloud_cluster->points.size ();
	    // 	cloud_cluster->height = 1;
	    // 	cloud_cluster->is_dense = true;

	    // 	// convert to rosmsg and publish:
	    // 	ROS_DEBUG("Publishing extracted cloud");
	    // 	pcl::toROSMsg(*cloud_filtered, *ros_cloud_filtered);
	    // 	if(j<MAX_CLUSTERS)
	    // 	    cloud_pub[j].publish(ros_cloud_filtered);
	    // 	else
	    // 	    ROS_DEBUG("Too many clusters found, on number %d",j);
	    // 	j++;
	    // }
	    
	    // pcl::compute3DCentroid(*cloud_filtered, centroid);
	    // xpos = centroid(0); ypos = centroid(1); zpos = centroid(2);

	    // pcl::toROSMsg(*cloud_filtered, *robot_cloud);
	    // robot_cloud_filtered->header.frame_id =
	    // 	"/oriented_optimization_frame";
	    // cloud_pub[0].publish(robot_cloud);
	    ros::Duration d = ros::Time::now()-time;
	    ROS_DEBUG("End of cloudcb; time elapsed = %f", d.toSec());
	}

    void pass_through(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
		      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
		      const Eigen::VectorXf lims)
	{
	    pcl::PointCloud<pcl::PointXYZ>::Ptr
		cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZ> ()),
		cloud_filtered_y (new pcl::PointCloud<pcl::PointXYZ> ());
	    pcl::PassThrough<pcl::PointXYZ> pass;

	    if(lims.size() != 6)
	    {
		ROS_WARN("Limits for pass-through wrong size");
		return;
	    }

	    pass.setInputCloud(cloud_in);
	    pass.setFilterFieldName("x");
	    pass.setFilterLimits(lims(0), lims(1));
	    pass.filter(*cloud_filtered_x);
	    
	    pass.setInputCloud(cloud_filtered_x);
	    pass.setFilterFieldName("y");
	    pass.setFilterLimits(lims(2), lims(3));
	    pass.filter(*cloud_filtered_y);
	    
	    pass.setInputCloud(cloud_filtered_y);
	    pass.setFilterFieldName("z");
	    pass.setFilterLimits(lims(4), lims(5));
	    pass.filter(*cloud_out);

	    return;
	}
		    
};


//---------------------------------------------------------------------------
// Main
//---------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_tracker");

    // turn on debugging
    log4cxx::LoggerPtr my_logger =
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    my_logger->setLevel(
    ros::console::g_level_lookup[ros::console::levels::Debug]);

    ros::NodeHandle n;

    ROS_INFO("Starting Robot Tracker...\n");
    RobotTracker tracker;
  
    ros::spin();
  
    return 0;
}

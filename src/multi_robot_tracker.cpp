// Jarvis Schultz
// April 2011

//---------------------------------------------------------------------------
// Notes
// ---------------------------------------------------------------------------
// This node is for tracking mulitple robots.  A pass-through filter
// is applied to the raw Kinect data.  The filtered data is then
// downsampled.  The downsampled data then has a Euclidean cluster
// extraction algorithm applied to find the robots.  The centroids of
// these clusters are then passed onto the coordinator node to handle
// the data association problem.  

// ---------------------------------------------------------------------------
// Includes
// ---------------------------------------------------------------------------

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

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
#include <geometry_msgs/Point.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/pcl_base.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>


//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------
#define MAX_CLUSTERS 4
typedef pcl::PointXYZ PointT;
std::string filename;

//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------

class RobotTracker
{

private:
    ros::NodeHandle n_;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub[MAX_CLUSTERS+1];
    ros::Publisher robots_pub;
    Eigen::Affine3f const_transform;
    tf::Transform tf;
    int number_robots;
    Eigen::VectorXf robot_limits;
    Eigen::Vector4f centroid;
    

public:
    RobotTracker()
	{
	    // get filter limits:
	    get_frame_limits(filename);

	    ROS_DEBUG("Creating subscribers and publishers");
	    cloud_sub = n_.subscribe("/camera/depth/points", 1,
	    			     &RobotTracker::cloudcb, this);
	    robots_pub = n_.advertise<puppeteer_msgs::Robots>
	    	("/robot_positions", 100);
	    cloud_pub[0] = n_.advertise<sensor_msgs::PointCloud2>
		("/filtered_cloud", 1);
	    int i = 1;
	    for (i=1; i<MAX_CLUSTERS+1; i++)
	    {
		std::stringstream ss;
		ss << "/cluster_" << i << "_cloud";
		cloud_pub[i] = n_.advertise<sensor_msgs::PointCloud2>
		    (ss.str(), 1);
	    }
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

	    // get number of robots
	    if(ros::param::has("/number_robots")) 
		ros::param::get("/number_robots", number_robots);
	    else
		number_robots = 1;
	    return;
	}

    // this function gets called every time new pcl data comes in
    void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan)
	{
	    ROS_INFO("Kinect point cloud receieved");
	    ros::Time start_time = ros::Time::now();
	    ros::Time tcur = ros::Time::now();

	    Eigen::Vector4f centroid;
	    geometry_msgs::Point point;
	    pcl::PassThrough<pcl::PointXYZ> pass;
	    Eigen::VectorXf lims(6);
	    sensor_msgs::PointCloud2::Ptr
		ros_cloud (new sensor_msgs::PointCloud2 ()),
		ros_cloud_filtered (new sensor_msgs::PointCloud2 ());    
	    pcl::PointCloud<pcl::PointXYZ>::Ptr
	    	cloud (new pcl::PointCloud<pcl::PointXYZ> ()),
	    	cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ> ()),
	    	cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
	    
	    // set a parameter telling the world that I am tracking the robot
	    ros::param::set("/tracking_robot", true);

	    ROS_DEBUG("finished declaring vars : %f", (ros::Time::now()-tcur).toSec());
	    tcur = ros::Time::now();

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

	    ROS_DEBUG("cloud transformed and converted to pcl : %f",
		      (ros::Time::now()-tcur).toSec());
	    tcur = ros::Time::now();
	    
	    // set time stamp and frame id
	    ros::Time tstamp = ros::Time::now();

	    // run through pass-through filter to eliminate tarp and below robots.
	    ROS_DEBUG("Pass-through filter");
	    pass_through(cloud, cloud_filtered, robot_limits);

	    ROS_DEBUG("done with pass-through : %f", (ros::Time::now()-tcur).toSec());
	    tcur = ros::Time::now();

	    // now let's publish that filtered cloud
	    ROS_DEBUG("Converting and publishing cloud");		      
	    pcl::toROSMsg(*cloud_filtered, *ros_cloud_filtered);
	    ros_cloud_filtered->header.frame_id =
		"/oriented_optimization_frame";
	    cloud_pub[0].publish(ros_cloud_filtered);

	    ROS_DEBUG("published filtered cloud : %f", (ros::Time::now()-tcur).toSec());
	    tcur = ros::Time::now();

	    // Let's do a downsampling before doing cluster extraction
	    pcl::VoxelGrid<pcl::PointXYZ> vg;
	    vg.setInputCloud (cloud_filtered);
	    vg.setLeafSize (0.01f, 0.01f, 0.01f);
	    vg.filter (*cloud_downsampled);

	    ROS_DEBUG("finished downsampling : %f", (ros::Time::now()-tcur).toSec());
	    tcur = ros::Time::now();


	    ROS_DEBUG("Begin extraction filtering");
	    // build a KdTree object for the search method of the extraction
	    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree
	    	(new pcl::search::KdTree<pcl::PointXYZ> ());
	    tree->setInputCloud (cloud_downsampled);
	    
	    ROS_DEBUG("done with KdTree initialization : %f",
		      (ros::Time::now()-tcur).toSec());
	    tcur = ros::Time::now();


	    // create a vector for storing the indices of the clusters
	    std::vector<pcl::PointIndices> cluster_indices;

	    // setup extraction:
	    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	    ec.setClusterTolerance (0.02); // cm
	    ec.setMinClusterSize (50);
	    ec.setMaxClusterSize (3000);
	    ec.setSearchMethod (tree);
	    ec.setInputCloud (cloud_downsampled);

	    // now we can perform cluster extraction
	    ec.extract (cluster_indices);

	    ROS_DEBUG("finished extraction : %f", (ros::Time::now()-tcur).toSec());
	    tcur = ros::Time::now();

	    // run through the indices, create new clouds, and then publish them
	    int j=1;
	    int number_clusters=0;
	    geometry_msgs::PointStamped pt;
	    puppeteer_msgs::Robots robots;
	    std::vector<int> num;

	    for (std::vector<pcl::PointIndices>::const_iterator
	    	     it=cluster_indices.begin();
	    	 it!=cluster_indices.end (); ++it)
	    {
		number_clusters = (int) cluster_indices.size();
	    	ROS_DEBUG("Number of clusters found: %d",number_clusters);
	    	pcl::PointCloud<pcl::PointXYZ>::Ptr
	    	    cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	    	for (std::vector<int>::const_iterator
	    		 pit = it->indices.begin ();
	    	     pit != it->indices.end (); pit++)
	    	{
	    	    cloud_cluster->points.push_back(cloud_downsampled->points[*pit]);
	    	}
	    	cloud_cluster->width = cloud_cluster->points.size ();
	    	cloud_cluster->height = 1;
	    	cloud_cluster->is_dense = true;

	    	// convert to rosmsg and publish:
	    	ROS_DEBUG("Publishing extracted cloud");
	    	pcl::toROSMsg(*cloud_cluster, *ros_cloud_filtered);
		ros_cloud_filtered->header.frame_id =
		    "/oriented_optimization_frame";
	    	if(j < MAX_CLUSTERS+1)
	    	    cloud_pub[j].publish(ros_cloud_filtered);
	    	j++;

		// compute centroid and add to Robots:
		pcl::compute3DCentroid(*cloud_cluster, centroid);
		pt.point.x = centroid(0);
		pt.point.y = centroid(1);
		pt.point.z = centroid(2);
		pt.header.stamp = tstamp;
		pt.header.frame_id = "/oriented_optimization_frame";
		robots.robots.push_back(pt);
		// add number of points in cluster to num:
		num.push_back(cloud_cluster->points.size());		
	    }

	    ROS_DEBUG("finished creating and publishing clusters : %f", 
		      (ros::Time::now()-tcur).toSec());
	    tcur = ros::Time::now();


	    if (number_clusters > number_robots)
	    {
		ROS_WARN("Number of clusters found is greater "
			 "than the number of robots");
		// pop minimum cluster count
		remove_least_likely(&robots, &num);
	    }
	    
	    robots.header.stamp = tstamp;
	    robots.header.frame_id = "/oriented_optimization_frame";
	    robots.number = number_clusters;

	    robots_pub.publish(robots);

	    ROS_DEBUG("removed extra clusters, and published : %f",
		      (ros::Time::now()-tcur).toSec());
	    tcur = ros::Time::now();

	    ros::Duration d = ros::Time::now() - start_time;
	    ROS_DEBUG("End of cloudcb; time elapsed = %f (%f Hz)",
		      d.toSec(), 1/d.toSec());
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

    void remove_least_likely(puppeteer_msgs::Robots* r, std::vector<int>* n)
	{
	    // we just need to find the point cloud with the fewest
	    // number of points, and pop it out of r and n

	    int j=0;
	    int number_clusters = (int) (n->size());
	    ROS_DEBUG("cl-ro = %d - %d = %d",number_clusters, number_robots,
		      number_clusters-number_robots);

	    for (j=0; j<number_clusters-number_robots; j++)
	    {
		// get location of minimum element:
		std::vector<int>::iterator loc =
		    std::min_element(n->begin(), n->end());
		// pop values
		r->robots.erase(r->robots.begin()+std::distance(n->begin(), loc));
		n->erase(n->begin()+std::distance(n->begin(), loc));
	    }	    
	    
	    return;
	}

    void get_frame_limits(std::string f)
	{
	    // first define the size of of the limits vector:
	    robot_limits.resize(6);

	    // open the file
	    std::ifstream file;
	    std::string line;
	    float tmp;
	    file.open(f.c_str(), std::fstream::in);
	    for (int i=0; i<6; i++)
	    {
		getline(file, line);
		tmp = atof(line.c_str());
		robot_limits(i) = tmp;
	    }
	    file.close();

	    return;
	}
		    
};


//---------------------------------------------------------------------------
// Main
//---------------------------------------------------------------------------

int main(int argc, char **argv)
{
    // get the filename:
    std::string working_dir;
    working_dir = argv[0];
    std::size_t found = working_dir.find("bin");
    std::string tmp_dir = working_dir.substr(0, found);
    working_dir = tmp_dir+"launch/";
    filename = working_dir+"robot_limits.txt";
        
    ros::init(argc, argv, "multi_robot_tracker");

    // turn on debugging
    // log4cxx::LoggerPtr my_logger =
    // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    // my_logger->setLevel(
    // ros::console::g_level_lookup[ros::console::levels::Debug]);

    ros::NodeHandle n;

    ROS_INFO("Starting Multi-Robot Tracker...\n");
    RobotTracker tracker;
  
    ros::spin();
  
    return 0;
}

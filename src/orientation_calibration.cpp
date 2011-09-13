
// nu_objecttracker.cpp
// Jake Ware and Jarvis Schultz
// Winter 2011

//---------------------------------------------------------------------------
// Notes
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <kbhit.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <log4cxx/logger.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <puppeteer_msgs/PointPlus.h>
#include <puppeteer_msgs/speed_command.h>
#include <geometry_msgs/Point.h>

#include <pcl/common/transform.h>
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
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <assert.h>


//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------
#define POINT_THRESHOLD (5)
typedef pcl::PointXYZ PointT;
std::string working_dir;

//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------

class ObjectTracker
{

private:
    ros::NodeHandle n_;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub[3];
    ros::Publisher point_pub;
    ros::Publisher pointplus_pub;
    ros::Publisher point_pub2;
    ros::Publisher pointplus_pub2;
    float xpos_last;
    float ypos_last;
    float zpos_last;
    bool locate;
    puppeteer_msgs::speed_command srv;
    bool complete_flag;

public:
    ObjectTracker()
	{
	    cloud_sub = n_.subscribe("/camera/depth/points", 1, &ObjectTracker::cloudcb, this);
	    point_pub = n_.advertise<geometry_msgs::Point> ("object1_position_2", 100);
	    pointplus_pub = n_.advertise<puppeteer_msgs::PointPlus> ("object1_position", 100);
	    point_pub2 = n_.advertise<geometry_msgs::Point> ("object2_position_2", 100);
	    pointplus_pub2 = n_.advertise<puppeteer_msgs::PointPlus> ("object2_position", 100);
	    cloud_pub[0] = n_.advertise<sensor_msgs::PointCloud2> ("object1_cloud", 1);
	    cloud_pub[1] = n_.advertise<sensor_msgs::PointCloud2> ("object2_cloud", 1);
	    cloud_pub[2] = n_.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1);
  
	    xpos_last = 0.0;
	    ypos_last = 0.0;
	    zpos_last = 0.0;
	    locate = true;
	    complete_flag = false;
	}

    
    Eigen::Vector3f planar_inliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
				   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
	{
	    static int call_count = 0;
	    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	    pcl::ExtractIndices<pcl::PointXYZ> extract;
	    sensor_msgs::PointCloud2::Ptr
		filtered_cloud (new sensor_msgs::PointCloud2 ());
	    Eigen::Vector3f mean_normal;

	    ROS_DEBUG("Starting planar_inliers");
	    
	    // Create the segmentation object
	    pcl::SACSegmentation<pcl::PointXYZ> seg;
	    // Optional
	    seg.setOptimizeCoefficients (true);
	    // Mandatory
	    seg.setModelType (pcl::SACMODEL_PLANE);
	    seg.setMethodType (pcl::SAC_RANSAC);
	    seg.setDistanceThreshold (0.01);

	    seg.setInputCloud (cloud_in->makeShared ());
	    // Inliers has indices in cloud_in belonging to plane,
	    // coefficients has plane eqn coeffs i.e. ax+by+cz+d=0
	    ROS_DEBUG("About to segment planar model");
	    seg.segment (*inliers, *coefficients);

	    ROS_DEBUG("Extracting new cloud");
	    extract.setInputCloud(cloud_in);
	    extract.setIndices(inliers);
	    extract.setNegative(false);
	    extract.filter(*cloud_out);

	    ROS_DEBUG("Publishing new point cloud");
	    // Now, we can publish the cloud:
	    pcl::toROSMsg(*cloud_out, *filtered_cloud);
	    cloud_pub[2].publish(filtered_cloud);

	    ROS_DEBUG("Setting up normal estimation parameters");
	    // Create a KD-Tree
	    pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
	    tree->setInputCloud (cloud_out);
	    // Now, let's estimate the normals of this cloud:
	    pcl::PointCloud<pcl::PointXYZ> mls_points;
	    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::Normal> mls;
	    pcl::PointCloud<pcl::Normal>::Ptr mls_normals (new pcl::PointCloud<pcl::Normal> ());
	    mls.setOutputNormals (mls_normals);

	    // Set sampling parameters
	    mls.setInputCloud (cloud_out);
	    mls.setPolynomialFit (true);
	    mls.setSearchMethod (tree);
	    mls.setSearchRadius (0.03);

	    ROS_DEBUG("Finding normals");
	    // Filter:
	    mls.reconstruct(mls_points);

	    ROS_DEBUG("Finding mean value");
	    // Now, let's find the mean normal value:
	    float avgnormal[3] = {0.0,0.0,0.0};
	    float avgcounts[3] = {0.0,0.0,0.0};
	    for(int i=0; i < (int) mls_normals->points.size(); i++)
	    {
		for(int j=0; j<3; j++)
		    if(!std::isnan(mls_normals->points.at(i).normal[j]))
		    {
			avgnormal[j] += mls_normals->points.at(i).normal[j];
			avgcounts[j] += 1.0;
		    }
	    }
	    for(int j=0; j<3; j++)
		avgnormal[j] /= avgcounts[j];
	    call_count++;
	    printf("Num = %3d     ", call_count);
	    std::cout <<
		"X Normal = " << avgnormal[0] << "\t" <<
		"Y Normal = " << avgnormal[1] << "\t" << 
		"Z Normal = " << avgnormal[2] << "\n";

	    ROS_DEBUG("Setting mean_normal vector");
	    mean_normal << avgnormal[0], avgnormal[1], avgnormal[2];

	    ROS_DEBUG("Leaving planar_inliers");
	    return(mean_normal);
	}

    // this function gets called every time new pcl data comes in
    void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan)
	{
	    sensor_msgs::PointCloud2::Ptr
		object1_cloud (new sensor_msgs::PointCloud2 ()),
		object2_cloud (new sensor_msgs::PointCloud2 ());    

	    pcl::PointCloud<pcl::PointXYZ>::Ptr
		cloud (new pcl::PointCloud<pcl::PointXYZ> ()),
		cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZ> ()),
		cloud_filtered_y (new pcl::PointCloud<pcl::PointXYZ> ()),
		cloud_filtered_z (new pcl::PointCloud<pcl::PointXYZ> ()),
		planar_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

	    // convert pcl2 message to pcl object
	    pcl::fromROSMsg(*scan,*cloud);
	
	    Eigen::Vector4f centroid;
	    float xpos = 0.0;
	    float ypos = 0.0;
	    float zpos = 0.0;
	    // float D_sphere = 0.05; //meters
	    float D_sphere = 0.1; //meters
	    float R_search = 2.0*D_sphere;

	    // create new points to store object position
	    puppeteer_msgs::PointPlus pointplus;
	    geometry_msgs::Point point;

	    // pass through filter
	    pcl::PassThrough<pcl::PointXYZ> pass;

	    // set time stamp and frame id
	    ros::Time tstamp = ros::Time::now();
	    pointplus.header.stamp = tstamp;
	    pointplus.header.frame_id = "depth_optical_frame";

	    // did we lose the object?
	    if (locate == true)
	    {
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("x");
		pass.setFilterLimits(-0.80, 0.80);
		pass.filter(*cloud_filtered_x);

		pass.setInputCloud(cloud_filtered_x);
		pass.setFilterFieldName("y");
		pass.setFilterLimits(-5.0, 0.5);
		pass.filter(*cloud_filtered_y);

		pass.setInputCloud(cloud_filtered_y);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0, 3.25);
		pass.filter(*cloud_filtered_z);
    
		pcl::compute3DCentroid(*cloud_filtered_z, centroid);
		xpos = centroid(0);
		ypos = centroid(1);
		zpos = centroid(2);

		// Publish cloud:
		pcl::toROSMsg(*cloud_filtered_z, *object2_cloud);
		cloud_pub[1].publish(object2_cloud); 
		
		// are there enough points in the point cloud?
		if(cloud_filtered_z->points.size() > POINT_THRESHOLD)
		{
		    locate = false;  // We have re-found the object!

		    // set point message values and publish
		    point.x = xpos;
		    point.y = ypos;
		    point.z = zpos;
	  
		    point_pub.publish(pointplus);
	  
		    // set values to publish
		    pointplus.x = xpos;
		    pointplus.y = ypos;
		    pointplus.z = zpos;
		    pointplus.error = false;

		    pointplus_pub.publish(pointplus);
		}
		// otherwise we should publish a blank centroid
		// position with an error flag
		else
		{
		    // set values to publish
		    pointplus.x = 0.0;
		    pointplus.y = 0.0;
		    pointplus.z = 0.0;
		    pointplus.error = true;

		    pointplus_pub.publish(pointplus);	    
		}
	    }
	    // otherwise find the centroid again
	    else
	    {
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("x");
		pass.setFilterLimits(xpos_last-2.0*R_search, xpos_last+2.0*R_search);
		pass.filter(*cloud_filtered_x);

		pass.setInputCloud(cloud_filtered_x);
		pass.setFilterFieldName("y");
		pass.setFilterLimits(ypos_last-R_search, ypos_last+R_search);
		pass.filter(*cloud_filtered_y);

		pass.setInputCloud(cloud_filtered_y);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(zpos_last-R_search, zpos_last+R_search);
		pass.filter(*cloud_filtered_z);
 
		// are there enough points in the point cloud?
		if(cloud_filtered_z->points.size() < POINT_THRESHOLD)
		{
		    locate = true;
	  	  
		    ROS_WARN("Lost Object at: x = %f  y = %f  z = %f\n",
			     xpos_last,ypos_last,zpos_last);
	  
		    pointplus.x = 0.0;
		    pointplus.y = 0.0;
		    pointplus.z = 0.0;
		    pointplus.error = true;

		    pointplus_pub.publish(pointplus);
		    	  	  
		    return;
		}
	
		pcl::compute3DCentroid(*cloud_filtered_z, centroid);
		xpos = centroid(0);
		ypos = centroid(1);
		zpos = centroid(2);

		pcl::toROSMsg(*cloud_filtered_z, *object1_cloud);
		cloud_pub[0].publish(object1_cloud);

		tf::Transform transform;
		transform.setOrigin(tf::Vector3(xpos, ypos, zpos));
		transform.setRotation(tf::Quaternion(0, 0, 0, 1));

		static tf::TransformBroadcaster br;
		br.sendTransform(tf::StampedTransform
				 (transform,ros::Time::now(),
				  "openni_rgb_optical_frame","object1"));

		// set point message values and publish
		point.x = xpos;
		point.y = ypos;
		point.z = zpos;
    
		point_pub.publish(pointplus);

		// set pointplus message values and publish
		pointplus.x = xpos;
		pointplus.y = ypos;
		pointplus.z = zpos;
		pointplus.error = false;

		pointplus_pub.publish(pointplus);
	    }
		
	    xpos_last = xpos;
	    ypos_last = ypos;
	    zpos_last = zpos;

	    // Filter the segmented cloud to get the planar inliers:
	    static int call_flag = 0, call_count = 0;
	    static bool calibrate_flag = true;
	    static Eigen::Vector3f current_normal(0.0,0.0,0.0),
		count_normal(0.0,0.0,0.0);
	    
	    if (call_flag%5 == 0 && calibrate_flag == true)
	    {
		ROS_DEBUG("Calling planar_inliers for the %d time",call_count+1);
		current_normal = planar_inliers(cloud_filtered_z, planar_cloud);
		ROS_DEBUG("Incrementing counts and adding vectors");
		call_count++;
		count_normal += current_normal;
	    }
	    call_flag++;
	    if (call_count == 20)
	    {
	    	count_normal /= call_count;
	    	std::cout << "Mean Normal:\n" << count_normal << std::endl;
	    	call_count++;
		calibrate_flag = false;

		// Now we can call the function generates a transform
		// based on this vector
		generate_transform(count_normal);
	    }
	    else if (call_count == 21)
	    {
		complete_flag = true;
		ROS_INFO_ONCE("Calibration Successfully Completed");
		ROS_INFO_ONCE("Press any button to kill node and write out transformation data");
		generate_transform(count_normal);
	    }

		
	    
	}

    void generate_transform(Eigen::Vector3f normal)
	{
	    Eigen::Vector3f xvec,yvec,zvec;
	    Eigen::Affine3f t;
	    Eigen::Vector3f orig(0.0,0.0,0.0);

	    if( normal(2) >= 0)
		// Then normal is into vertical surface
		zvec = normal;
	    else
		// Normal is towards kinect
		zvec = -normal;
	    xvec << 1.0,0.0,0.0;
	    yvec << zvec.cross(xvec);

	    // current vectors are aligned with gravity, but let's now
	    // tranform the data into the same coordinate system that
	    // I use in my optimizations
	    ROS_DEBUG("Getting Transformation as Affine3f");
	    pcl::getTransformationFromTwoUnitVectorsAndOrigin(
	    	yvec, zvec, orig, t);

	    // Now let's convert that to a tf, and create a tf publisher:
	    // tf::Transform transform;
	    static tf::TransformBroadcaster br;
	    // transform.setOrigin(tf::Vector3(orig(0),orig(1),orig(2)));
	    ROS_DEBUG("Extract Rotation Matrix");
	    Eigen::Matrix3f Rot = t.linear();
	    // transform.setRotation(btMatrix3x3(Rot(1,1),Rot(1,2),Rot(1,3),
	    // 				      Rot(2,1),Rot(2,2),Rot(2,3),
	    // 				      Rot(3,1),Rot(3,2),Rot(3,3)));
	    ROS_DEBUG("Instantiate a new tf Transform");
	    tf::Transform tr = tf::Transform(btMatrix3x3
					     (Rot(0,0),Rot(0,1),Rot(0,2),
					      Rot(1,0),Rot(1,1),Rot(1,2),
					      Rot(2,0),Rot(2,1),Rot(2,2)),
					     btVector3(orig(0),orig(1),orig(2)));
	    
	    br.sendTransform(tf::StampedTransform
			     (tr, ros::Time::now(), "openni_rgb_optical_frame",
			      "oriented_optimization_frame"));

	    if (complete_flag == true)
	    {
		if (kbhit())
		    write_calibration(t);
	    }
	}

    void write_calibration(Eigen::Affine3f tr)
	{
	    ROS_DEBUG("About to write text file");
	    // This function writes out a calibrated transformation to
	    // a text file that can be read by other tracking codes
	    std::ofstream file;
	    std::string filename;
	    std::size_t found = working_dir.find("bin");
	    std::string tmp_dir = working_dir.substr(0, found);
	    working_dir = tmp_dir+"data/";

	    file.open(filename.c_str());

	    cout << filename << "\n";

	    ROS_DEBUG("Converting and writing");
	    // First let's write out the rotation matrix:
	    Eigen::Matrix3f Rot = tr.linear();
	    file << Rot << std::endl;

	    // Now write out the transformation vector:
	    Eigen::Vector3f orig(0.0,0.0,0.0);
	    file << "\n" << orig << std::endl;

	    ROS_DEBUG("Killing node");
	    file.close();
	    ros::shutdown();			    
	}

		      
	    
};


//---------------------------------------------------------------------------
// main
//---------------------------------------------------------------------------

int main(int argc, char **argv)
{
    working_dir = argv[0];
    
    ROSCONSOLE_AUTOINIT;
    ros::init(argc, argv, "system_calibrator");
    // log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    // my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
    
    ros::NodeHandle n;
    
    ROS_INFO("Starting Calibrations...\n");
    ObjectTracker tracker;
    
    ros::spin();
  
    return 0;
}

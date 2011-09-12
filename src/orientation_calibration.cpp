
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

#include <iostream>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <puppeteer_msgs/PointPlus.h>
#include <puppeteer_msgs/speed_command.h>
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
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>


//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------
#define POINT_THRESHOLD (5)
typedef pcl::PointXYZ PointT;


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
	}

    
    void planar_inliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
	{
	    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	    pcl::ExtractIndices<pcl::PointXYZ> extract;
	    sensor_msgs::PointCloud2::Ptr
		filtered_cloud (new sensor_msgs::PointCloud2 ());
	    
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
	    seg.segment (*inliers, *coefficients);

	    extract.setInputCloud(cloud_in);
	    extract.setIndices(inliers);
	    extract.setNegative(false);
	    extract.filter(*cloud_out);

	    // Now, we can publish the cloud:
	    pcl::toROSMsg(*cloud_out, *filtered_cloud);
	    cloud_pub[2].publish(filtered_cloud);

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

	    // Filter:
	    mls.reconstruct(mls_points);

	    // Now, let's find the mean normal value:
	    double avgnormal[3] = {0.0,0.0,0.0};
	    double avgcounts[3] = {0.0,0.0,0.0};
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

	    std::cout <<
		"X Average Normal = " << avgnormal[0] << "\t"
		"Y Average Normal = " << avgnormal[1] << "\t"
		"Z Average Normal = " << avgnormal[2] << "\n";
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
	    static int call_flag = 0;
	    if (call_flag%5 == 0)
		planar_inliers(cloud_filtered_z, planar_cloud);
	    call_flag++;

	    /*
	    // write point clouds out to file
	    pcl::io::savePCDFileASCII ("test1_cloud.pcd", *cloud);
	    */

	    // pass.setInputCloud(cloud);
	    // pass.setFilterFieldName("x");
	    // pass.setFilterLimits(-.80, 0.80);
	    // pass.filter(*cloud_filtered_x);

	    // pass.setInputCloud(cloud_filtered_x);
	    // pass.setFilterFieldName("y");
	    // pass.setFilterLimits(-5.0, 1.00);
	    // pass.filter(*cloud_filtered_y);

	    // pass.setInputCloud(cloud_filtered_y);
	    // pass.setFilterFieldName("z");
	    // pass.setFilterLimits(0, 3.25);
	    // pass.filter(*cloud_filtered_z);

	    // pcl::toROSMsg(*cloud_filtered_z, *object2_cloud);
	    // cloud_pub[1].publish(object2_cloud);
	}
};


//---------------------------------------------------------------------------
// Main
//---------------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_tracker");
  ros::NodeHandle n;

  ROS_INFO("Starting Object Tracker...\n");
  ObjectTracker tracker;
  
  ros::spin();
  
  return 0;
}

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

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>


//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------

bool gotdata = false;
int datacount = 100;
typedef pcl::PointXYZ PointT;
float xpos_last = 0.0;
float ypos_last = 0.0;
float zpos_last = 0.0;
bool locate = true;
bool lost = false;
ros::Publisher point_pub;


//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------

class BlobDetector {

private:
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  ros::Publisher cloudpub_[2];

public:
  BlobDetector() {
    sub_=n_.subscribe("/camera/depth/points", 1, &BlobDetector::cloudcb, this);
    cloudpub_[0] = n_.advertise<sensor_msgs::PointCloud2> ("object1_cloud", 1);
    cloudpub_[1] = n_.advertise<sensor_msgs::PointCloud2> ("object2_cloud", 1);
  }

  // this function gets called every time new pcl data comes in
  void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan) {
    sensor_msgs::PointCloud2::Ptr
		object1_cloud (new sensor_msgs::PointCloud2 ()),
		object2_cloud (new sensor_msgs::PointCloud2 ());    

    pcl::PointCloud<pcl::PointXYZ>::Ptr
		cloud (new pcl::PointCloud<pcl::PointXYZ> ()),
		cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZ> ()),
		cloud_filtered_y (new pcl::PointCloud<pcl::PointXYZ> ()),
		cloud_filtered_z (new pcl::PointCloud<pcl::PointXYZ> ());
		    

    // convert pcl2 message to pcl object
    pcl::fromROSMsg(*scan,*cloud);
	
    Eigen::Vector4f centroid;
    float xpos = 0.0;
    float ypos = 0.0;
    float zpos = 0.0;
    float D_sphere = 0.05; //meters
    float R_search = 2.0*D_sphere;

    geometry_msgs::Point point;

    // pass through filter
    pcl::PassThrough<pcl::PointXYZ> pass;

    // Now 
    if (locate == true)
    {
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("x");
		pass.setFilterLimits(-2.0, 2.0);
		pass.filter(*cloud_filtered_x);

		pass.setInputCloud(cloud_filtered_x);
		pass.setFilterFieldName("y");
		pass.setFilterLimits(-0.5, 0.98);
		pass.filter(*cloud_filtered_y);

		pass.setInputCloud(cloud_filtered_y);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(.75, 3.0);
		pass.filter(*cloud_filtered_z);

		pcl::compute3DCentroid(*cloud_filtered_z, centroid);
		xpos = centroid(0);
		ypos = centroid(1);
		zpos = centroid(2);

		// Publish cloud:
		pcl::toROSMsg(*cloud_filtered_z, *object2_cloud);
		cloudpub_[1].publish(object2_cloud);
		
		if(cloud_filtered_z->points.size() > 10)
		{
			// Then we have re-found the object!
			locate = false;
		}
    }
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

		if(cloud_filtered_z->points.size() < 10) {
			locate = true;
			lost = true;
			if(lost == true) {
				ROS_INFO ("We lost the object!");
				printf("x = %f  y = %f  z = %f\n",xpos_last,ypos_last,zpos_last);
			}
			
			return;
		}
	
		pcl::compute3DCentroid(*cloud_filtered_z, centroid);
		xpos = centroid(0);
		ypos = centroid(1);
		zpos = centroid(2);
		
		// Publish cloud:
		pcl::toROSMsg(*cloud_filtered_z, *object1_cloud);
		cloudpub_[0].publish(object1_cloud);

		tf::Transform transform;
		transform.setOrigin(tf::Vector3(xpos, ypos, zpos));
		transform.setRotation(tf::Quaternion(0, 0, 0, 1));

		static tf::TransformBroadcaster br;
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "openni_depth_optical_frame", "object1"));

		// set point message values and publish
		point.x = xpos;
		point.y = ypos;
		point.z = zpos;
    
		ros::Time tstamp = ros::Time::now();
		point_pub.publish(point);

    }

    xpos_last = xpos;
    ypos_last = ypos;
    zpos_last = zpos;

   
    /*
    // write point clouds out to file
    pcl::io::savePCDFileASCII ("test1_cloud.pcd", *cloud);
    */
  }
};


//---------------------------------------------------------------------------
// Main
//---------------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nu_objecttracker");
  ros::NodeHandle n;
  point_pub = n.advertise<geometry_msgs::Point> ("object1_position", 100);
  printf("Starting Program...\n");
  BlobDetector detector;
  ros::spin();
  return 0;
}

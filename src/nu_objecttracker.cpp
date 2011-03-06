// nu_objecttracker.cpp
// Jake Ware and Jarvis Schultz
// Winter 2011

//---------------------------------------------------------------------------
// Notes
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------

#include <iostream>

#include <ros/ros.h>
#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/feature.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>


//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------

bool gotdata = false;
int datacount = 100;


//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------

class BlobDetector {

private:
  ros::NodeHandle n_;
  ros::Subscriber sub_;

public:
  BlobDetector() {
    sub_=n_.subscribe("/camera/depth/points", 1, &BlobDetector::cloudcb, this);
  }

  // this function gets called every time new pcl data comes in
  void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan) {
    sensor_msgs::PointCloud2::Ptr 
      cloud_voxel_pcl2 (new sensor_msgs::PointCloud2 ());

    // voxel filter
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
    sor.setInputCloud (scan);
    sor.setLeafSize (0.02, 0.02, 0.02);
    sor.filter (*cloud_voxel_pcl2);   

    pcl::PointCloud<pcl::PointXYZ>::Ptr
      cloud (new pcl::PointCloud<pcl::PointXYZ> ()),
      cloud_voxel (new pcl::PointCloud<pcl::PointXYZ> ()),
      cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZ> ()),
      cloud_filtered_y (new pcl::PointCloud<pcl::PointXYZ> ()),
      cloud_filtered_z (new pcl::PointCloud<pcl::PointXYZ> ());

    pcl::fromROSMsg(*scan,*cloud);
    pcl::fromROSMsg(*cloud_voxel_pcl2,*cloud_voxel);

    datacount++;    
    //printf("%i\n", datacount);

    //if(datacount%100 == 0) {
      //printf("Beginning analysis...\n");
      Eigen::Vector4f centroid;
      float xpos = 0.0;
      float ypos = 0.0;
      float zpos = 0.0;

      //ros::Time tstart = ros::Time::now();
      //std::cout << "start time:  " << tstart << "\n ";

      // pass through filter
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(cloud_voxel);
      pass.setFilterFieldName("x");
      pass.setFilterLimits(-1.0, 1.0);
      pass.filter(*cloud_filtered_x);

      pass.setInputCloud(cloud_filtered_x);
      pass.setFilterFieldName("y");
      pass.setFilterLimits(-0.5, 0.5);
      pass.filter(*cloud_filtered_y);

      pass.setInputCloud(cloud_filtered_y);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(1, 1.8);
      pass.filter(*cloud_filtered_z);

      pcl::compute3DCentroid(*cloud_filtered_z, centroid);
      xpos = centroid(0);
      ypos = centroid(1);
      zpos = centroid(2);

      //ros::Time tstop = ros::Time::now();
      //std::cout << "finish time:  " << tstop << "\n ";

      printf("X: %f\n", xpos);
      //printf("Y: %f\n", ypos);
      //printf("Z: %f\n", zpos);

      //printf("Saving point cloud data...\n");

      // write point clouds out to file
      //pcl::io::savePCDFileASCII ("test1_cloud.pcd", *cloud);
      //pcl::io::savePCDFileASCII ("test1_cloud_voxel.pcd", *cloud_voxel);
      //pcl::io::savePCDFileASCII ("test1_cloud_filtered_x.pcd", *cloud_filtered_x);
      //pcl::io::savePCDFileASCII ("test1_cloud_filtered_y.pcd", *cloud_filtered_y);
      //pcl::io::savePCDFileASCII ("test1_cloud_filtered_z.pcd", *cloud_filtered_z);
      //printf("Complete\n");
      //}
  }
};


//---------------------------------------------------------------------------
// Main
//---------------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nu_objecttracker");
  ros::NodeHandle n;
  printf("Starting Program...\n");
  BlobDetector detector;
  ros::spin();
  return 0;
}

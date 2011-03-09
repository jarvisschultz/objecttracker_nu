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
#include <tf/transform_broadcaster.h>

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
      cloud_voxel_pcl2 (new sensor_msgs::PointCloud2 ()),
      object1_cloud (new sensor_msgs::PointCloud2 ()),
      object2_cloud (new sensor_msgs::PointCloud2 ());    

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
    

    // convert pcl2 message to pcl object
    pcl::fromROSMsg(*scan,*cloud);  
    // pcl::fromROSMsg(*cloud_voxel_pcl2,*cloud_voxel);

    datacount++;  // we got a new set of data so increment datacount
    //printf("%i\n", datacount);

    //printf("Beginning analysis...\n");
    Eigen::Vector4f centroid;
    float xpos = 0.0;
    float ypos = 0.0;
    float zpos = 0.0;

    // ros::Time tstart = ros::Time::now();
    // std::cout << "start time:  " << tstart << "\n ";

    // pass through filter
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-1.0, 1.0);
    pass.filter(*cloud_filtered_x);

    pass.setInputCloud(cloud_filtered_x);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.5, 0.5);
    pass.filter(*cloud_filtered_y);

    pass.setInputCloud(cloud_filtered_y);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(.75, 2.5);
    pass.filter(*cloud_filtered_z);

    // Search for a sphere:
    // Declare variables:
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT> ());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
    pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers_sphere (new pcl::PointIndices ());
    pcl::PointCloud<PointT>::Ptr cloud_sphere (new pcl::PointCloud<PointT> ());

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered_z);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Perform normal optimization:
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_SPHERE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setRadiusLimits (0.01,0.2);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setInputCloud (cloud_filtered_z);
    seg.setInputNormals (cloud_normals);
    // Obtain the sphere inliers and coefficients
    seg.segment (*inliers_sphere, *coefficients_sphere);
   
    // Extract the sphere inliers from the input cloud
    extract.setInputCloud (cloud_filtered_z);
    extract.setIndices (inliers_sphere);
    extract.setNegative (false);

    // Convert the extracted sphere object to a new point cloud
    extract.filter (*cloud_sphere);

    // Get XYZ data:
    // xpos = coefficients_sphere.x();
    // ypos = coefficients_sphere.y();
    // zpos = coefficients_sphere.z();

    // Publish cloud:
    pcl::toROSMsg(*cloud_sphere, *object1_cloud);
    cloudpub_[0].publish(object1_cloud);

    pcl::compute3DCentroid(*cloud_sphere, centroid);
    xpos = centroid(0);
    ypos = centroid(1);
    zpos = centroid(2); 

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(xpos, ypos, zpos));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));

    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "openni_depth_optical_frame", "object1"));

    // ros::Time tstop = ros::Time::now();
    // std::cout << "finish time:  " << tstop << "\n ";

    /*
    printf("X: %f\n", xpos);
    printf("Y: %f\n", ypos);
    printf("Z: %f\n", zpos);
    */

    /*
    if(datacount%100 == 0) {
    printf("Saving point cloud data...\n");
    
    // write point clouds out to file
    pcl::io::savePCDFileASCII ("test1_cloud.pcd", *cloud);
    pcl::io::savePCDFileASCII ("test1_cloud_voxel.pcd", *cloud_voxel);
    pcl::io::savePCDFileASCII ("test1_cloud_filtered_x.pcd", *cloud_filtered_x);
    pcl::io::savePCDFileASCII ("test1_cloud_filtered_y.pcd", *cloud_filtered_y);
    pcl::io::savePCDFileASCII ("test1_cloud_filtered_z.pcd", *cloud_filtered_z);
    printf("Complete\n");
    }
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
  printf("Starting Program...\n");
  BlobDetector detector;
  ros::spin();
  return 0;
}

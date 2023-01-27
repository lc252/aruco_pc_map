#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>


ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZ>);


void passthrough_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input)
{
    // Create and execute filter
    pcl::PassThrough<pcl::PointXYZ> pass;
    // x
    pass.setInputCloud(input);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.5, 0.5);
    pass.filter(*input);
    // y
    pass.setInputCloud(input);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.5, 0.5);
    pass.filter(*input);
    // z
    pass.setInputCloud(input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.5, 0.5);
    pass.filter(*input);
}

void voxel_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input)
{
    // Create and execute filter
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(input);
    sor.setLeafSize (0.005, 0.005, 0.005);  // cube length m
    sor.filter(*input);
}

void aggregate_cloud(sensor_msgs::PointCloud2 cloud_in)
{
    pcl::PointCloud<pcl::PointXYZ> new_cloud;
    pcl::fromROSMsg(cloud_in, new_cloud);

    // literally add lol
    *cloud_map += new_cloud;
    std::cout << "added" << std::endl;

    // filter aggregated cloud
    /* Filters
    Passthrough:
            - reduce the search size to the region of interest
            - ignore points in the far field with poorer accuracy
    Voxel:
            - downsample the number of points for processing
            - averaging samples in space reduces noise
            - average overlapping points from multiple scans
            - voxel size should be close to the sensors resolution
            so that precision is maintained but overlapping points
            are averaged.
    Outlier removal:
            - radial outliers
            - statistical outliers
            - possible but maybe not necessary
    */

    voxel_filter(cloud_map);
    passthrough_filter(cloud_map);

    sensor_msgs::PointCloud2 ros_cloud_map;
    pcl::toROSMsg(*cloud_map, ros_cloud_map);
    ros_cloud_map.header.frame_id = "map";
    pub.publish(ros_cloud_map);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_clouds");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("transformed_cloud", 1, aggregate_cloud);
    pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_map", 1);

    ros::spin();
}
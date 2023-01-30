#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>


ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZ>);


void cube_passthrough_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input, float bounds)
{
    // filter out points outside of a cubic ROI

    // Create and execute filter
    pcl::PassThrough<pcl::PointXYZ> pass;
    // x
    pass.setInputCloud(input);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-bounds, bounds);
    pass.filter(*input);
    // y
    pass.setInputCloud(input);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-bounds, bounds);
    pass.filter(*input);
    // z
    pass.setInputCloud(input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-bounds, bounds);
    pass.filter(*input);
}

void voxel_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input, float leaf_size)
{
    // 
    // Create and execute filter
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(input);
    sor.setLeafSize (leaf_size, leaf_size, leaf_size);  // cube length m
    sor.filter(*input);
}

void segment_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::PointCloud<pcl::PointXYZ>::Ptr &output)
{
    // return the largest planar segment of the cloud (ground plane)
    
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>()), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);

    seg.setInputCloud(input);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
        PCL_WARN("Planar segment not found...");
        return;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    extractor.setInputCloud(input);
    extractor.setIndices(inliers);
    
    // planar component is separated

    // planar inliers are stored to the output cloud
    extractor.setNegative(false);
    extractor.filter(*output);
    // planar ourliers are stored to the input cloud
    extractor.setNegative(true);
    extractor.filter(*input);
}

std::vector<pcl::PointIndices> get_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr &input)
{
    // Euclidean cluster extraction. Return the indices of cluster points

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02); // units m
    ec.setMinClusterSize(100); // points
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input);
    ec.extract(cluster_indices);

    return cluster_indices;
}


void add_cloud_to_map(sensor_msgs::PointCloud2 cloud_in)
{
    // add the new cloud to the map after some filtering

    // get pcl from ROS
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud;
    pcl::fromROSMsg(cloud_in, *new_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;

    /* Filtering Strategy
    Incoming cloud:
        - Passthrough the ROI
        - Segment the largest plane
            - Voxel the planar segment heavily
        - Extract clusters from non-planar component
            - Reject small clusters
            - Voxel lightly
    
    Add to map.

    Map:
        - Not yet determined.
    */


    // get the base plane, remove from new_cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane;
    segment_plane(new_cloud, plane);
    // filter the base plane
    voxel_filter(plane, 0.02);

    // cluster indices of new_cloud
    std::vector<pcl::PointIndices> cluster_indices;
    cluster_indices = get_clusters(new_cloud);
    // iterate through the cluster indicies
    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        // iterate through points of the cluster index
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            // add them to the current cluster
            cloud_cluster->push_back((*new_cloud)[*pit]);
        }
        // format cloud
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // voxel cloud cluster finely
        voxel_filter(cloud_cluster, 0.005);

        // add cluster back to cloud
        *filtered_cloud += *cloud_cluster;
    }

    // add filtered plane back to cloud
    *filtered_cloud += *plane;


    // literally add lol
    *cloud_map += *filtered_cloud;
    std::cout << "added" << std::endl;

    // filter map
    // voxel_filter(cloud_map, 0.005);
    // cube_passthrough_filter(cloud_map, 0.5);

    sensor_msgs::PointCloud2 ros_cloud_map;
    pcl::toROSMsg(*cloud_map, ros_cloud_map);
    ros_cloud_map.header.frame_id = "map";
    pub.publish(ros_cloud_map);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_clouds");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("transformed_cloud", 1, add_cloud_to_map);
    pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_map", 1);

    ros::spin();
}
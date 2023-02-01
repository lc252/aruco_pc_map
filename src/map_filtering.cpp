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
#include <pcl/conversions.h>
#include <eigen3/Eigen/Eigen>
#include <pcl/common/common.h>


ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZ>);


void cubic_roi_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input, float bounds)
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

void custom_roi_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::PointCloud<pcl::PointXYZ>::Ptr &output, Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt)
{
    // Return the part of the cloud in the ROI

    // Create and execute filter
    pcl::PassThrough<pcl::PointXYZ> pass;
    // x
    pass.setInputCloud(input);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(min_pt.x(), max_pt.x());
    pass.filter(*output);
    // y
    pass.setInputCloud(output);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(min_pt.y(), max_pt.y());
    pass.filter(*output);
    // z
    pass.setInputCloud(output);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_pt.z(), max_pt.z());
    pass.filter(*output);
}

void voxel_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input, float leaf_size)
{
    // 
    // Create and execute filter
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(input);
    sor.setLeafSize (leaf_size, leaf_size, leaf_size);  // cube length m
    /* setMinPointsPerVoxel
    by setting min points per voxel to 3, I 
    guarantee that ghost points from previous 
    scans where an object no longer exists will be lost 
    */
    // sor.setMinimumPointsNumberPerVoxel(3);
    sor.filter(*input);
}

void segment_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::PointCloud<pcl::PointXYZ>::Ptr &output)
{
    // split the input into a planar segment (output) and a clusters segment (input)
    
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>()), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    seg.setOptimizeCoefficients(false);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);        // more iterations = more likely to find largest plane, with a downsampled cloud it is more likely
    seg.setDistanceThreshold(0.0025); // tolerance on plane (mm)

    seg.setInputCloud(input);
    // finds the largest planar segment
    seg.segment(*inliers, *coefficients);   // returns the inliers of largest plane found
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
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.015); // units m
    ec.setMinClusterSize(100); // points
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input);
    ec.extract(cluster_indices);

    return cluster_indices;
}


void add_cloud_to_map(sensor_msgs::PointCloud2 cloud_in)
{
    // add the new cloud to the map and filter
    /* Filtering Strategy
    Incoming cloud:
        - Passthrough the ROI
    
    Add to map.

    Map:
        - Segment the largest plane
            - Voxel the planar segment heavily
        - Copy the cluster segment
            - Voxel heavily
            - For each cluster in copied clusters
                - Find the x, y, z bounds
                - Reject volumes smaller than ? 
        - Extract clusters from original cluster segment using x, y, z bounds + tolerance
            - Voxel lightly, maintain resolution, remove duplicates
    */


    // get pcl from ROS
    pcl::PointCloud<pcl::PointXYZ>::Ptr incoming_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_in, *incoming_cloud);
    // container for filtered clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_copy(new pcl::PointCloud<pcl::PointXYZ>);
    std::cout << "message converted" << std::endl;

    // ROI filter the incoming cloud
    cubic_roi_filter(incoming_cloud, 0.5);
    std::cout << "roi filter" << std::endl;

    // literally add to the map lol
    *cloud_map += *incoming_cloud;
    std::cout << "added to map" << std::endl;
    // populate the copy with data from the map
    *map_copy += *cloud_map;
    std::cout << "populated copy" << std::endl;
    // downsample copy
    voxel_filter(map_copy, 0.01);

    // segment the plane from the clusters
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
    segment_plane(map_copy, plane);

    // find clusters in the copy
    std::cout << "getting clusters in downsampled copy" << std::endl;
    std::vector<pcl::PointIndices> cluster_indices;
    cluster_indices = get_clusters(map_copy);
    std::cout << "got clusters" << std::endl;
    // iterate through the cluster indicies

    pcl::PointCloud<pcl::PointXYZ>::Ptr test(new pcl::PointCloud<pcl::PointXYZ>); // debugging
    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr hd_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        // iterate through points of the cluster index
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            // add them to the current cluster
            cloud_cluster->push_back((*map_copy)[*pit]);
        }
        // format cloud
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // spatial bounds of the cluster
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);
        // add a tolerance equal to the voxel leaf size. This ensures any corners lost in downsampling are kept when extracting the full feature.
        Eigen::Vector4f tolerance{0.01, 0.01, 0.01, 0.01};
        min_pt -= tolerance;
        max_pt += tolerance;
        // extract the full resolution cluster
        custom_roi_filter(cloud_map, hd_cluster, min_pt, max_pt);
        *test += *hd_cluster;
    }

    std::cout << "finished iterating clusters" << std::endl;

    sensor_msgs::PointCloud2 ros_cloud_map;
    pcl::toROSMsg(*test, ros_cloud_map);
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
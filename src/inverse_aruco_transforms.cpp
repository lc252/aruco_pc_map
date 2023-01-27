// Adapted from Shakif Turjo's hear_fiducial.py
// take fiducial transforms and apply them to the point cloud.


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Transform.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Eigen>

ros::Publisher pub;

void inverse_transform(fiducial_msgs::FiducialTransformArray msg)
{
    // compute the inverse transform from camera to fiducial

    // first find the transform with the lowest error
    fiducial_msgs::FiducialTransform best_tf;
    best_tf.image_error = 999;
    for(fiducial_msgs::FiducialTransform transform : msg.transforms)
    {
        if(transform.image_error < best_tf.image_error)
        {
            best_tf = transform;
        }
    }
    if(best_tf.fiducial_id == 0)
    {
        return;
    }

    // extract quaternion and translation vector from the received fiducial_msgs::Fiducialransform
    Eigen::Quaternionf quat(
        best_tf.transform.rotation.w, 
        best_tf.transform.rotation.x, 
        best_tf.transform.rotation.y, 
        best_tf.transform.rotation.z);
    Eigen::Vector3f tran(
        best_tf.transform.translation.x, 
        best_tf.transform.translation.y, 
        best_tf.transform.translation.z);

    // Build transformation matrix
    // identity matrix
    Eigen::Matrix4f tf_matrix = Eigen::Matrix4f::Identity();
    // insert rotation
    tf_matrix.block<3,3>(0,0) = quat.toRotationMatrix();
    // insert translation
    tf_matrix.block<3,1>(0,3) = tran;

    // inverse transformation matrix
    Eigen::Matrix4f inv_tf = tf_matrix.inverse();


    // extract quaternion and translation vector from the optical_color_frame to camera_link tf
    // tf listener
    tf::TransformListener tf_listener;
    // tf container
    tf::StampedTransform optical_to_link_tf;
    // lookup optical->camera
    /*
    tf_listener.lookupTransform("/camera_link", "/camera_color_optical_frame", ros::Time(0), optical_to_link_tf);
    // assign to eigen variables
    tf::Vector3 static_t = optical_to_link_tf.getOrigin();
    tf::Quaternion static_q = optical_to_link_tf.getRotation();
    Eigen::Vector3f st(static_t.x(), static_t.y(), static_t.z());
    Eigen::Quaternionf sq(static_q.w(), static_q.x(), static_q.y(), static_q.z());
    */

    // do it manually because the above is throwing errors and id rather get it funcitonal for now
    Eigen::Vector3f st(0.015, 0, 0);
    Eigen::Quaternionf sq(0.496, 0.506, -0.495, 0.502);
   

    // build transforation matrix
    // identity matrix
    Eigen::Matrix4f static_tf = Eigen::Matrix4f::Identity();
    // insert rotation
    static_tf.block<3,3>(0,0) = sq.toRotationMatrix();
    // insert translation
    static_tf.block<3,1>(0,3) = st;
    

    //  result: (aruco->optical)^-1 * (optical->camera)
    Eigen::Matrix4f result;
    result = inv_tf * static_tf;


    // result matrix
    Eigen::Quaternionf res_quat(result.block<3,3>(0,0));
    Eigen::Vector3f res_tran(result.block<3,1>(0,3));

    // Broadcaster
    static tf::TransformBroadcaster br;

    // tf to be broadcasted
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = "aruco_" + std::to_string(best_tf.fiducial_id);
    tf_msg.child_frame_id = "camera_link";
    // fill rotation values from Eigen
    tf_msg.transform.rotation.x = res_quat.x();
    tf_msg.transform.rotation.y = res_quat.y();
    tf_msg.transform.rotation.z = res_quat.z();
    tf_msg.transform.rotation.w = res_quat.w();
    // fill translation values from Eigen
    tf_msg.transform.translation.x = res_tran.x();
    tf_msg.transform.translation.y = res_tran.y();
    tf_msg.transform.translation.z = res_tran.z();

    br.sendTransform(tf_msg);

    return;

}

// stub
void transform_callback(fiducial_msgs::FiducialTransformArray msg)
{
    // Ask for the inverse transform directly rather than manually constructing it
    // An alternative function to inverse_transform 
    ;
}

// stub class
/*
class ArucoTransformer
{
public:
    ArucoTransformer()
    {
        ;
    }
private:
    // When a tf is published, callback to create the transform
    message_filters::Subscriber<tf::Transform> tf_sub;

    void transform_callback()
    {
        ;
    }
};
*/


int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_cloud");
    ros::NodeHandle nh;

    ros::Rate rate(30);

    // When a fiducial transform is published, callback to inverse it
    ros::Subscriber cloud_sub = nh.subscribe("fiducial_transforms", 1, inverse_transform);

    // // When a tf is published, callback to create the transform
    // message_filters::Subscriber<tf::Transform> tf_sub;

    ros::spin();
        
    return 0;
}


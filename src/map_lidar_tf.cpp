#include <ros/ros.h>
#include <tf/tf.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Core>


class Map_lidar_tf
{
public:
    Map_lidar_tf();
    void tf_sensor_to_map(ros::Time sensor_time, std::string sensor_frame);
    void tf_map_to_sensor(ros::Time sensor_time, std::string sensor_frame);
    void tf_quat_to_rpy(double& roll, double& pitch, double& yaw, tf::Quaternion quat);
    tf::StampedTransform transform;
    tf::StampedTransform transform_re;
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber config_sub_;
    ros::Subscriber sensor_points_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber tf_sub_;
    ros::Publisher match_points_pub_;
    ros::Publisher unmatch_points_pub_;

    tf::TransformListener tf_listener_;

    pcl::KdTreeFLANN<pcl::PointXYZI> tree_;

    double distance_threshold_;
    double min_clipping_height_;
    double max_clipping_height_;
    double lidar_offset;

    std::string map_frame_;
    std::string sendto_frame_;

    Eigen::Matrix4f x_rotation_matrix;
    Eigen::Matrix4f y_rotation_matrix;
    Eigen::Matrix4f z_rotation_matrix;
    Eigen::Matrix4f origin_matrix;
    Eigen::Matrix4f x_rotation_matrix_re;
    Eigen::Matrix4f y_rotation_matrix_re;
    Eigen::Matrix4f z_rotation_matrix_re;
    Eigen::Matrix4f origin_matrix_re;

    void pointsMapCallback(const sensor_msgs::PointCloud2::ConstPtr& map_cloud_msg_ptr);
    void sensorPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& sensorTF_cloud_msg_ptr);
    void searchMatchingCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr match_cloud_ptr,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr unmatch_cloud_ptr);
};

Map_lidar_tf::Map_lidar_tf()
    : nh_()
    , nh_private_("~")
    , distance_threshold_(0.4)
    , min_clipping_height_(-1.5)
    , max_clipping_height_(1.5)
    , lidar_offset(0.0) //0.93
    , map_frame_("/map")
    , sendto_frame_("/rslidar")
{
    nh_private_.param("distance_threshold", distance_threshold_, distance_threshold_);
    nh_private_.param("min_clipping_height", min_clipping_height_, min_clipping_height_);
    nh_private_.param("max_clipping_height", max_clipping_height_, max_clipping_height_);

    sensor_points_sub_ = nh_.subscribe("/filtered_points", 1, &Map_lidar_tf::sensorPointsCallback, this);
    map_sub_ = nh_.subscribe("/points_map", 10, &Map_lidar_tf::pointsMapCallback, this);
    
    match_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_ground", 10);
    unmatch_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_no_ground", 10);
    // min_clipping_height_ = 0.1;

}


void Map_lidar_tf::tf_sensor_to_map(const ros::Time sensor_time, std::string sensor_frame)
{
    tf_listener_.waitForTransform("/map", sensor_frame, sensor_time, ros::Duration(3.0));
    try{
        tf_listener_.lookupTransform("/map", sensor_frame, sensor_time, transform);
    
    }catch (tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        return;
    }

    std::cout << "Origin x : " << transform.getOrigin().x() << " ";
    std::cout << "Origin y : " << transform.getOrigin().y() << " ";
    std::cout << "Origin z : " << transform.getOrigin().z() << "\n";

    std::cout << "Rotation x : " << transform.getRotation().x() << " ";
    std::cout << "Rotation y : " << transform.getRotation().y() << " ";
    std::cout << "Rotation z : " << transform.getRotation().z() << " ";
    std::cout << "Rotation w : " << transform.getRotation().w() << "\n\n" << std::endl;

    tf::Quaternion tf_quaternion = transform.getRotation();

    double roll;
    double pitch;
    double yaw;

    tf_quat_to_rpy(roll, pitch, yaw, tf_quaternion);

    std::cout << "roll (x) : " << roll << " ";
    std::cout << "pitch (y) : " << pitch << " ";
    std::cout << "yaw (z) : " << yaw << "\n\n" << std::endl;

    x_rotation_matrix << \
    1, 0, 0, 0,\
    0, cos(roll), -sin(roll), 0,\ 
    0, sin(roll),  cos(roll), 0,\
    0, 0, 0, 1;

    y_rotation_matrix << \
     cos(pitch), 0,  sin(pitch), 0,\ 
     0         , 1,  0, 0,\
    -sin(pitch), 0,  cos(pitch), 0,\
     0                               , 0,  0                               , 1;
    
    z_rotation_matrix << \
     cos(yaw), -sin(yaw), 0, 0,\
     sin(yaw), cos(yaw), 0, 0,\
     0, 0, 1, 0,\ 
     0, 0, 0,1;

    origin_matrix << \
     1, 0 , 0, transform.getOrigin().x(),\ 
     0, 1 , 0, transform.getOrigin().y(),\
     0, 0 , 1, transform.getOrigin().z(),\
     0, 0 , 0, 1;
    // origin_matrix << \
    //  1, 0 , 0, 0,\ 
    //  0, 1 , 0, 0,\
    //  0, 0 , 1, 0,\
    //  0, 0 , 0, 1;
}

void Map_lidar_tf::tf_map_to_sensor(const ros::Time sensor_time, std::string sensor_frame)
{
    tf_listener_.waitForTransform(sensor_frame, "/map", sensor_time, ros::Duration(3.0));
    try{
        tf_listener_.lookupTransform(sensor_frame, "/map", sensor_time, transform_re);
    
    }catch (tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        return;
    }

    std::cout << "Origin x : " << transform_re.getOrigin().x() << " ";
    std::cout << "Origin y : " << transform_re.getOrigin().y() << " ";
    std::cout << "Origin z : " << transform_re.getOrigin().z() << "\n";

    std::cout << "Rotation x : " << transform_re.getRotation().x() << " ";
    std::cout << "Rotation y : " << transform_re.getRotation().y() << " ";
    std::cout << "Rotation z : " << transform_re.getRotation().z() << " ";
    std::cout << "Rotation w : " << transform_re.getRotation().w() << "\n\n" << std::endl;

    tf::Quaternion tf_quaternion_re = transform_re.getRotation();

    double roll_re;
    double pitch_re;
    double yaw_re;

    tf_quat_to_rpy(roll_re, pitch_re, yaw_re, tf_quaternion_re);

    std::cout << "roll (x) : " << roll_re << " ";
    std::cout << "pitch (y) : " << pitch_re << " ";
    std::cout << "yaw (z) : " << yaw_re << "\n\n" << std::endl;

    x_rotation_matrix_re << \
    1, 0, 0, 0,\
    0, cos(roll_re), sin(roll_re), 0,\ 
    0, -sin(roll_re),  cos(roll_re), 0,\
    0, 0, 0, 1;

    y_rotation_matrix_re << \
     cos(pitch_re), 0,  -sin(pitch_re), 0,\ 
     0            , 1,  0            , 0,\
     sin(pitch_re), 0,  cos(pitch_re), 0,\
     0            , 0,  0            , 1;
    
    z_rotation_matrix_re << \
     cos(yaw_re), -sin(yaw_re), 0, 0,\
     sin(yaw_re), cos(yaw_re), 0, 0,\
     0, 0, 1, 0,\ 
     0, 0, 0, 1;

    origin_matrix_re << \
     1, 0 , 0, transform_re.getOrigin().x(),\ 
     0, 1 , 0, transform_re.getOrigin().y(),\
     0, 0 , 1, transform_re.getOrigin().z(),\
     0, 0 , 0, 1;
}

void Map_lidar_tf::tf_quat_to_rpy(double& roll, double& pitch, double& yaw, tf::Quaternion quat){
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}
void Map_lidar_tf::pointsMapCallback(const sensor_msgs::PointCloud2::ConstPtr& map_cloud_msg_ptr)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*map_cloud_msg_ptr, *map_cloud_ptr);
    tree_.setInputCloud(map_cloud_ptr);

    map_frame_ = map_cloud_msg_ptr->header.frame_id;
}

void Map_lidar_tf::sensorPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& sensorTF_cloud_msg_ptr)
{
    const ros::Time sensor_time = sensorTF_cloud_msg_ptr->header.stamp;
    const std::string sensor_frame = sensorTF_cloud_msg_ptr->header.frame_id;
    tf_sensor_to_map(sensor_time, sensor_frame);
    tf_map_to_sensor(sensor_time, sensor_frame);

    pcl::PointCloud<pcl::PointXYZI>::Ptr sensorTF_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*sensorTF_cloud_msg_ptr, *sensorTF_cloud_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr sensorTF_clipping_height_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    sensorTF_clipping_height_cloud_ptr->header = sensorTF_cloud_ptr->header;
    for (size_t i = 0; i < sensorTF_cloud_ptr->points.size(); ++i)
    {
        if (sensorTF_cloud_ptr->points[i].z > min_clipping_height_ &&
            sensorTF_cloud_ptr->points[i].z < max_clipping_height_)
        {
        sensorTF_clipping_height_cloud_ptr->points.push_back(sensorTF_cloud_ptr->points[i]);
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr mapTF_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    
    //Affine conversion
    // pcl::transformPointCloud(*sensorTF_clipping_height_cloud_ptr, *sensorTF_clipping_height_cloud_ptr, origin_matrix);
    pcl::transformPointCloud(*sensorTF_clipping_height_cloud_ptr, *sensorTF_clipping_height_cloud_ptr, z_rotation_matrix);
    pcl::transformPointCloud(*sensorTF_clipping_height_cloud_ptr, *sensorTF_clipping_height_cloud_ptr, y_rotation_matrix);
    // pcl::transformPointCloud(*sensorTF_clipping_height_cloud_ptr, *mapTF_cloud_ptr, z_rotation_matrix);
    pcl::transformPointCloud(*sensorTF_clipping_height_cloud_ptr, *sensorTF_clipping_height_cloud_ptr, x_rotation_matrix);
    pcl::transformPointCloud(*sensorTF_clipping_height_cloud_ptr, *mapTF_cloud_ptr, origin_matrix);

    // pcl_ros::transformPointCloud(*sensorTF_clipping_height_cloud_ptr, *mapTF_cloud_ptr, transform);

    pcl::PointCloud<pcl::PointXYZI>::Ptr mapTF_match_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr mapTF_unmatch_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    searchMatchingCloud(mapTF_cloud_ptr, mapTF_match_cloud_ptr, mapTF_unmatch_cloud_ptr);

    std::cout << "send to : " << sendto_frame_ << std::endl; 

    //Affine conversion
    // pcl::transformPointCloud(*mapTF_match_cloud_ptr, *mapTF_match_cloud_ptr, z_rotation_matrix_re);
    // pcl::transformPointCloud(*mapTF_match_cloud_ptr, *mapTF_match_cloud_ptr, y_rotation_matrix_re);
    // pcl::transformPointCloud(*mapTF_match_cloud_ptr, *mapTF_match_cloud_ptr, x_rotation_matrix_re);
    // pcl::transformPointCloud(*mapTF_match_cloud_ptr, *mapTF_match_cloud_ptr, origin_matrix_re);

    sensor_msgs::PointCloud2 mapTF_match_cloud_msg;
    pcl::toROSMsg(*mapTF_match_cloud_ptr, mapTF_match_cloud_msg);
    mapTF_match_cloud_msg.header.stamp = sensor_time;
    mapTF_match_cloud_msg.header.frame_id = sendto_frame_;
    mapTF_match_cloud_msg.fields = sensorTF_cloud_msg_ptr->fields;

    match_points_pub_.publish(mapTF_match_cloud_msg);

    pcl::transformPointCloud(*mapTF_unmatch_cloud_ptr, *mapTF_unmatch_cloud_ptr, z_rotation_matrix_re);
    pcl::transformPointCloud(*mapTF_unmatch_cloud_ptr, *mapTF_unmatch_cloud_ptr, y_rotation_matrix_re);
    pcl::transformPointCloud(*mapTF_unmatch_cloud_ptr, *mapTF_unmatch_cloud_ptr, x_rotation_matrix_re);
    pcl::transformPointCloud(*mapTF_unmatch_cloud_ptr, *mapTF_unmatch_cloud_ptr, origin_matrix_re);
    
    sensor_msgs::PointCloud2 mapTF_unmatch_cloud_msg;
    pcl::toROSMsg(*mapTF_unmatch_cloud_ptr, mapTF_unmatch_cloud_msg);
    mapTF_unmatch_cloud_msg.header.stamp = sensor_time;
    mapTF_unmatch_cloud_msg.header.frame_id = sendto_frame_;
    mapTF_unmatch_cloud_msg.fields = sensorTF_cloud_msg_ptr->fields;

    unmatch_points_pub_.publish(mapTF_unmatch_cloud_msg);
}

void Map_lidar_tf::searchMatchingCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                                           pcl::PointCloud<pcl::PointXYZI>::Ptr match_cloud_ptr,
                                           pcl::PointCloud<pcl::PointXYZI>::Ptr unmatch_cloud_ptr)
{
    match_cloud_ptr->points.clear();
    unmatch_cloud_ptr->points.clear();

    match_cloud_ptr->points.reserve(in_cloud_ptr->points.size());
    unmatch_cloud_ptr->points.reserve(in_cloud_ptr->points.size());

    std::vector<int> nn_indices(1);
    std::vector<float> nn_dists(1);
    const double squared_distance_threshold = distance_threshold_ * distance_threshold_;

    for (size_t i = 0; i < in_cloud_ptr->points.size(); ++i)
    {
        tree_.nearestKSearch(in_cloud_ptr->points[i], 1, nn_indices, nn_dists);
        if (nn_dists[0] <= squared_distance_threshold)
        {
            match_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
        }
        else
        {
            unmatch_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
        }

    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "compare_map_filter");
    Map_lidar_tf node;

    ros::Rate rate(10.0);
    while(ros::ok()){
        ros::spinOnce();
    }

    return 0;
}

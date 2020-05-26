
#include "process/lidar_process.h"


LidarProcess::LidarProcess(ros::NodeHandle node, std::string config_path){
    // Initialze Config Params
    Init(config_path);

    // Initialize visualization marker vector
    bbox_list_.header.frame_id = "0";
    bbox_list_.ns = "bbox line list";
    bbox_list_.action = visualization_msgs::Marker::ADD;
    bbox_list_.pose.orientation.w = 1.0;
    bbox_list_.id = 1;
    bbox_list_.type = visualization_msgs::Marker::LINE_LIST;
    bbox_list_.scale.x = 0.1;
    if (true) {
        bbox_list_.color.r = 1.0;
        bbox_list_.color.g = 0.0;
        bbox_list_.color.b = 0.0;
        bbox_list_.color.a = 1.0;
    }

    // Initialize Point Cloud Pointers
    filtered_cloud_ground_ptr_.reset(new pcl_util::VPointCloud);
    filtered_cloud_objects_ptr_.reset(new pcl_util::VPointCloud);
    filtered_cloud_ground_ptr_.reset(new pcl_util::VPointCloud);

    // Initialize Subcribers
    lidar_subscriber_ = node.subscribe("/livox/lidar_1HDDGAU00100091", 100, &LidarProcess::ProcessLidarData, this, ros::TransportHints().reliable().tcpNoDelay(true));

    // Initialize Publishers
    filtered_cloud_publisher_ = node.advertise<pcl_util::VPointCloud>("lidar_filtered", 1);
    filtered_cloud_objects_publisher_ = node.advertise<pcl_util::VPointCloud>("lidar_filtered_objects", 1);
    filtered_cloud_ground_publisher_ = node.advertise<pcl_util::VPointCloud>("lidar_filtered_ground", 1);
    bbox_publisher_ = node.advertise<visualization_msgs::Marker>("lidar_bbox_marker", 1);
    velocity_publisher_ = node.advertise<visualization_msgs::MarkerArray>("lidar_velocity_marker", 1);

}

LidarProcess::~LidarProcess(){

}

bool LidarProcess::Init(std::string &config_path) {
    std::ifstream file(config_path, std::ios::binary);
    if (!file.is_open()){
        std::cout << "Error opening file." << std::endl;
        return false;
    }

    Json::Value root;
    Json::Reader reader;
    if (reader.parse(file, root)){
        // roi_filter
        std::string roi_filter_config = "roi_filter";
        roi_filter_ = std::make_shared<ROIFilter>();
        if (!roi_filter_->Init(root, roi_filter_config)){
            ROS_WARN("Init roi_filter failed.");
        }
        ROS_INFO("Init successfully, roi_filter.");

        // calibrate
        std::string calibrate_config = "calibrate";
        calibrate_ = std::make_shared<Calibrate>();
        if (!calibrate_->Init(root, calibrate_config)){
            ROS_WARN("Init calibrate failed.");
        }
        ROS_INFO("Init successfully, calibrate.");

        // curb detect
        std::string curb_detect_config = "curb_detect";
        grid_map_ = std::make_shared<GridMap>();
        if (!grid_map_->Init(root, curb_detect_config)){
            ROS_WARN("Init curb detect failed.");
        }
        ROS_INFO("Init successfully, curb detect.");

        return true;
    }else{
        std::cout << "Parse error." << std::endl;
        return false;
    }
}

void LidarProcess::ProcessLidarData(const pcl_util::VPointCloudPtr &in_cloud_ptr) {
    filtered_cloud_ptr_.reset(new pcl_util::VPointCloud);
    filtered_cloud_objects_ptr_.reset(new pcl_util::VPointCloud);
    filtered_cloud_ground_ptr_.reset(new pcl_util::VPointCloud);

    filtered_cloud_ptr_->header.stamp = in_cloud_ptr->header.stamp;
    filtered_cloud_ptr_->header.frame_id = in_cloud_ptr->header.frame_id;
    filtered_cloud_ground_ptr_->header.stamp = in_cloud_ptr->header.stamp;
    filtered_cloud_ground_ptr_->header.frame_id = in_cloud_ptr->header.frame_id;
    filtered_cloud_objects_ptr_->header.stamp = in_cloud_ptr->header.stamp;
    filtered_cloud_objects_ptr_->header.frame_id = in_cloud_ptr->header.frame_id;

    bbox_list_.header.stamp = pcl_conversions::fromPCL(in_cloud_ptr->header.stamp);
    bbox_list_.points.clear();
    bbox_list_.colors.clear();
    velocity_list_.markers.clear();

    ProcessPointCloud(in_cloud_ptr);

    filtered_cloud_publisher_.publish(filtered_cloud_ptr_);
    filtered_cloud_objects_publisher_.publish(filtered_cloud_objects_ptr_);
    filtered_cloud_ground_publisher_.publish(filtered_cloud_ground_ptr_);
    bbox_publisher_.publish(bbox_list_);
    velocity_publisher_.publish(velocity_list_);

    return;
}

void LidarProcess::ProcessPointCloud(const pcl_util::VPointCloudPtr &in_cloud_ptr){
    pcl_util::VPointCloudPtr filter_cloud_all_ptr(new pcl_util::VPointCloud());
    roi_filter_->Filter(in_cloud_ptr, filter_cloud_all_ptr);


    grid_map_->ConstructGridMap(filter_cloud_all_ptr, filtered_cloud_ptr_);





}
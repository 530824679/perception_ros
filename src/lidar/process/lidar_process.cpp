
#include "process/lidar_process.h"


LidarProcess::LidarProcess(ros::NodeHandle node, std::string config_path){
    // Initialze Config Params
    Init(config_path);

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
    lidar_bbox_publisher_ = node.advertise<visualization_msgs::Marker>("lidar_bbox_marker", 1);
    lidar_velocity_publisher_ = node.advertise<visualization_msgs::MarkerArray>("lidar_velocity_marker", 1);

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

void LidarProcess::ProcessLidarData(const pcl_util::VPointCloudPtr in_cloud_ptr) {
    std::cout << "start call back" << std::endl;

    PointXYZ out_point_cloud_;
    out_point_cloud_.header.stamp = in_cloud_ptr->header.stamp;
    out_point_cloud_.header.frame_id = in_cloud_ptr->header.frame_id;


    // set the exact point cloud size -- the vectors should already have enough space
    size_t num_points = in_cloud_ptr->size();
    out_point_cloud_.points.resize(num_points);


    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    calibrate_round_.Correct(in_cloud_ptr, out_cloud_ptr);

    std::cout << out_cloud_ptr->size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    size_t result_count = 0;
    grid_map_.ConstructGridMap(out_cloud_ptr, object_cloud_ptr, result_count);
    //out_point_cloud_.resize(result_count);


    //基于半径的离群点剔除
    pcl::RadiusOutlierRemoval<pcl::PointXYZ>  rout;
    rout.setInputCloud(object_cloud_ptr);
    rout.setRadiusSearch(0.8);
    rout.setMinNeighborsInRadius(2);
    rout.filter(out_point_cloud_);




    out_publisher_.publish(out_point_cloud_);

    std::cout << "end call back" << std::endl;

}
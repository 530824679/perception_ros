
#include "process/lidar_process.h"

LidarProcess::LidarProcess(ros::NodeHandle node, std::string config_path){
    // Initialze Config Params
    Init(config_path);

    // Initialize visualization marker vector
    bbox_list_.header.frame_id = "livox_frame";
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
    filtered_cloud_ground_ptr_.reset(new pcl_util::PointCloud);
    filtered_cloud_objects_ptr_.reset(new pcl_util::PointCloud);
    filtered_cloud_ground_ptr_.reset(new pcl_util::PointCloud);

    // Initialize Subcribers
    lidar_subscriber_ = node.subscribe("/livox/lidar_1HDDGAU00100091", 100, &LidarProcess::ProcessLidarData, this, ros::TransportHints().reliable().tcpNoDelay(true));

    // Initialize Publishers
    filtered_cloud_publisher_ = node.advertise<pcl_util::PointCloud>("lidar_filtered", 1);
    filtered_cloud_objects_publisher_ = node.advertise<pcl_util::PointCloud>("lidar_filtered_objects", 1);
    filtered_cloud_ground_publisher_ = node.advertise<pcl_util::PointCloud>("lidar_filtered_ground", 1);
    bbox_publisher_ = node.advertise<visualization_msgs::Marker>("lidar_bbox_marker", 1);
    velocity_publisher_ = node.advertise<visualization_msgs::MarkerArray>("lidar_velocity_marker", 1);
    object_publisher_ = node.advertise<perception_ros::ObjectInfoArray>("lidar_object_marker", 1);

    viewer_ = CloudViewer();
}

LidarProcess::LidarProcess(std::string config_path){
    // Initialze Config Params
    Init(config_path);
    pcl_util::PointCloudPtr in_cloud_ptr(new pcl_util::PointCloud);
    if (pcl::io::loadPCDFile<pcl_util::Point>("/media/linuxidc/5d1fdaab-1568-48a9-8654-91a6a537a58f/dajiang/pcd/530.pcd", *in_cloud_ptr) == -1) {
        PCL_ERROR("PCD file reading failed.");
        return;
    }

    viewer_ = CloudViewer();
    ProcessLidarData(in_cloud_ptr);

}

LidarProcess::~LidarProcess(){

}

pcl_util::PCLVisualizerPtr LidarProcess::CloudViewer() {
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    angle_ = TopDown;
    render_.InitCamera(angle_, viewer);

    return viewer;
}

bool LidarProcess::Init(std::string &config_path) {
    std::ifstream file(config_path, std::ios::binary);
    if (!file.is_open()){
        logger.Log(ERROR, "[%s]: Error opening file.\n", __func__);
        return false;
    }

    Json::Value root;
    Json::Reader reader;
    if (reader.parse(file, root)){
        // roi_filter
        std::string roi_filter_config = "roi_filter";
        roi_filter_ = std::make_shared<ROIFilter>();
        if (!roi_filter_->Init(root, roi_filter_config)){
            logger.Log(WARNING, "[%s]: Init roi_filter failed.\n", __func__);
        }
        logger.Log(INFO, "[%s]: Init successfully, roi_filter.\n", __func__);

        // calibrate
        std::string calibrate_config = "calibrate";
        calibrate_ = std::make_shared<Calibrate>();
        if (!calibrate_->Init(root, calibrate_config)){
            logger.Log(WARNING, "[%s]: Init calibrate failed.\n", __func__);
        }
        logger.Log(INFO, "[%s]: Init successfully, calibrate.\n", __func__);

        // segmentation
        std::string segmentation_config = "segmentation";
        cluster_ = std::make_shared<Cluster>();
        if (!cluster_->Init(root, segmentation_config)){
            logger.Log(WARNING, "[%s]: Init segment object failed.\n", __func__);
        }
        logger.Log(INFO, "[%s]: Init successfully, segment object.\n", __func__);

        // object_builder
        std::string bbox_estimator_config = "bbox_estimator";
        bbox_estimator_  = std::make_shared<BBoxEstimator>();

        // object tracking
        std::string tracking_config = "tracking";
        tracking_  = std::make_shared<Tracking>();
        if (!tracking_->Init(root, tracking_config)){
            logger.Log(WARNING, "[%s]: Init tracking object failed.\n", __func__);
        }
        logger.Log(INFO, "[%s]: Init successfully, tracking object.\n", __func__);

        return true;
    }else{
        logger.Log(ERROR, "[%s]: Parse error.\n", __func__);
        return false;
    }
}

void LidarProcess::ProcessLidarData(const pcl_util::PointCloudPtr &in_cloud_ptr) {
    filtered_cloud_ptr_.reset(new pcl_util::PointCloud);
    filtered_cloud_objects_ptr_.reset(new pcl_util::PointCloud);
    filtered_cloud_ground_ptr_.reset(new pcl_util::PointCloud);

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




//    filtered_cloud_publisher_.publish(filtered_cloud_ptr_);
//    filtered_cloud_objects_publisher_.publish(filtered_cloud_objects_ptr_);
//    filtered_cloud_ground_publisher_.publish(filtered_cloud_ground_ptr_);
//    bbox_publisher_.publish(bbox_list_);
//    velocity_publisher_.publish(velocity_list_);

    return;
}

void LidarProcess::ProcessPointCloud(const pcl_util::PointCloudPtr &in_cloud_ptr){
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

    pcl_util::PointCloudPtr filter_cloud_all_ptr(new pcl_util::PointCloud());
    roi_filter_->Filter(in_cloud_ptr, filter_cloud_all_ptr);

    pcl_util::PointCloudPtr calibrate_cloud_all_ptr(new pcl_util::PointCloud());
    calibrate_->Correct(filter_cloud_all_ptr, calibrate_cloud_all_ptr);

    std::vector<pcl_util::PointCloud> cluster_cloud_vec;
    // TMP
    pcl_util::PointCloudPtr object_cloud_ptr(new pcl_util::PointCloud());
    cluster_->Process(filter_cloud_all_ptr, cluster_cloud_vec, object_cloud_ptr);

    std::vector<BBox> bboxes;
    std::vector<BBox2D> bbox2des;
    bbox_estimator_->Estimate(cluster_cloud_vec, bboxes,bbox2des);


    //tracking_->Process(bboxes, object_array_);

    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = end - start;
    std::cout << "Done! Took " << fp_ms.count() << "ms\n";











    // TMP Visualization
    viewer_->removeAllPointClouds();
    viewer_->removeAllShapes();


    for (int i = 0; i < cluster_cloud_vec.size(); ++i) {
        float random_r = rand() % 10 / (float)10.0;
        float random_g = rand() % 10 / (float)10.0;
        float random_b = rand() % 10 / (float)10.0;
        render_.RenderPointCloud(viewer_, cluster_cloud_vec[i].makeShared(), "PointCloud"+std::to_string(i), Color(random_r,random_g,random_b));
    }

    int clusterid = 0;
    for (size_t i = 0; i < bboxes.size(); i++) {
        render_.RenderBBox(viewer_, bboxes[i], clusterid, Color(0,1,0));
        clusterid++;
    }
    
    int clusterid2d = 0;
    for (size_t i = 0; i < bbox2des.size(); i++) {
        render_.RenderBBox2D(viewer_, bbox2des[i], clusterid2d, Color(0,0,1));
        clusterid2d++;
    }

    //viewer_->spinOnce();
    while (!viewer_->wasStopped()) {
        viewer_->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }
}
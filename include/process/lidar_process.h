/******************************************************************************/
/*!
File name: lidar_process.h

Description:
This file define class of lidar process to realize lidar perception module.

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef PERCEPTION_ROS_LIDAR_PROCESS_H
#define PERCEPTION_ROS_LIDAR_PROCESS_H

// ros include
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/package.h>

// pcl include
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/publisher.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//opencv include
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
// json include
#include <jsoncpp/json/json.h>

// egien include
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<Eigen/Dense>

// system include
#include <iostream>
#include <string.h>
#include <chrono>
#include <tf/transform_listener.h>
// local include
#include "roi_filter/roi_filter.h"
#include "calibration/calibrate.h"
#include "curb_detect/curb_detect.h"
#include "segmentation/object_cluster.h"
#include "object_builder/bbox_fitting.h"
#include "tracker/tracking.h"
#include "tracker/imm_ukf_pda.h"
#include "common/pcl_types.h"
#include "common/logging.h"
#include "render/render.h"
#include "objects_visualizer/visualize_tracked_objects.h"
// msg include
#include "perception_ros/ObjectInfoArray.h"
#include "perception_ros/DetectedObject.h"
#include "perception_ros/DetectedObjectArray.h"

extern Logging logger;

class LidarProcess {

public:
    LidarProcess(ros::NodeHandle node, std::string config_path);
    LidarProcess(std::string config_path);
    ~LidarProcess();

    bool Init(std::string &config_path);

    void ProcessLidarData(const pcl_util::PointCloudPtr &in_cloud_ptr);
    void ProcessLocalizationData(const tf::StampedTransform& local_to_global_);

private:
    void ProcessPointCloud(const pcl_util::PointCloudPtr &in_cloud_ptr);
    pcl_util::PCLVisualizerPtr CloudViewer();

private:
    // Sub module ptr
    std::shared_ptr<ROIFilter> roi_filter_;
    std::shared_ptr<Calibrate> calibrate_;
    std::shared_ptr<Cluster> cluster_;
    std::shared_ptr<BBoxEstimator> bbox_estimator_;
    std::shared_ptr<Tracking> tracking_; //kf_tracking_
    std::shared_ptr<ImmUkfPda> ukf_tracking_;//ukf_tracking_
    std::shared_ptr<VisualizeDetectedObjects> visualization_;

    // Point clouds
    pcl_util::PointCloudPtr filtered_cloud_ptr_;
    pcl_util::PointCloudPtr filtered_cloud_objects_ptr_;
    pcl_util::PointCloudPtr filtered_cloud_ground_ptr_;

    //get the local to global coordinate
    
    //tf::StampedTransform local_to_global_;

    // Markers for visualization
    visualization_msgs::Marker bbox_list_;
    visualization_msgs::MarkerArray velocity_list_;
    perception_ros::ObjectInfoArray object_array_;
    perception_ros::DetectedObjectArray detetcted_object_array_;
    perception_ros::ObjectInfoArray object_info_array_;

    // Subscriber
    ros::Subscriber lidar_subscriber_;
    ros::Subscriber localization_;
    // Data publisher
    ros::Publisher filtered_cloud_publisher_;
    ros::Publisher filtered_cloud_ground_publisher_;
    ros::Publisher filtered_cloud_objects_publisher_;
    ros::Publisher pub_object_array_;

    // Marker publisher
    ros::Publisher bbox_publisher_;
    ros::Publisher velocity_publisher_;
    ros::Publisher object_publisher_;

    //BoundingBoxArray publisher
    ros::Publisher bounding_box_tracked_;
    ros::Publisher bounding_box_detected_;


    // Visualize
    pcl_util::PCLVisualizerPtr viewer_;
    CameraAngle angle_;
    Render render_;
    VisualizeDetectedObjects visualize_objects_;

    std_msgs::Header input_header_;
};

#endif //PERCEPTION_ROS_LIDAR_PROCESS_H

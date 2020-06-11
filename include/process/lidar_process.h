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

// pcl include
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/publisher.h>
#include <pcl_ros/point_cloud.h>

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

// local include
#include "roi_filter/roi_filter.h"
#include "calibration/calibrate.h"
#include "curb_detect/curb_detect.h"
#include "segmentation/object_cluster.h"
#include "object_builder/bbox_fitting.h"
#include "common/pcl_types.h"
#include "common/logging.h"
#include "render/render.h"

// msg include
#include "perception_ros/ObjectInfoArray.h"

extern Logging logger;

class LidarProcess {

public:
    LidarProcess(ros::NodeHandle node, std::string config_path);
    LidarProcess(std::string config_path);
    ~LidarProcess();

    bool Init(std::string &config_path);

    void ProcessLidarData(const pcl_util::PointCloudPtr &in_cloud_ptr);

private:
    void ProcessPointCloud(const pcl_util::PointCloudPtr &in_cloud_ptr);
    pcl_util::PCLVisualizerPtr CloudViewer();

private:
    // Sub module ptr
    std::shared_ptr<ROIFilter> roi_filter_;
    std::shared_ptr<Calibrate> calibrate_;
    std::shared_ptr<Cluster> cluster_;
    std::shared_ptr<BBoxEstimator> bbox_estimator_;

    // Point clouds
    pcl_util::PointCloudPtr filtered_cloud_ptr_;
    pcl_util::PointCloudPtr filtered_cloud_objects_ptr_;
    pcl_util::PointCloudPtr filtered_cloud_ground_ptr_;

    // Markers for visualization
    visualization_msgs::Marker bbox_list_;
    visualization_msgs::MarkerArray velocity_list_;

    // Subscriber
    ros::Subscriber lidar_subscriber_;

    // Data publisher
    ros::Publisher filtered_cloud_publisher_;
    ros::Publisher filtered_cloud_ground_publisher_;
    ros::Publisher filtered_cloud_objects_publisher_;

    // Marker publisher
    ros::Publisher bbox_publisher_;
    ros::Publisher velocity_publisher_;

    // Visualize
    pcl_util::PCLVisualizerPtr viewer_;
    CameraAngle angle_;
    Render render_;
};




#endif //PERCEPTION_ROS_LIDAR_PROCESS_H

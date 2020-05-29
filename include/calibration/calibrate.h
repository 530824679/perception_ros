/******************************************************************************/
/*!
File name: calibrate.h

Description:
This file define class of calibrate ground to calibrate ground plane.

Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef PERCEPTION_ROS_CALIBRATE_H
#define PERCEPTION_ROS_CALIBRATE_H

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

// json include
#include <jsoncpp/json/json.h>

// local include
#include "common/logging.h"
#include "common/pcl_types.h"

extern Logging logger;

class Calibrate {
public:
    Calibrate();
    ~Calibrate();

    bool Init(Json::Value params, std::string key);
    void Correct(const pcl_util::PointCloudPtr in_cloud_ptr, pcl_util::PointCloudPtr out_cloud_ptr);
    
private:
    void FilterPointCloud(const pcl_util::PointCloudPtr in_cloud_ptr, pcl_util::PointCloudPtr out_cloud_ptr);
    Eigen::MatrixXf EstimateGroundPlane(const pcl_util::PointCloudPtr in_cloud_ptr, const float distance_threshold);
    Eigen::Matrix4f CreateRotateMatrix(Eigen::Vector3f before, Eigen::Vector3f after);

    float x_min_;
    float x_max_;
    float y_min_;
    float y_max_;
    float z_min_;
    float z_max_;
};


#endif //PERCEPTION_ROS_CALIBRATE_GROUND_H

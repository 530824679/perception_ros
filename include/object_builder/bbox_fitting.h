/******************************************************************************/
/*!
File name: bbox_fitting.h

Description:
This file define class of BBoxEstimator use to fitting the bounding box of cluster bundles.

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef PERCEPTION_ROS_BBOX_FITTING_H
#define PERCEPTION_ROS_BBOX_FITTING_H

// local include
#include "common/logging.h"
#include "common/pcl_types.h"
#include "perception_ros/DetectedObject.h"
#include "perception_ros/DetectedObjectArray.h"
#include <tf/transform_listener.h>

extern Logging logger;

class BBoxEstimator{
public:
    BBoxEstimator();
    ~BBoxEstimator();

    //void Estimate(std::vector<pcl_util::PointCloud> &clusters, std::vector<BBox> &bboxes);
    void Estimate(std::vector<pcl_util::PointCloud> &clusters, std::vector<BBox> &bboxes,std::vector<BBox2D> &bbox2des);
    void Estimate(std::vector<pcl_util::PointCloud> &clusters, perception_ros::DetectedObjectArray &detetcted_object_array);
private:
    bool SearchBasedFitting(pcl_util::PointCloudPtr &&in_cloud_ptr, BBox &box,BBox2D &box2d);
    bool SearchBasedFitting(pcl_util::PointCloudPtr &&in_cloud_ptr,perception_ros::DetectedObject &detected_object);

    float CalcCloseness(const std::vector<float> &C_1, const std::vector<float> &C_2);

    bool CalcBBox(pcl_util::PointCloudPtr &in_cloud_ptr, std::vector<std::pair<float, float>> &Q, float dz, BBox &box,BBox2D &box2d);
    bool CalcBBox(pcl_util::PointCloudPtr &in_cloud_ptr, std::vector<std::pair<float, float>> &Q, float dz, perception_ros::DetectedObject &detected_object);

    Eigen::Array3f CalcCloudCentroid(pcl_util::PointCloudPtr &in_cloud_ptr);
};



#endif //PERCEPTION_ROS_BBOX_FITTING_H

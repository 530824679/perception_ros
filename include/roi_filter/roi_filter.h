/******************************************************************************/
/*!
File name: roi_filter.h

Description:
This file define class of ROIFilter extract the region to be processed.

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef PERCEPTION_ROS_ROI_FILTER_H
#define PERCEPTION_ROS_ROI_FILTER_H

// json include
#include <jsoncpp/json/json.h>

// local include
#include "common/pcl_types.h"
#include "common/logging.h"

extern Logging logger;

class ROIFilter {
public:
    ROIFilter();
    ~ROIFilter();

    bool Init(Json::Value params, std::string key);

    bool Filter(const pcl_util::PointCloudPtr &in_cloud_ptr, pcl_util::PointCloudPtr &filtered_cloud_all_ptr);
    void FilterROI(const pcl_util::PointCloudPtr &in_cloud_ptr);

private:
    float roi_x_min_;
    float roi_x_max_;
    float roi_y_min_;
    float roi_y_max_;
    float roi_z_min_;
    float roi_z_max_;

    pcl_util::PointCloudPtr filtered_cloud_all_ptr_;


};

#endif //PERCEPTION_ROS_ROI_FILTER_H

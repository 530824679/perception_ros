//
/******************************************************************************/
/*!
File name: curb_detect.h

Description:
This file define class of CurbDetect to get the left and right curb points.

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef PERCEPTION_ROS_CURB_DETECT_H
#define PERCEPTION_ROS_CURB_DETECT_H

// local include
#include "segmentation/grid_map.h"
#include "common/logging.h"

extern Logging logger;

class CurbDetect{
public:
    CurbDetect();
    ~CurbDetect();

    bool Init(Json::Value params, std::string key);

    void Process(pcl_util::VPointCloudPtr &in_cloud_ptr, pcl_util::VPointCloudPtr &out_cloud_ptr);

private:

};

#endif //PERCEPTION_ROS_CURB_DETECT_H

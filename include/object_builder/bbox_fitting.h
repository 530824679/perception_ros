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

class BBoxEstimator{
public:
    BBoxEstimator();
    ~BBoxEstimator();

    bool Estimate(pcl_util::PointCloudPtr &in_cloud_ptr, pcl_util::BBox &box);

private:
    double CalcCloseness(const std::vector<double> &C_1, const std::vector<double> &C_2);

};



#endif //PERCEPTION_ROS_BBOX_FITTING_H

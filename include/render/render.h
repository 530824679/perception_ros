/******************************************************************************/
/*!
File name: render.h

Description:
This file define class of Render use to visualize point cloud.

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef PERCEPTION_ROS_RENDER_H
#define PERCEPTION_ROS_RENDER_H

// pcl include
#include <pcl/visualization/pcl_visualizer.h>

// local include
#include "common/pcl_types.h"
#include "common/logging.h"

class Render{
public:
    Render();
    ~Render();

    void InitCamera(CameraAngle angle, pcl_util::PCLVisualizerPtr &viewer);
    void RenderPointCloud(pcl_util::PCLVisualizerPtr &viewer, const pcl_util::PointCloudPtr &cloud, std::string name, Color color = Color(1,1,1));
    void RenderBBox(pcl_util::PCLVisualizerPtr &viewer, BBox box, int id, Color color);
};



#endif //PERCEPTION_ROS_RENDER_H

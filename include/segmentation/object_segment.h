//
/******************************************************************************/
/*!
File name: grid_map.h

Description:
This file define class of GridMap to transform point cloud to grid map.

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef PERCEPTION_ROS_GRID_MAP_H
#define PERCEPTION_ROS_GRID_MAP_H

// ros include
#include <ros/ros.h>

// pcl include
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

// opencv include
#include <opencv2/opencv.hpp>

// json include
#include <jsoncpp/json/json.h>

// math include
#include <math.h>

// local include
#include "common/pcl_types.h"
#include "common/logging.h"

extern Logging logger;

class Grid{
public:
    int row_;
    int column_;
    int type_;

    Grid(int row, int column, int type){
        row_ = row;
        column_ = column;
        type_ = type;
    }

    bool operator<(const Grid &p) const{
        return (row < p.row_) || (row == p.row_ && column < p.column_);
    }
};

class Segment {
public:
    Segment();
    ~Segment();

    bool SetParams(int column, int row, float grid_size, float height_threshold, float absolute_height);
    float Min(float x, float y);
    float Max(float x, float y);

    bool BuildGridMap(pcl_util::PointCloudPtr &in_cloud_ptr, std::map<Grid, std::vector<pcl_util::Point>> &grid);
private:
    int column_;
    int row_;
    float grid_size_;
    float height_threshold_;
    float absolute_height_;
};

#endif //PERCEPTION_ROS_GRID_MAP_H
//
/******************************************************************************/
/*!
File name: pcl_types.h

Description:
This file define head to redefine the PCL type

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef PERCEPTION_ROS_PCL_TYPES_H
#define PERCEPTION_ROS_PCL_TYPES_H

// pcl include
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/visualization/pcl_visualizer.h>

namespace pcl_util{

    typedef pcl::PointXY Point2d;
    typedef pcl::PointCloud<Point2d> PointCloud2d;
    typedef pcl::PointCloud<Point2d>::Ptr PointCloud2dPtr;
    typedef pcl::PointCloud<Point2d>::ConstPtr PointCloud2dConstPtr;

    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;
    typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;
    typedef pcl::PointCloud<Point>::ConstPtr PointCloudConstPtr;

    typedef pcl::PointXYZI VPoint;
    typedef pcl::PointCloud<VPoint> VPointCloud;
    typedef pcl::PointCloud<VPoint>::Ptr VPointCloudPtr;
    typedef pcl::PointCloud<VPoint>::ConstPtr VPointCloudConstPtr;

    typedef pcl::PointXYZRGB CPoint;
    typedef pcl::PointCloud<CPoint> CPointCloud;
    typedef pcl::PointCloud<CPoint>::Ptr CPointCloudPtr;
    typedef pcl::PointCloud<CPoint>::ConstPtr CPointCloudConstPtr;

    typedef pcl::PointIndices PointIndices;
    typedef pcl::PointIndices::Ptr PointIndicesPtr;

    //typedef pcl::ExtractIndices<Point> ExtractIndices;
    //typedef pcl::ExtractIndices<VPoint> VExtractIndices;
    //typedef pcl::ExtractIndices<CPoint> CExtractIndices;

    typedef pcl::visualization::PCLVisualizer PCLVisualizer;
    typedef pcl::visualization::PCLVisualizer::Ptr PCLVisualizerPtr;

    typedef pcl::KdTreeFLANN<Point> KdTree;

    struct BoundingCube{
        float x;
        float y;
        float z;
        float length;
        float width;
        float height;
        float yaw;

        float centroid_x;
        float centroid_y;
        float centroid_z;
    };

}

#endif //PERCEPTION_ROS_PCL_TYPES_H

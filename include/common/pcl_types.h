//
// Created by chenwei on 20-5-21.
//

#ifndef PERCEPTION_ROS_PCL_TYPES_H
#define PERCEPTION_ROS_PCL_TYPES_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/search/impl/kdtree.hpp>

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

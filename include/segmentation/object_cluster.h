//
/******************************************************************************/
/*!
File name: grid_map.h

Description:
This file define class of Segment to cluster object.

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/


#ifndef PERCEPTION_ROS_OBJECT_CLUSTER_H
#define PERCEPTION_ROS_OBJECT_CLUSTER_H

// pcl include
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

// system include
#include <thread>
#include <mutex>

// local include
#include "object_segment.h"
#include "common/logging.h"

extern Logging logger;

class Cluster{
public:
    Cluster();
    ~Cluster();

    bool Init(Json::Value params, std::string key);
    void MultCluster(pcl_util::PointCloudPtr &in_cloud_ptr, std::vector<pcl_util::PointCloud> &object_cloud);
    void ClusterObject(pcl_util::PointCloudPtr &in_cloud_ptr, double max_cluster_distance, std::vector<pcl_util::PointCloud> &object_cloud);
    void Process(pcl_util::PointCloudPtr &in_cloud_ptr, pcl_util::PointCloudPtr &out_cloud_ptr);

private:
    std::shared_ptr<Segment> segment_;

    std::vector<float> seg_distance_;
    std::vector<float> cluster_scale_;
    int min_cluster_size_;
    int max_cluster_size_;
};

#endif //PERCEPTION_ROS_OBJECT_CLUSTER_H

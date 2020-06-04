
#include "segmentation/object_cluster.h"

Cluster::Cluster() {
    thread_num_ = 4;
}

Cluster::~Cluster() {

}

bool Cluster::Init(Json::Value params, std::string key){
    if(params.isMember(key)) {
        Json::Value segmentation_param = params[key];

        int column, row;
        float grid_size, height_threshold, absolute_height;
        if (segmentation_param.isMember("column") && segmentation_param["column"].isInt()) {
            column = segmentation_param["column"].asInt();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named column in the segmentation config.\n", __func__);
            return false;
        }

        if (segmentation_param.isMember("row") && segmentation_param["row"].isInt()) {
            row = segmentation_param["row"].asInt();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named row in the segmentation config.\n", __func__);
            return false;
        }

        if (segmentation_param.isMember("size") && segmentation_param["size"].isDouble()) {
            grid_size = segmentation_param["size"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named size in the segmentation config.\n", __func__);
            return false;
        }

        if (segmentation_param.isMember("threshold") && segmentation_param["threshold"].isDouble()) {
            height_threshold = segmentation_param["threshold"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named threshold in the segmentation config.\n", __func__);
            return false;
        }

        if (segmentation_param.isMember("abs_height") && segmentation_param["abs_height"].isDouble()) {
            absolute_height = segmentation_param["abs_height"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named absolute_height in the segmentation config.\n", __func__);
            return false;
        }

        if (segmentation_param.isMember("min_cluster_size") && segmentation_param["min_cluster_size"].isInt()) {
            min_cluster_size_ = segmentation_param["min_cluster_size"].asInt();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named min_cluster_size in the segmentation config.\n", __func__);
            return false;
        }

        if (segmentation_param.isMember("max_cluster_size") && segmentation_param["max_cluster_size"].isInt()) {
            max_cluster_size_ = segmentation_param["max_cluster_size"].asInt();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named max_cluster_size_ in the segmentation config.\n", __func__);
            return false;
        }

        if (segmentation_param.isMember("seg_distance") && segmentation_param["seg_distance"].isArray()) {
            int size = segmentation_param["seg_distance"].size();
            for (int i = 0; i < size; ++i) {
                seg_distance_.push_back(segmentation_param["seg_distance"][i].asFloat());
            }
        } else {
            logger.Log(ERROR, "[%s]: Has not key named seg_distance in the segmentation config.\n", __func__);
            return false;
        }

        if (segmentation_param.isMember("cluster_scale") && segmentation_param["cluster_scale"].isArray()) {
            int size = segmentation_param["cluster_scale"].size();
            for (int i = 0; i < size; ++i) {
                cluster_scale_.push_back(segmentation_param["cluster_scale"][i].asFloat());
            }
        } else {
            logger.Log(ERROR, "[%s]: Has not key named isArray in the segmentation config.\n", __func__);
            return false;
        }

        // set grid map params
        segment_ = std::make_shared<Segment>();
        segment_->SetParams(column, row, grid_size, height_threshold, absolute_height);

    }else{
        logger.Log(ERROR, "[%s]: Has not key named segmentation in the perception config.\n", __func__);
    }
}

void Cluster::VoxelGridFilter(pcl_util::PointCloudPtr &in_cloud_ptr, pcl_util::PointCloudPtr &out_cloud_ptr,
                              float leaf_size) {
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(in_cloud_ptr);
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.filter(*out_cloud_ptr);
}


void Cluster::EuclCluster(pcl_util::PointCloudPtr &in_cloud_ptr, std::vector<pcl_util::PointCloud> &object_cloud) {
    size_t segment_size = cluster_scale_.size();
    std::vector<pcl_util::PointCloudPtr> segment_array(segment_size);

    for (size_t i = 0; i < segment_array.size(); i++) {
        pcl_util::PointCloudPtr tmp(new pcl_util::PointCloud);
        segment_array[i] = tmp;
    }

#pragma omp for
    for (size_t i = 0; i < in_cloud_ptr->points.size(); i++) {
        pcl_util::Point current_point;
        current_point.x = in_cloud_ptr->points[i].x;
        current_point.y = in_cloud_ptr->points[i].y;
        current_point.z = in_cloud_ptr->points[i].z;

        float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));
        if (origin_distance >= 40) {
            continue;
        }

        if (origin_distance < seg_distance_[0]) {
            segment_array[0]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[1]) {
            segment_array[1]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[2]) {
            segment_array[2]->points.push_back(current_point);
        }
        else{
            segment_array[3]->points.push_back(current_point);
        }
    }

    std::vector<std::thread> threads(thread_num_);
    for (unsigned int i = 0; i < thread_num_; ++i){
        threads[i] = std::thread(&Cluster::ClusterThread, this, std::ref(segment_array[i]), cluster_scale_[i], std::ref(object_cloud));
    }

    for (auto it = threads.begin(); it != threads.end(); ++it){
        it->join();
    }
}

void Cluster::ClusterThread(pcl_util::PointCloudPtr &in_cloud_ptr, float max_cluster_distance, std::vector<pcl_util::PointCloud> &object_cloud){
    pcl::search::KdTree<pcl_util::Point>::Ptr tree(new pcl::search::KdTree<pcl_util::Point>);

    pcl_util::PointCloudPtr cloud_2d(new pcl_util::PointCloud);
    pcl::copyPointCloud(*in_cloud_ptr, *cloud_2d);

    for (size_t i = 0; i < cloud_2d->points.size(); i++)
    {
        cloud_2d->points[i].z = 0;
    }

    if (cloud_2d->points.size() > 0)
        tree->setInputCloud(cloud_2d);

    std::vector<pcl::PointIndices> local_indices;

    pcl::EuclideanClusterExtraction<pcl_util::Point> euclidean;
    euclidean.setInputCloud(cloud_2d);
    euclidean.setClusterTolerance(max_cluster_distance);
    euclidean.setMinClusterSize(min_cluster_size_);
    euclidean.setMaxClusterSize(max_cluster_size_);
    euclidean.setSearchMethod(tree);
    euclidean.extract(local_indices);

    for (size_t i = 0; i < local_indices.size(); i++){
        pcl_util::PointCloud cloud;
        for (auto pit = local_indices[i].indices.begin(); pit != local_indices[i].indices.end(); ++pit) {
            cloud.push_back(in_cloud_ptr->points[*pit]);
        }
        mutex_.lock();
        object_cloud.push_back(cloud);
        mutex_.unlock();
    }
}

void Cluster::Process(pcl_util::PointCloudPtr &in_cloud_ptr, std::vector<pcl_util::PointCloud> &object_point_cloud){
    pcl_util::PointCloudPtr filter_cloud_ptr(new pcl_util::PointCloud());
    segment_->BuildGridMap(in_cloud_ptr, filter_cloud_ptr);

    VoxelGridFilter(filter_cloud_ptr, filter_cloud_ptr, 0.1);
    EuclCluster(filter_cloud_ptr, object_point_cloud);

    int size = object_point_cloud.size();
    for (int i = 0; i < size; ++i) {
        logger.Log(INFO, "object size is [%d].\n", object_point_cloud[i].points.size());
    }

}
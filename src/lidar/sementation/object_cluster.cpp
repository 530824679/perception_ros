
#include "segmentation/object_cluster.h"

Segment::Segment() {

}

Segment::~Segment() {

}

bool Segment::Init(Json::Value params, std::string key){
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
        grid_map_ = std::make_shared<GridMap>();
        grid_map_->SetParams(column, row, grid_size, height_threshold, absolute_height);

    }else{
        logger.Log(ERROR, "[%s]: Has not key named segmentation in the perception config.\n", __func__);
    }
}

void Segment::Cluster(pcl_util::VPointCloudPtr &in_cloud_ptr, std::vector<pcl_util::VPointCloud> &object_cloud) {
    size_t segment_size = cluster_scale_.size();
    std::vector<pcl_util::VPointCloudPtr> segment_array(segment_size);

#pragma omp for
    for (size_t i = 0; i < segment_array.size(); i++) {
        pcl_util::VPointCloudPtr tmp(new pcl_util::VPointCloud);
        segment_array[i] = tmp;
    }

#pragma omp for
    for (size_t i = 0; i < in_cloud_ptr->points.size(); i++) {
        pcl_util::VPoint current_point;
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

#pragma omp for
    for (size_t i = 0; i < segment_array.size(); i++){
        ClusterObject(segment_array[i], cluster_scale_[i], object_cloud);
    }

}

void Segment::ClusterObject(pcl_util::VPointCloudPtr &in_cloud_ptr, double max_cluster_distance, std::vector<pcl_util::VPointCloud> &object_cloud){
    pcl::search::KdTree<pcl_util::VPoint>::Ptr tree(new pcl::search::KdTree<pcl_util::VPoint>);

    pcl_util::VPointCloudPtr cloud_2d(new pcl_util::VPointCloud);
    pcl::copyPointCloud(*in_cloud_ptr, *cloud_2d);
#pragma omp for
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
    {
        cloud_2d->points[i].z = 0;
    }

    if (cloud_2d->points.size() > 0)
        tree->setInputCloud(cloud_2d);

    std::vector<pcl::PointIndices> local_indices;

    pcl::EuclideanClusterExtraction<pcl_util::VPoint> euclidean;
    euclidean.setInputCloud(cloud_2d);
    euclidean.setClusterTolerance(max_cluster_distance);
    euclidean.setMinClusterSize(min_cluster_size_);
    euclidean.setMaxClusterSize(max_cluster_size_);
    euclidean.setSearchMethod(tree);
    euclidean.extract(local_indices);

#pragma omp for
    for (size_t i = 0; i < local_indices.size(); i++){
        pcl_util::VPointCloud cloud;
        for (auto pit = local_indices[i].indices.begin(); pit != local_indices[i].indices.end(); ++pit) {
            cloud.push_back(in_cloud_ptr->points[*pit]);
        }
        object_cloud.push_back(cloud);
    }
}

void Segment::Process(pcl_util::VPointCloudPtr &in_cloud_ptr, pcl_util::VPointCloudPtr &out_cloud_ptr){
    std::vector<std::vector<int>> grid_map_type;
    std::vector<std::vector<pcl_util::VPointCloud>> grid_map_vec;
    grid_map_->BuildGridMap(in_cloud_ptr, grid_map_type, grid_map_vec);



    //std::vector<pcl_util::VPointCloud> object_point_cloud;
    //Cluster(out_cloud_ptr, object_point_cloud);





}
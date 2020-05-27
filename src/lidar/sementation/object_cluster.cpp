
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
            std::cout << "Has not key named column in the curb detect config" << std::endl;
            return false;
        }

        if (segmentation_param.isMember("row") && segmentation_param["row"].isInt()) {
            row = segmentation_param["row"].asInt();
        } else {
            std::cout << "Has not key named row in the curb detect config" << std::endl;
            return false;
        }

        if (segmentation_param.isMember("size") && segmentation_param["size"].isDouble()) {
            grid_size = segmentation_param["size"].asFloat();
        } else {
            std::cout << "Has not key named size in the curb detect config" << std::endl;
            return false;
        }

        if (segmentation_param.isMember("threshold") && segmentation_param["threshold"].isDouble()) {
            height_threshold = segmentation_param["threshold"].asFloat();
        } else {
            std::cout << "Has not key named threshold in the curb detect config" << std::endl;
            return false;
        }

        if (segmentation_param.isMember("abs_height") && segmentation_param["abs_height"].isDouble()) {
            absolute_height = segmentation_param["abs_height"].asFloat();
        } else {
            std::cout << "Has not key named absolute_height in the curb detect config" << std::endl;
            return false;
        }

        if (segmentation_param.isMember("threads_num") && segmentation_param["threads_num"].isInt()) {
            threads_num_ = segmentation_param["threads_num"].asInt();
        } else {
            std::cout << "Has not key named threads_num in the curb detect config" << std::endl;
            return false;
        }

        if (segmentation_param.isMember("seg_distance") && segmentation_param["seg_distance"].isArray()) {
            int size = segmentation_param["seg_distance"].size();
            for (int i = 0; i < size; ++i) {
                seg_distance_.push_back(segmentation_param["seg_distance"][i].asFloat());
            }
        } else {
            std::cout << "Has not key named seg_distance in the curb detect config" << std::endl;
            return false;
        }

        if (segmentation_param.isMember("cluster_scale") && segmentation_param["cluster_scale"].isArray()) {
            int size = segmentation_param["cluster_scale"].size();
            for (int i = 0; i < size; ++i) {
                cluster_scale_.push_back(segmentation_param["cluster_scale"][i].asFloat());
            }
        } else {
            std::cout << "Has not key named isArray in the curb detect config" << std::endl;
            return false;
        }

        // set grid map params
        grid_map_ = std::make_shared<GridMap>();
        grid_map_->SetParams(column, row, grid_size, height_threshold, absolute_height);
    }else{
        std::cout << "Has not key named curb_detect in the perception config" << std::endl;
    }
}

void Segment::Cluster(pcl_util::VPointCloudPtr &in_cloud_ptr) {
    size_t segment_size = cluster_scale_.size();
    std::vector<pcl_util::VPointCloudPtr> segment_array(segment_size);

#pragma omp for
    for (size_t i = 0; i < segment_array.size(); i++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
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

    std::vector<std::thread> threads(threads_num_);
    for (unsigned int i = 0; i < threads_num_; ++i){
        threads[i] = std::thread(&Segment::ClusterThread, this, segment_array[i], cluster_scale_[i]);
    }
    for (auto it = threads.begin(); it != threads.end(); ++it){
        it->join();
    }
}

void Segment::ClusterThread(pcl_util::VPointCloudPtr &in_cloud_ptr, double max_cluster_distance){
    pcl::search::KdTree<pcl_util::VPoint>::Ptr tree(new pcl::search::KdTree<pcl_util::VPoint>);

    pcl_util::VPointCloudPtr cloud_2d(new pcl_util::VPointCloud);
    pcl::copyPointCloud(*in_cloud_ptr, *cloud_2d);
#pragma omp for
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
    {
        cloud_2d->points[i].z = 0;
    }



}

void Segment::Process(pcl_util::VPointCloudPtr &in_cloud_ptr, pcl_util::VPointCloudPtr &out_cloud_ptr){
    pcl_util::VPointCloudPtr ground_cloud_ptr(new pcl_util::VPointCloud());
    pcl_util::VPointCloudPtr object_cloud_ptr(new pcl_util::VPointCloud());
    grid_map_->ConstructGridMap(in_cloud_ptr, ground_cloud_ptr, out_cloud_ptr);

    Cluster(out_cloud_ptr);

}
#include <pcl/filters/passthrough.h>
#include "roi_filter/roi_filter.h"

ROIFilter::ROIFilter() {

}

ROIFilter::~ROIFilter() {

}

bool ROIFilter::Init(Json::Value params, std::string key){
    if(params.isMember(key)) {
        Json::Value roi_filter_param = params[key];

        if (roi_filter_param.isMember("roi_x_min") && roi_filter_param["roi_x_min"].isDouble()) {
            roi_x_min_ = roi_filter_param["roi_x_min"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named roi_x_min in the roi filter config.\n", __func__);
            return false;
        }

        if (roi_filter_param.isMember("roi_x_max") && roi_filter_param["roi_x_max"].isDouble()) {
            roi_x_max_ = roi_filter_param["roi_x_max"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named roi_x_max in the roi filter config.\n", __func__);
            return false;
        }

        if (roi_filter_param.isMember("roi_y_min") && roi_filter_param["roi_y_min"].isDouble()) {
            roi_y_min_ = roi_filter_param["roi_y_min"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named roi_y_min in the roi filter config.\n", __func__);
            return false;
        }

        if (roi_filter_param.isMember("roi_y_max") && roi_filter_param["roi_y_max"].isDouble()) {
            roi_y_max_ = roi_filter_param["roi_y_max"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named roi_y_max in the roi filter config.\n", __func__);
            return false;
        }

        if (roi_filter_param.isMember("roi_z_min") && roi_filter_param["roi_z_min"].isDouble()) {
            roi_z_min_ = roi_filter_param["roi_z_min"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named roi_z_min in the roi filter config.\n", __func__);
            return false;
        }

        if (roi_filter_param.isMember("roi_z_max") && roi_filter_param["roi_z_max"].isDouble()) {
            roi_z_max_ = roi_filter_param["roi_z_max"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named roi_z_max in the roi filter config.\n", __func__);
            return false;
        }
    }else{
        logger.Log(ERROR, "[%s]: Has not key named roi_filter in the perception config.\n", __func__);
    }
}

bool ROIFilter::Filter(const pcl_util::VPointCloudPtr &in_cloud_ptr, pcl_util::VPointCloudPtr &filtered_cloud_all_ptr){
    FilterROI(in_cloud_ptr);
    *filtered_cloud_all_ptr = *filtered_cloud_all_ptr_;
    return true;
}

void ROIFilter::FilterROI(const pcl_util::VPointCloudPtr &in_cloud_ptr){
    filtered_cloud_all_ptr_.reset(new pcl_util::VPointCloud(*in_cloud_ptr));

    pcl::PassThrough<pcl_util::VPoint> cloudXFilter, cloudYFilter, cloudZFilter;
    cloudXFilter.setInputCloud(filtered_cloud_all_ptr_);
    cloudXFilter.setFilterFieldName("x");
    cloudXFilter.setFilterLimits(roi_x_min_, roi_x_max_);
    cloudXFilter.filter(*filtered_cloud_all_ptr_);

    cloudYFilter.setInputCloud(filtered_cloud_all_ptr_);
    cloudYFilter.setFilterFieldName("y");
    cloudYFilter.setFilterLimits(roi_y_min_, roi_y_max_);
    cloudYFilter.filter(*filtered_cloud_all_ptr_);

    cloudZFilter.setInputCloud(filtered_cloud_all_ptr_);
    cloudZFilter.setFilterFieldName("z");
    cloudZFilter.setFilterLimits(roi_z_min_, roi_z_max_);
    cloudZFilter.filter(*filtered_cloud_all_ptr_);

    return;
}


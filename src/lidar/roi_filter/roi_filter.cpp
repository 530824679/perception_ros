#include "roi_filter/roi_filter.h"

ROIFilter::ROIFilter() {

}

ROIFilter::~ROIFilter() {

}

bool ROIFilter::Init(Json::Value params, std::string key){
    if(params.isMember(key)) {
        Json::Value roi_filter_param = params[key];

        if (roi_filter_param.isMember("roi_x_min") && roi_filter_param["roi_x_min"].isInt()) {
            roi_x_min_ = roi_filter_param["roi_x_min"].asInt();
        } else {
            std::cout << "Has not key named roi_x_min in the roi filter config" << std::endl;
            return false;
        }

        if (roi_filter_param.isMember("roi_x_max") && roi_filter_param["roi_x_max"].isInt()) {
            roi_x_max_ = roi_filter_param["roi_x_max"].asInt();
        } else {
            std::cout << "Has not key named roi_x_max in the roi filter config" << std::endl;
            return false;
        }

        if (roi_filter_param.isMember("roi_y_min") && roi_filter_param["roi_y_min"].isInt()) {
            roi_y_min_ = roi_filter_param["roi_y_min"].asInt();
        } else {
            std::cout << "Has not key named roi_y_min in the roi filter config" << std::endl;
            return false;
        }

        if (roi_filter_param.isMember("roi_y_max") && roi_filter_param["roi_y_max"].isInt()) {
            roi_y_max_ = roi_filter_param["roi_y_max"].asInt();
        } else {
            std::cout << "Has not key named roi_y_max in the roi filter config" << std::endl;
            return false;
        }

        if (roi_filter_param.isMember("roi_z_min") && roi_filter_param["roi_z_min"].isInt()) {
            roi_z_min_ = roi_filter_param["roi_z_min"].asInt();
        } else {
            std::cout << "Has not key named roi_z_min in the roi filter config" << std::endl;
            return false;
        }

        if (roi_filter_param.isMember("roi_z_max") && roi_filter_param["roi_z_max"].isInt()) {
            roi_z_max_ = roi_filter_param["roi_z_max"].asInt();
        } else {
            std::cout << "Has not key named roi_z_max in the roi filter config" << std::endl;
            return false;
        }
    }else{
        std::cout << "Has not key named roi_filter in the perception config" << std::endl;
    }
}

void ROIFilter::FilterROI(const pcl_util::VPointCloudPtr &in_cloud_ptr){
    filtered_cloud_all_ptr_.reset(new pcl_util::VPointCloud(*in_cloud_ptr));



}


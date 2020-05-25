
#include <pcl/point_cloud.h>
#include "curb_detect/grid_map.h"

GridMap::GridMap(){

}

GridMap::~GridMap() {

}

bool GridMap::Init(Json::Value params, std::string key){
    if(params.isMember(key)) {
        Json::Value curb_detect_param = params[key];

        if (curb_detect_param.isMember("column") && curb_detect_param["column"].isInt()) {
            column_ = curb_detect_param["column"].asInt();
        } else {
            std::cout << "Has not key named column in the curb detect config" << std::endl;
            return false;
        }

        if (curb_detect_param.isMember("row") && curb_detect_param["row"].isInt()) {
            row_ = curb_detect_param["row"].asInt();
        } else {
            std::cout << "Has not key named row in the curb detect config" << std::endl;
            return false;
        }

        if (curb_detect_param.isMember("size") && curb_detect_param["size"].isDouble()) {
            grid_size_ = curb_detect_param["size"].asFloat();
        } else {
            std::cout << "Has not key named size in the curb detect config" << std::endl;
            return false;
        }

        if (curb_detect_param.isMember("threshold") && curb_detect_param["threshold"].isDouble()) {
            height_threshold_ = curb_detect_param["threshold"].asFloat();
        } else {
            std::cout << "Has not key named threshold in the curb detect config" << std::endl;
            return false;
        }
    }else{
        std::cout << "Has not key named curb_detect in the perception config" << std::endl;
    }
}

float GridMap::Min(float x, float y){
    return (x) < (y) ? (x) : (y);
}

float GridMap::Max(float x, float y){
    return (x) > (y) ? (x) : (y);
}

void GridMap::ConstructGridMap(pcl_util::VPointCloudPtr in_cloud_ptr, pcl_util::VPointCloudPtr out_cloud_ptr) {
    float min_height[row_][column_] = {0};
    float max_height[row_][column_] = {0};
    bool init[row_][column_] = {false};

    for (unsigned int count = 0; count < in_cloud_ptr->points.size(); count++) {
        int x_grid = floor(double(in_cloud_ptr->points[count].x) / grid_size_);
        int y_grid = floor(double(in_cloud_ptr->points[count].y) / grid_size_);
        if((x_grid < row_ && x_grid >= 0) && (y_grid < column_/2 && y_grid >= -column_/2)){
            if(!init[x_grid][y_grid]){
                min_height[x_grid][y_grid] = in_cloud_ptr->points[count].z;
                max_height[x_grid][y_grid] = in_cloud_ptr->points[count].z;
                init[x_grid][y_grid] = true;
            }else{
                min_height[x_grid][y_grid] = Min(min_height[x_grid][y_grid], in_cloud_ptr->points[count].z);
                max_height[x_grid][y_grid] = Max(max_height[x_grid][y_grid], in_cloud_ptr->points[count].z);
            }
        }
    }


    int result_count = 0;
    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); ++i){
        int x_grid = floor(double(in_cloud_ptr->points[i].x) / grid_size_);
        int y_grid = floor(double(in_cloud_ptr->points[i].y) / grid_size_);

        if((x_grid < row_ && x_grid >= 0) && (y_grid < column_/2 && y_grid >= -column_/2) && init[x_grid][y_grid]) {

            std::cout << "max is : " << max_height[x_grid][y_grid] << " min is : " << min_height[x_grid][y_grid] << std::endl;
            if((max_height[x_grid][y_grid] - min_height[x_grid][y_grid] > height_threshold_) || (min_height[x_grid][y_grid] > height_threshold_)){
                out_cloud_ptr->points[result_count].x = in_cloud_ptr->points[i].x;
                out_cloud_ptr->points[result_count].y = in_cloud_ptr->points[i].y;
                out_cloud_ptr->points[result_count].z = in_cloud_ptr->points[i].z;
                result_count++;
            }
        }
    }
}
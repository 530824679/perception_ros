
#include <pcl/point_cloud.h>
#include "segmentation/grid_map.h"

GridMap::GridMap(){
    column_ = 0;
    row_ = 0;
    grid_size_ = 0.0;
    height_threshold_ = 0.0;
    absolute_height_ = 0.0;
}

GridMap::~GridMap() {

}

bool GridMap::SetParams(int column, int row, float grid_size, float height_threshold, float absolute_height) {
    column_ = column;
    row_ = row;
    grid_size_ = grid_size;
    height_threshold_ = height_threshold;
    absolute_height_ = absolute_height;
}

float GridMap::Min(float x, float y){
    return (x) < (y) ? (x) : (y);
}

float GridMap::Max(float x, float y){
    return (x) > (y) ? (x) : (y);
}

bool GridMap::ConstructGridMap(pcl_util::VPointCloudPtr &in_cloud_ptr, pcl_util::VPointCloudPtr &ground_cloud_ptr, pcl_util::VPointCloudPtr &object_cloud_ptr){
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

    pcl::ExtractIndices<pcl_util::VPoint> cliper;
    pcl_util::PointIndices indices;
    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); ++i){
        int x_grid = floor(double(in_cloud_ptr->points[i].x) / grid_size_);
        int y_grid = floor(double(in_cloud_ptr->points[i].y) / grid_size_);

        if((x_grid < row_ && x_grid >= 0) && (y_grid < column_/2 && y_grid >= -column_/2) && init[x_grid][y_grid]) {

            if((max_height[x_grid][y_grid] - min_height[x_grid][y_grid] < height_threshold_) && (max_height[x_grid][y_grid] < absolute_height_)){
                std::cout << "max is : " << max_height[x_grid][y_grid] << " min is : " << min_height[x_grid][y_grid] << std::endl;
                indices.indices.push_back(i);
            }
        }
    }

    cliper.setInputCloud (in_cloud_ptr);
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(false);
    cliper.filter(*ground_cloud_ptr);

    cliper.setNegative(true);
    cliper.filter(*object_cloud_ptr);

    return true;
}
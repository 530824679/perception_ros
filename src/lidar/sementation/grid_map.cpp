
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

void GridMap::InitGridMap(std::vector<std::vector<int>> &grid_map_type, std::vector<std::vector<pcl_util::VPointCloud>> &grid_map_vec){
    if (column_ > 0 && row_ > 0){
        grid_map_vec.resize(row_);
        grid_map_type.resize(row_);
        for (int i = 0; i < row_; ++i) {
            grid_map_vec[i].resize(column_);
            grid_map_type[i].resize(column_);
            for (int j = 0; j < column_; ++j) {
                grid_map_type[i][j] = UNKNOW;
                grid_map_vec[i][j].clear();
            }
        }
    }else{
        logger.Log(ERROR, "[%s]: The Grid Map column is [%d], row is [%d] incorrect\n", __func__, column_, row_);
    }
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

bool GridMap::BuildGridMap(pcl_util::VPointCloudPtr &in_cloud_ptr, std::vector<std::vector<int>> &grid_map_type, std::vector<std::vector<pcl_util::VPointCloud>> &grid_map_vec){

    InitGridMap(grid_map_type, grid_map_vec);

    float min_height[row_][column_] = {0};
    float max_height[row_][column_] = {0};
    bool init[row_][column_] = {false};

    for (unsigned int count = 0; count < in_cloud_ptr->points.size(); count++) {
        if (isnan(in_cloud_ptr->points[count].x))
            continue;

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
            grid_map_vec[x_grid][y_grid].push_back(in_cloud_ptr->points[count]);
        }
    }

    for (int i = 0; i < column_; ++i) {
        for (int j = 0; j < row_; ++j) {
            if (grid_map_vec[i][j].empty())
                continue;
            if((max_height[i][j] - min_height[i][j]) > height_threshold_){
                grid_map_type[i][j] = OBSTACLE;
            }else{
                // the lowest point height is between[-lidar_height-0.05, -lidar_height + 0.1]
                if (min_height[i][j] > (-0.04 - 0.05) && min_height[i][j] < (-0.04 + 0.1))
                {
                    grid_map_type[i][j] = GROUND;
                }
            }
        }
    }

    return true;
}
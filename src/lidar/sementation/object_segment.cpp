
#include <pcl/point_cloud.h>
#include "segmentation/object_segment.h"

Segment::Segment(){
    column_ = 0;
    row_ = 0;
    grid_size_ = 0.0;
    height_threshold_ = 0.0;
    absolute_height_ = 0.0;
}

Segment::~Segment() {

}

bool Segment::SetParams(int column, int row, float grid_size, float height_threshold, float absolute_height) {
    column_ = column;
    row_ = row;
    grid_size_ = grid_size;
    height_threshold_ = height_threshold;
    absolute_height_ = absolute_height;
}

float Segment::Min(float x, float y){
    return (x) < (y) ? (x) : (y);
}

float Segment::Max(float x, float y){
    return (x) > (y) ? (x) : (y);
}

void Segment::InitGridMap(std::vector<std::vector<int>> &grid_map_type){
    grid_map_type.resize(row_);
    for (int i = 0; i < row_; ++i) {
        grid_map_type[i].resize(column_);
        for (int j = 0; j < column_; ++j) {
            grid_map_type[i][j] = UNKNOW;
        }
    }
}

bool Segment::BuildGridMap(pcl_util::PointCloudPtr &in_cloud_ptr, std::vector<std::vector<int>> &grid_map_type){
    InitGridMap(grid_map_type);

    float min_height[row_][column_] = {0};
    float max_height[row_][column_] = {0};
    bool init[row_][column_] = {false};

    for (unsigned int count = 0; count < in_cloud_ptr->points.size(); count++) {
        if (std::isnan(in_cloud_ptr->points[count].x))
            continue;

        int x_grid = floor(double(in_cloud_ptr->points[count].x) / grid_size_);
        int y_grid = floor(double(in_cloud_ptr->points[count].y) / grid_size_) + column_/2;

        if((x_grid < row_ && x_grid >= 0) && (y_grid < column_ && y_grid >= 0)){
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

    for (int i = 0; i < row_; ++i) {
        for (int j = 0; j < column_; ++j) {
            if (init[i][j] == false)
                continue;
            if((max_height[i][j] - min_height[i][j]) < height_threshold_ && max_height[i][j] < absolute_height_){
                grid_map_type[i][j] = GROUND;
            }else{
                grid_map_type[i][j] = OBSTACLE;
            }
            logger.Log(INFO, "[%d][%d]: [%d]", i, j, grid_map_type[i][j]);
        }
        logger.Log(INFO, "\n");
    }

    return true;
}
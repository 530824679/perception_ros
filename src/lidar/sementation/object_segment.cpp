
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

bool Segment::BuildGridMap(pcl_util::PointCloudPtr &in_cloud_ptr, pcl_util::PointCloudPtr &out_cloud_ptr){
    Grid grid[row_][column_];

    for (unsigned int count = 0; count < in_cloud_ptr->points.size(); count++) {
        if (std::isnan(in_cloud_ptr->points[count].x))
            continue;

        int x_grid = floor(double(in_cloud_ptr->points[count].x) / grid_size_);
        int y_grid = floor(double(in_cloud_ptr->points[count].y) / grid_size_) + column_/2;

        if((x_grid < row_ && x_grid >= 0) && (y_grid < column_ && y_grid >= 0)){
            if(!grid[x_grid][y_grid].init_){
                grid[x_grid][y_grid].min_height_ = in_cloud_ptr->points[count].z;
                grid[x_grid][y_grid].max_height_ = in_cloud_ptr->points[count].z;
                grid[x_grid][y_grid].init_ = true;
            }else{
                grid[x_grid][y_grid].min_height_ = Min(grid[x_grid][y_grid].min_height_, in_cloud_ptr->points[count].z);
                grid[x_grid][y_grid].max_height_ = Max(grid[x_grid][y_grid].max_height_, in_cloud_ptr->points[count].z);
            }
            grid[x_grid][y_grid].point_num_++;
            grid[x_grid][y_grid].grid_inliers_->indices.push_back(count);
            grid[x_grid][y_grid].grid_cloud_->points.push_back(in_cloud_ptr->points[count]);
        }
    }

    for (int i = 0; i < row_; ++i) {
        for (int j = 0; j < column_; ++j) {
            if (!grid[i][j].init_){
                continue;
            }

            for (int k = 0; k < grid[i][j].point_num_; ++k) {
                if ((grid[i][j].grid_cloud_->points[k].z > height_threshold_)){
                    out_cloud_ptr->points.push_back(grid[i][j].grid_cloud_->points[k]);
                }
            }
        }
    }


//    for (int i = 0; i < row_; ++i) {
//        for (int j = 0; j < column_; ++j) {
//            if (!grid[i][j].init_)
//                continue;
//            if(grid[i][j].max_height_ > absolute_height_){
//                grid_type[i][j] = OBSTACLE;
//            }else{
//
//            }
//            //logger.Log(INFO, "[%d][%d]: [%d]", i, j, grid[i][j].type_);
//        }
//        //logger.Log(INFO, "\n");
//    }

    return true;
}
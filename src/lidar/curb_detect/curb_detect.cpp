#include "curb_detect/curb_detect.h"

CurbDetect::CurbDetect() {

}

CurbDetect::~CurbDetect() {

}

void CurbDetect::Process(pcl_util::VPointCloudPtr &in_cloud_ptr, pcl_util::VPointCloudPtr &out_cloud_ptr) {
//    pcl_util::VPointCloudPtr grip_map_ptr(new pcl_util::VPointCloud());
//    grid_map_->ConstructGridMap(in_cloud_ptr, grip_map_ptr);
//
//    pcl_util::VPointCloud left_point_cloud, right_point_cloud;
//    for (int i = 0; i < grip_map_ptr->points.size(); i++){
//        pcl_util::VPoint point;
//        point.x = grip_map_ptr->points[i].x;
//        point.y = grip_map_ptr->points[i].y;
//        point.z = 0;
//
//        if(grip_map_ptr->points[i].y > 0){
//            left_point_cloud.push_back(point);
//        }else{
//            right_point_cloud.push_back(point);
//        }
//    }





//    right_point_cloud.header.frame_id = "livox_frame";
//    right_point_cloud.height = 1;
//    right_point_cloud.width = right_point_cloud.points.size();
//    out_cloud_ptr = right_point_cloud.makeShared();

}
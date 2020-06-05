
#include "object_builder/bbox_fitting.h"

BBoxEstimator::BBoxEstimator(){

}

BBoxEstimator::~BBoxEstimator(){

}

bool BBoxEstimator::SearchBasedFitting(pcl_util::PointCloudPtr &in_cloud_ptr, pcl_util::BBox &box) {
    float min_z = in_cloud_ptr->points.front().z;
    float max_z = in_cloud_ptr->points.front().z;
    for (auto &p: in_cloud_ptr->points){
        if (p.z < min_z){
            min_z = p.z;
        }
        if (p.z > max_z){
            max_z = p.z;
        }
    }

    std::vector<std::pair<float, float>> Q;
    const float max_angle = M_PI / 2.0;
    const float angle_resulution = M_PI / 180.0;
    for(float theta = 0; theta < max_angle; theta += angle_resulution){
        Eigen::Vector2d e_1, e_2;
        e_1 << std::cos(theta), std::sin(theta);
        e_2 << -std::sin(theta), std::cos(theta);

        std::vector<float> c_1, c_2;
        for (const auto &point: in_cloud_ptr->points) {
            c_1.emplace_back(point.x * e_1.x() + point.y * e_1.y());
            c_2.emplace_back(point.x * e_2.x() + point.y * e_2.y());
        }

        float q = CalcCloseness(c_1, c_2);
        Q.emplace_back(theta, q);
    }


}

double BBoxEstimator::CalcCloseness(const std::vector<float> &C_1, const std::vector<float> &C_2) {

}

#include "object_builder/bbox_fitting.h"

BBoxEstimator::BBoxEstimator(){

}

BBoxEstimator::~BBoxEstimator(){

}

void BBoxEstimator::Estimate(std::vector<pcl_util::PointCloud> clusters, std::vector<BBox> bboxes){
    for (int i = 0; i < clusters.size(); i++) {
        auto cluster = clusters[i];
        BBox box{};
        if (SearchBasedFitting(cluster.makeShared(), box)){

        }else{
            logger.Log(ERROR, "Search Based Fitting false.\n");
        }
    }
}

bool BBoxEstimator::SearchBasedFitting(pcl_util::PointCloudPtr &in_cloud_ptr, BBox &box) {
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

    float dz = max_z - min_z;
    bool ret = CalcBBox(in_cloud_ptr, Q, dz ,box);
    if (ret){
        return true;
    } else{
        logger.Log(ERROR, "Calculate bounding box false.\n");
        return false;
    }
}

float BBoxEstimator::CalcCloseness(const std::vector<float> &C_1, const std::vector<float> &C_2) {
    const float min_c_1 = *std::min_element(C_1.begin(), C_1.end());
    const float max_c_1 = *std::max_element(C_1.begin(), C_1.end());
    const float min_c_2 = *std::min_element(C_2.begin(), C_2.end());
    const float max_c_2 = *std::max_element(C_2.begin(), C_2.end());

    std::vector<float> D_1;
    for (const auto& c_1_element : C_1)
    {
        const float v = std::min(max_c_1 - c_1_element, c_1_element - min_c_1);
        D_1.push_back(std::fabs(v));
    }

    std::vector<float> D_2;
    for (const auto& c_2_element : C_2)
    {
        const float v = std::min(max_c_2 - c_2_element, c_2_element - min_c_2);
        D_2.push_back(v * v);
    }

    const float d_min = 0.05;
    const float d_max = 0.50;
    float beta = 0;
    for (size_t i = 0; i < D_1.size(); ++i)
    {
        const float d = std::min(std::max(std::min(D_1.at(i), D_2.at(i)), d_min), d_max);
        beta += 1.0 / d;
    }
    return beta;
}

bool BBoxEstimator::CalcBBox(pcl_util::PointCloudPtr &in_cloud_ptr, std::vector<std::pair<float, float>> &Q, float dz, BBox &box){
    float theta_star;
    float max_q;
    for (size_t i = 0; i < Q.size(); ++i) {
        if (max_q < Q.at(i).second || i == 0) {
            max_q = Q.at(i).second;
            theta_star = Q.at(i).first;
        }
    }

    Eigen::Vector2d e_1_star;
    Eigen::Vector2d e_2_star;
    e_1_star << std::cos(theta_star), std::sin(theta_star);
    e_2_star << -std::sin(theta_star), std::cos(theta_star);

    std::vector<float> C_1_star;
    std::vector<float> C_2_star;
    for (const auto& point: in_cloud_ptr->points) {
        C_1_star.emplace_back(point.x * e_1_star.x() + point.y * e_1_star.y());
        C_2_star.emplace_back(point.x * e_2_star.x() + point.y * e_2_star.y());
    }

    const float min_C_1_star = *std::min_element(C_1_star.begin(), C_1_star.end());
    const float max_C_1_star = *std::max_element(C_1_star.begin(), C_1_star.end());
    const float min_C_2_star = *std::min_element(C_2_star.begin(), C_2_star.end());
    const float max_C_2_star = *std::max_element(C_2_star.begin(), C_2_star.end());

    const float a_1 = std::cos(theta_star);
    const float b_1 = std::sin(theta_star);
    const float c_1 = min_C_1_star;
    const float a_2 = -1.0 * std::sin(theta_star);
    const float b_2 = std::cos(theta_star);
    const float c_2 = min_C_2_star;
    const float a_3 = std::cos(theta_star);
    const float b_3 = std::sin(theta_star);
    const float c_3 = max_C_1_star;
    const float a_4 = -1.0 * std::sin(theta_star);
    const float b_4 = std::cos(theta_star);
    const float c_4 = max_C_2_star;

    // calc center of bounding box
    float intersection_x_1 = (b_1 * c_2 - b_2 * c_1) / (a_2 * b_1 - a_1 * b_2);
    float intersection_y_1 = (a_1 * c_2 - a_2 * c_1) / (a_1 * b_2 - a_2 * b_1);
    float intersection_x_2 = (b_3 * c_4 - b_4 * c_3) / (a_4 * b_3 - a_3 * b_4);
    float intersection_y_2 = (a_3 * c_4 - a_4 * c_3) / (a_3 * b_4 - a_4 * b_3);

    // calc dimention of bounding box
    Eigen::Vector2d e_x;
    Eigen::Vector2d e_y;
    e_x << a_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1)), b_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1));
    e_y << a_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2)), b_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2));
    Eigen::Vector2d diagonal_vec;
    diagonal_vec << intersection_x_1 - intersection_x_2, intersection_y_1 - intersection_y_2;

    box.yaw = std::atan2(e_1_star.y(), e_1_star.x());
    box.x = (intersection_x_1 + intersection_x_2) / 2.0;
    box.y = (intersection_y_1 + intersection_y_2) / 2.0;
    box.z = CalcCloudCentroid(in_cloud_ptr).z();

    constexpr float ep = 0.001;
    box.dx = std::fabs(e_x.dot(diagonal_vec));
    box.dy = std::fabs(e_y.dot(diagonal_vec));
    box.dz = std::max(dz, ep);

    if (box.dx < ep && box.dy < ep)
        return false;
    box.dx = std::max(box.dx, ep);
    box.dy = std::max(box.dy, ep);
    return true;
}

Eigen::Array3f BBoxEstimator::CalcCloudCentroid(pcl_util::PointCloudPtr &in_cloud_ptr){
    Eigen::Array3f min_pt = in_cloud_ptr->at(0).getArray3fMap();
    Eigen::Array3f max_pt = in_cloud_ptr->at(0).getArray3fMap();
    for (int i = 1; i < in_cloud_ptr->size(); i++) {
        min_pt = in_cloud_ptr->at(i).getArray3fMap().min(min_pt);
        max_pt = in_cloud_ptr->at(i).getArray3fMap().max(max_pt);
    }

    Eigen::Array3f centroid = (min_pt + max_pt) / 2.0f;
    return centroid;
}
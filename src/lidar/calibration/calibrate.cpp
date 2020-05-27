
#include "calibration/calibrate.h"

Calibrate::Calibrate(){

}

Calibrate::~Calibrate(){

}

bool Calibrate::Init(Json::Value params, std::string key){
    if(params.isMember(key)) {
        Json::Value calibrate_param = params[key];

        if (calibrate_param.isMember("x_min") && calibrate_param["x_min"].isDouble()) {
            x_min_ = calibrate_param["x_min"].asFloat();
        } else {
            std::cout << "Has not key named x_min in the calibrate config" << std::endl;
            return false;
        }

        if (calibrate_param.isMember("x_max") && calibrate_param["x_max"].isDouble()) {
            x_max_ = calibrate_param["x_max"].asFloat();
        } else {
            std::cout << "Has not key named x_max in the calibrate config" << std::endl;
            return false;
        }

        if (calibrate_param.isMember("y_min") && calibrate_param["y_min"].isDouble()) {
            y_min_ = calibrate_param["y_min"].asFloat();
        } else {
            std::cout << "Has not key named y_min in the calibrate config" << std::endl;
            return false;
        }

        if (calibrate_param.isMember("y_max") && calibrate_param["y_max"].isDouble()) {
            y_max_ = calibrate_param["y_max"].asFloat();
        } else {
            std::cout << "Has not key named y_max in the calibrate config" << std::endl;
            return false;
        }

        if (calibrate_param.isMember("z_min") && calibrate_param["z_min"].isDouble()) {
            z_min_ = calibrate_param["z_min"].asFloat();
        } else {
            std::cout << "Has not key named z_min in the calibrate config" << std::endl;
            return false;
        }

        if (calibrate_param.isMember("z_max") && calibrate_param["z_max"].isDouble()) {
            z_max_ = calibrate_param["z_max"].asFloat();
        } else {
            std::cout << "Has not key named z_max in the calibrate config" << std::endl;
            return false;
        }
    }else{
        std::cout << "Has not key named calibrate in the perception config" << std::endl;
    }
}

void Calibrate::Correct(const pcl_util::VPointCloudPtr in_cloud_ptr, pcl_util::VPointCloudPtr out_cloud_ptr){
    pcl_util::VPointCloudPtr filter_cloud_ptr(new pcl_util::VPointCloud);
    FilterPointCloud(in_cloud_ptr, filter_cloud_ptr);

    Eigen::MatrixXf normal_mat = EstimateGroundPlane(filter_cloud_ptr, 0.1);
    Eigen::Matrix4f rotation = CreateRotateMatrix(normal_mat, Eigen::Vector3f(0,0,1));

    pcl::transformPointCloud(*in_cloud_ptr, *out_cloud_ptr, rotation);
}

void Calibrate::FilterPointCloud(const pcl_util::VPointCloudPtr in_cloud_ptr, pcl_util::VPointCloudPtr out_cloud_ptr){
    pcl::ExtractIndices<pcl_util::VPoint> cliper;
    cliper.setInputCloud(in_cloud_ptr);
    pcl_util::PointIndices indices;

#pragma omp for
    for (size_t i = 0; i < in_cloud_ptr->points.size(); i++){
        if (in_cloud_ptr->points[i].x < x_max_ && in_cloud_ptr->points[i].x > x_min_ &&
                in_cloud_ptr->points[i].y < y_max_ && in_cloud_ptr->points[i].y > y_min_ &&
                in_cloud_ptr->points[i].z < z_max_ && in_cloud_ptr->points[i].z > z_min_){
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl_util::PointIndices>(indices));
    cliper.setNegative(false);
    cliper.filter(*out_cloud_ptr);
}

Eigen::MatrixXf Calibrate::EstimateGroundPlane(const pcl_util::VPointCloudPtr in_cloud_ptr, const float distance_threshold){
    pcl::SACSegmentation<pcl_util::VPoint> plane_seg;
    pcl_util::PointIndicesPtr Plane_inliers(new pcl_util::PointIndices);
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    plane_seg.setOptimizeCoefficients(true);
    plane_seg.setModelType(pcl::SACMODEL_PLANE);
    plane_seg.setMethodType(pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold(distance_threshold);
    plane_seg.setInputCloud(in_cloud_ptr);
    plane_seg.segment(*Plane_inliers, *plane_coefficients);

//    std::cout << "Model coefficeients:" << plane_coefficients->values[0] << " "
//              << plane_coefficients->values[1] << " "
//              << plane_coefficients->values[2] << " "
//              << plane_coefficients->values[3] << std::endl;

    Eigen::Vector3f normal_vector(plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2]);
    return normal_vector;
}

Eigen::Matrix4f Calibrate::CreateRotateMatrix(Eigen::Vector3f before, Eigen::Vector3f after){
    before.normalize();
    after.normalize();

    float angle = acos(before.dot(after));
    Eigen::Vector3f p_rotate =before.cross(after);
    p_rotate.normalize();

    Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
    rotationMatrix(0, 0) = cos(angle) + p_rotate[0] * p_rotate[0] * (1 - cos(angle));
    rotationMatrix(0, 1) = p_rotate[0] * p_rotate[1] * (1 - cos(angle) - p_rotate[2] * sin(angle));//这里跟公式比多了一个括号，但是看实验结果它是对的。
    rotationMatrix(0, 2) = p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));

    rotationMatrix(1, 0) = p_rotate[2] * sin(angle) + p_rotate[0] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 1) = cos(angle) + p_rotate[1] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 2) = -p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));

    rotationMatrix(2, 0) = -p_rotate[1] * sin(angle) +p_rotate[0] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 1) = p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 2) = cos(angle) + p_rotate[2] * p_rotate[2] * (1 - cos(angle));

    return rotationMatrix;
}
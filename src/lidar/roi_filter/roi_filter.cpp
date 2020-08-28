#include <pcl/filters/passthrough.h>
#include "roi_filter/roi_filter.h"

ROIFilter::ROIFilter() {

}

ROIFilter::~ROIFilter() {

}

bool ROIFilter::Init(Json::Value params, std::string key){
    if(params.isMember(key)) {
        Json::Value roi_filter_param = params[key];

        if (roi_filter_param.isMember("roi_x_min") && roi_filter_param["roi_x_min"].isDouble()) {
            roi_x_min_ = roi_filter_param["roi_x_min"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named roi_x_min in the roi filter config.\n", __func__);
            return false;
        }

        if (roi_filter_param.isMember("roi_x_max") && roi_filter_param["roi_x_max"].isDouble()) {
            roi_x_max_ = roi_filter_param["roi_x_max"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named roi_x_max in the roi filter config.\n", __func__);
            return false;
        }

        if (roi_filter_param.isMember("roi_y_min") && roi_filter_param["roi_y_min"].isDouble()) {
            roi_y_min_ = roi_filter_param["roi_y_min"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named roi_y_min in the roi filter config.\n", __func__);
            return false;
        }

        if (roi_filter_param.isMember("roi_y_max") && roi_filter_param["roi_y_max"].isDouble()) {
            roi_y_max_ = roi_filter_param["roi_y_max"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named roi_y_max in the roi filter config.\n", __func__);
            return false;
        }

        if (roi_filter_param.isMember("roi_z_min") && roi_filter_param["roi_z_min"].isDouble()) {
            roi_z_min_ = roi_filter_param["roi_z_min"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named roi_z_min in the roi filter config.\n", __func__);
            return false;
        }

        if (roi_filter_param.isMember("roi_z_max") && roi_filter_param["roi_z_max"].isDouble()) {
            roi_z_max_ = roi_filter_param["roi_z_max"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named roi_z_max in the roi filter config.\n", __func__);
            return false;
        }
    }else{
        logger.Log(ERROR, "[%s]: Has not key named roi_filter in the perception config.\n", __func__);
    }
}

bool ROIFilter::Filter(const pcl_util::PointCloudPtr &in_cloud_ptr, pcl_util::PointCloudPtr &filtered_cloud_all_ptr){
    FilterROI(in_cloud_ptr);
    *filtered_cloud_all_ptr = *filtered_cloud_all_ptr_;
    return true;
}

void ROIFilter::FilterROI(const pcl_util::PointCloudPtr &in_cloud_ptr){
    filtered_cloud_all_ptr_.reset(new pcl_util::PointCloud(*in_cloud_ptr));

    pcl::PassThrough<pcl_util::Point> cloudXFilter, cloudYFilter, cloudZFilter;
    cloudXFilter.setInputCloud(filtered_cloud_all_ptr_);
    cloudXFilter.setFilterFieldName("x");
    cloudXFilter.setFilterLimits(roi_x_min_, roi_x_max_);
    cloudXFilter.filter(*filtered_cloud_all_ptr_);

    cloudYFilter.setInputCloud(filtered_cloud_all_ptr_);
    cloudYFilter.setFilterFieldName("y");
    cloudYFilter.setFilterLimits(roi_y_min_, roi_y_max_);
    cloudYFilter.filter(*filtered_cloud_all_ptr_);

    cloudZFilter.setInputCloud(filtered_cloud_all_ptr_);
    cloudZFilter.setFilterFieldName("z");
    cloudZFilter.setFilterLimits(roi_z_min_, roi_z_max_);
    cloudZFilter.filter(*filtered_cloud_all_ptr_);

    return;
}

bool ROIFilter::GroundPlaneFilter(pcl_util::PointCloudPtr input_cloud,Eigen::Vector4d &plane_coefficients){
        

        pcl_util::PointCloud cloud=*input_cloud;//0.3ms
        int points_num = cloud.size();
        std::cout<<"points cloud"<<points_num<<std::endl;
        int max_iterations_=500;
        
        std::unordered_set<int> inliers_result;
  
        while (max_iterations_){
            std::unordered_set<int> inliers;
            //Eigen::Vector4d coefficient;

            while(inliers.size() < 3){
                inliers.insert(rand()%points_num);
            }

            pcl_util::Point pt1, pt2, pt3;
            auto iter = inliers.begin();
            pt1.x = cloud[*iter].x;
            pt1.y = cloud[*iter].y;
            pt1.z = cloud[*iter].z;
            iter++;
            pt2.x = cloud[*iter].x;
            pt2.y = cloud[*iter].y;
            pt2.z = cloud[*iter].z;
            iter++;
            pt3.x = cloud[*iter].x;
            pt3.y = cloud[*iter].y;
            pt3.z = cloud[*iter].z;

            if(CollineationJudge(pt1, pt2, pt3))
                continue;

            float a, b, c, d, sqrt_abc;
            a = (pt2.y - pt1.y) * (pt3.z - pt1.z) - (pt2.z - pt1.z) * (pt3.y - pt1.y);
            b = (pt2.z - pt1.z) * (pt3.x - pt1.x) - (pt2.x - pt1.x) * (pt3.z - pt1.z);
            c = (pt2.x - pt1.x) * (pt3.y - pt1.y) - (pt2.y - pt1.y) * (pt3.x - pt1.x);
            d = -(a * pt1.x + b * pt1.y + c * pt1.z);
            sqrt_abc = sqrt(a * a + b * b + c * c);

            

            for (int index = 0; index < points_num; index++)
            {
                if (inliers.count(index) > 0)
                {
                    continue;
                }

                pcl_util::Point point = cloud[index];
                float x = point.x;
                float y = point.y;
                float z = point.z;
                float dist = fabs(a * x + b * y + c * z + d) / sqrt_abc;


                if(dist < 0.05)
                {
                    inliers.insert(index);
                }

                if(inliers.size() > inliers_result.size())
                {
                    inliers_result.swap(inliers);
                    plane_coefficients << a, b, c, d;
                }

            }

            max_iterations_--;
        }


        return true;
}


float ROIFilter::Distance(pcl_util::Point& pt1, pcl_util::Point& pt2)
{
    float x = pt1.x - pt2.x;
    float y = pt1.y - pt2.y;
    float z = pt1.z - pt2.z;
    return sqrt(x*x + y*y + z*z);
}

bool ROIFilter::CollineationJudge(pcl_util::Point& pt1, pcl_util::Point& pt2, pcl_util::Point& pt3)
{
    float edge_a = Distance(pt1, pt2);
    float edge_b = Distance(pt2, pt3);
    float edge_c = Distance(pt1, pt3);

    float p = 0.5 * (edge_a + edge_b + edge_c);
    float area = p * (p - edge_a) * (p - edge_b) * (p - edge_c);

    if(abs(area) < 1e-6)
        return true;
    else
        return false;
}




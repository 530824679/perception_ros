//
/******************************************************************************/
/*!
File name: tracking.h

Description:
This file define class of Tracking use to realize function of match and track.

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef PERCEPTION_ROS_TRACKING_H
#define PERCEPTION_ROS_TRACKING_H

// json include
#include <jsoncpp/json/json.h>

//opencv include
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
// local include
#include "common/logging.h"
#include "munkres.h"
#include "tracker.h"

// msg include
#include "perception_ros/ObjectInfoArray.h"

extern Logging logger;

class Tracking{
public:
    Tracking();
    ~Tracking();

    void Process(std::vector<BBox> bboxes, perception_ros::ObjectInfoArray &object_array,std::vector<InfoTracker>& trackerinfo);
    bool Init(Json::Value params, std::string key);
    float CalculateIou(const BBox& det, const Tracker& track);
    float CalculateRotateIOU(const BBox& det,const Tracker& track);
    void HungarianMatching(const std::vector<std::vector<float>>& iou_matrix, size_t nrows, size_t ncols, std::vector<std::vector<float>>& association);
    void AssociateDetectionsToTrackers(const std::vector<BBox> &bboxes,
                                       std::map<int, Tracker>& tracks,
                                       std::map<int, BBox>& matched,
                                       std::vector<BBox>& unmatched_det,
                                       float iou_threshold = 0.2);
    int track(std::map<int, Tracker> &tracks, std::vector<BBox> bboxes, int frame_index, int& current_id,
             perception_ros::ObjectInfoArray &object_array_msg,std::vector<InfoTracker>& trackerinfo);

private:
    std::map<int, Tracker> tracks_;
    int frame_index_;
    int current_id_;

    int max_coast_cycles_;
    int min_hits_;
    float min_confidence_;
};


#endif //PERCEPTION_ROS_TRACKING_H

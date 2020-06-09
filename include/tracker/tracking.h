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

// opencv include
#include <opencv2/core.hpp>

// json include
#include <jsoncpp/json/json.h>

// local include
#include "munkres.h"
#include "tracker.h"
#include "common/logging.h"

extern Logging logger;

class Tracking{
public:
    Tracking();
    ~Tracking();

    bool Init(Json::Value params, std::string key);
    float CalculateIou(const cv::Rect& det, const Tracker& track);
    void HungarianMatching(const std::vector<std::vector<float>>& iou_matrix, size_t nrows, size_t ncols, std::vector<std::vector<float>>& association);
    void AssociateDetectionsToTrackers(const std::vector<cv::Rect>& detection,
                                       std::map<int, Tracker>& tracks,
                                       std::map<int, cv::Rect>& matched,
                                       std::vector<cv::Rect>& unmatched_det,
                                       float iou_threshold = 0.2);
    int track(std::map<int, Tracker> &tracks, std::vector<cv::Rect> detections, int frame_index, int& current_ID);

private:
    int max_coast_cycles_;
    int min_hits_;
    float min_confidence_;
};


#endif //PERCEPTION_ROS_TRACKING_H

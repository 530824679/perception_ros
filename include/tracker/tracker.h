//
/******************************************************************************/
/*!
File name: tracker.h

Description:
This file define class of Tracker use to process of implementing the track.

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef PERCEPTION_ROS_TRACKER_H
#define PERCEPTION_ROS_TRACKER_H

// opencv include
#include <opencv2/core.hpp>

// local include
#include "kalman_filter.h"

class Tracker {
public:
    Tracker();
    ~Tracker();

    void Init(const cv::Rect& bbox);
    void Predict();
    void Update(const cv::Rect& bbox);
    cv::Rect GetStateAsBbox() const;
    float GetNIS() const;

    int GetCoastCycles();
    int GetHitStreak();

private:
    Eigen::VectorXd ConvertBboxToObservation(const cv::Rect& bbox) const;
    cv::Rect ConvertStateToBbox(const Eigen::VectorXd &state) const;

private:
    KalmanFilter kf_;
    int coast_cycles_;
    int hit_streak_;

};


#endif //PERCEPTION_ROS_TRACKER_H

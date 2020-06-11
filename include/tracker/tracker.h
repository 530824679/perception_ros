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

// local include
#include "kalman_filter.h"
#include "common/types.h"

class Tracker {
public:
    Tracker();
    ~Tracker();

    void Init(const BBox& bbox);
    void Predict();
    void Update(const BBox& bbox);
    BBox GetStateAsBbox() const;
    float GetNIS() const;

    int GetCoastCycles();
    int GetHitStreak();

private:
    Eigen::VectorXd ConvertBboxToObservation(const BBox& bbox) const;
    BBox ConvertStateToBbox(const Eigen::VectorXd &state) const;

private:
    KalmanFilter kf_;
    int coast_cycles_;
    int hit_streak_;

};


#endif //PERCEPTION_ROS_TRACKER_H

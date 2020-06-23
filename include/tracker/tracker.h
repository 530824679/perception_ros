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

#include <jsoncpp/json/json.h>
// local include
#include "common/logging.h"
#include "kalman_filter.h"
#include "common/types.h"
#include "velocity.h"

extern Logging logger;

class Tracker {
public:
    Tracker();
    ~Tracker();
    
    bool Init(Json::Value params, std::string key);
    void Init(const BBox& bbox);
    void Predict();
    void Update(const BBox& bbox);
    BBox GetStateAsBbox() const;
    VTracker GetStateAsVelocity() const;
    float GetNIS() const;

    int GetCoastCycles();
    int GetHitStreak();

    bool RejectOutlier(Velocity& publish_velocity_);

private:
    Eigen::VectorXd ConvertBboxToObservation(const BBox& bbox) const;
    BBox ConvertStateToBbox(const Eigen::VectorXd &state) const;
    VTracker ConvertStateToVelocity(const Velocity &publish_velocity_) const;


private:
    KalmanFilter kf_;
    Velocity publish_velocity_;

    int coast_cycles_;
    int hit_streak_;

    int lidar_rate_;
    float acc_threshold_;

};


#endif //PERCEPTION_ROS_TRACKER_H

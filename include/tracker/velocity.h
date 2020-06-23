//
// Created by linuxidc on 2020/4/14.
//

#include <queue>
#include "iostream"
#include <ros/ros.h>
#include "common/types.h"

class Velocity{
public:
    // Velocity(Center prev_center_,Center now_center_,Speed prev_speed_,Speed now_speed_,Speed sum_speed_,int year_):
    // prev_center_(0,0),now_center_(0,0),prev_speed_(0,0),now_speed_(0,0),sum_speed_(0,0),year_(0){};
    Velocity();
    ~Velocity();
    //float cal_velocity(std::queue<float>& velocity,float now_velocity,float& sum_velocity,int year,float prev_velocity);
    VTracker GetTrackVelocity();
    VTracker AssignVelocityValue(VTracker tra_velocity_);

    float CalculateVelocity(std::queue<float>& velocity,float now_velocity,float& sum_velocity);
public:

    std::queue<float> velocity_x_;
    std::queue<float> velocity_y_;
    VTracker tra_velocity_;


};

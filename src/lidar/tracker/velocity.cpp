//
// Created by linuxidc on 2020/4/29.
//

#include "tracker/velocity.h"

Velocity::Velocity(){
    tra_velocity_.p_x=0;
    tra_velocity_.p_y=0;
    tra_velocity_.pre_p_x=0;
    tra_velocity_.pre_p_y=0;
    tra_velocity_.pre_v_x=0;
    tra_velocity_.pre_v_y=0;
    tra_velocity_.v_x=0;
    tra_velocity_.v_y=0;
    tra_velocity_.sum_v_x=0;
    tra_velocity_.sum_v_y=0;
    tra_velocity_.year=0;
}

Velocity::~Velocity(){};

VTracker Velocity::GetTrackVelocity(){
    return tra_velocity_;
}

// VTracker Velocity::AssignVelocityValue(VTracker tra_velocity_){
//     VTracker tmp_vtracker;
//     tmp_vtracker.p_x=tra_velocity_.p_x;
//     tmp_vtracker.p_y=tra_velocity_.p_y;
//     tmp
// }

//float center_velocity::cal_velocity(std::queue<float> &velocity, float now_velocity, float &sum_velocity,int year,float prev_velocity) {
float Velocity::CalculateVelocity(std::queue<float> &velocity, float now_velocity, float &sum_velocity) {

    float tmp_velocity=0;
    
    if((tmp_velocity>100)&&(tmp_velocity<-100)){
       ROS_ERROR("The velocity is too large!");
        return 0;
    }

    velocity.push(now_velocity);
    sum_velocity+=now_velocity;
    
    if((sum_velocity>500)&&(sum_velocity<-500)){
       ROS_ERROR("The sum_velocity is too large!");
       return 0;
    }

    int size=velocity.size();
    if(size<6){
        tmp_velocity=sum_velocity/size;
    }else{
        sum_velocity-=velocity.front();
        tmp_velocity=sum_velocity/5;
        velocity.pop();
    }
    return tmp_velocity;
}

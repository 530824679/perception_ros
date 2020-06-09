//
/******************************************************************************/
/*!
File name: kalman_filter.h

Description:
This file define class of KalmanFilter to realize the implementation of kalman filter.

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef PERCEPTION_ROS_KALMAN_FILTER_H
#define PERCEPTION_ROS_KALMAN_FILTER_H

#include <iostream>
#include <eigen3/Eigen/Dense>

class KalmanFilter{
public:
    KalmanFilter(unsigned int num_states, unsigned int num_obs);
    virtual ~KalmanFilter() = default;

    virtual void Coast();
    void Predict();
    virtual Eigen::VectorXd PredictionToObservation(const Eigen::VectorXd &state);
    virtual void Update(const Eigen::VectorXd &z);
    float CalculateLogLikelihood(const Eigen::VectorXd& y, const Eigen::MatrixXd& S);

public:
    Eigen::VectorXd x_, x_predict_;
    Eigen::MatrixXd P_, P_predict_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;
    float NIS_;

private:
    unsigned int num_states_, num_obs_;
    float log_likelihood_delta_;
};

#endif //PERCEPTION_ROS_KALMAN_FILTER_H

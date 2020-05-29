/******************************************************************************/
/*!
File name: perception_node.cpp

Description:
This file runs the node's main function of perception

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#include "process/lidar_process.h"
#include "common/logging.h"

Logging logger;

int main(int argc, char **argv)
{
    logger.SetLevel(ERROR);

    try
    {
        //ros::init(argc, argv, "perception_node");
        //ros::NodeHandle nh;

        //LidarProcess node(nh, "/HDD_Disk/perception_ros/config/perception_params.json");
        LidarProcess node("/HDD_Disk/perception_ros/config/perception_params.json");

        logger.Log(INFO, "[%s]: Start Perception ROS loop.\n", __func__);

        //ros::spin();
    }
    catch (std::exception& e)
    {
        logger.Log(ERROR, "[%s]: EXCEPTION '[%d]'.\n", __func__, e.what());
        return 1;
    }
    catch (...)
    {
        logger.Log(ERROR, "[%s]: caught non-std EXCEPTION.\n", __func__);
        return 1;
    }

    return 0;
}
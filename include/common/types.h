//
// Created by chenwei on 20-5-28.
//

#ifndef PERCEPTION_ROS_TYPES_H
#define PERCEPTION_ROS_TYPES_H

enum LogLevel{
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    SILENCE
};

enum GridCellType{
    UNKNOW = 0,
    GROUND = 1,
    OBSTACLE = 2
};

#endif //PERCEPTION_ROS_TYPES_H

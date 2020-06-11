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

enum CameraAngle{
    XY,
    TopDown,
    Side,
    FPS
};

enum Category{
    UNKNOWN = 0,
    PEDESTRIAN = 1,
    MOTOR = 2,
    CAR = 3,
    TRUCK = 4
};

enum Moion{
    MOTION_UNKNOWN = 0,
    MOTION_MOVING = 1,
    MOTION_STATIONARY = 2
};

struct Color{
public:
    Color(float r, float g, float b):r_(r), g_(g), b_(b){};
    float GetR(){ return r_; };
    float GetG(){ return g_; };
    float GetB(){ return b_; };

private:
    float r_;
    float g_;
    float b_;
};

struct BBox{
    // center point
    float x;
    float y;
    float z;

    // dimension
    float dx;
    float dy;
    float dz;

    // angle
    float yaw;

    // centroid point
    float centroid_x;
    float centroid_y;
    float centroid_z;
};
#endif //PERCEPTION_ROS_TYPES_H

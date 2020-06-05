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
    float x;
    float y;
    float z;
    float dx;
    float dy;
    float dz;
    float yaw;

    float centroid_x;
    float centroid_y;
    float centroid_z;
};

#endif //PERCEPTION_ROS_TYPES_H

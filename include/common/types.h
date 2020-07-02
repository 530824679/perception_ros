//
// Created by chenwei on 20-5-28.
//
#ifndef PERCEPTION_ROS_TYPES_H
#define PERCEPTION_ROS_TYPES_H

#include "perception_ros/ObjectInfoArray.h"

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

struct BBox2D{
    float left_top_x;
    float left_top_y;
    float right_top_x;
    float right_top_y;
};

struct Speed{
    float v_x;
    float v_y;
};

struct Center{
    float p_x;
    float P_y;
};

struct VTracker{
    float pre_v_x;//the velocity of last frame
    float pre_v_y;
    float v_x;//the velocity of now frame
    float v_y;
    float sum_v_x;//the sum of velocity during five frames
    float sum_v_y;
    float pre_p_x;//the center point of last frame
    float pre_p_y;
    float p_x;//the center point of last frame
    float p_y;
    int year;//the age of tracker
};

struct InfoTracker{
    int id;
    
    float x;
    float y;
    float z;
    
    float width;
    float length;
    float height;

    float yaw;
};

struct Quaternion{//四元数
    float x;
    float y;
    float z;
    float w;
};

struct Point{
    float x;
    float y;
    float z;
};

struct Vector3{
    float x;
    float y;
    float z;
};

struct Pose{
    Point position;
    Quaternion orientation;
};

struct Twist{
    Vector3 linear;
    Vector3 angular;
};

struct UkfTracker{

    int header;
    int id;
    std::string label;
    float score;
    bool valid;
    //3Dbbox
    Pose pose;
    Vector3 dimensions;
    Vector3 variance;
    Twist velocity;
    Twist acceleration;

    bool pose_reliable;
    bool velocity_reliable;
    bool acceleration_reliable;

    ///2Dbbox
    std::string image_frame; // Image coordinate Frame,        Required if x,y,w,h defined
    int x;           // X coord in image space(pixel) of the initial point of the Rect
    int y;           // Y coord in image space(pixel) of the initial point of the Rect
    int width;      // Width of the Rect in pixels
    int height;     // Height of the Rect in pixels
    float angle;       // Angle [0 to 2*PI), allow rotated rects

    //Indicator information
    int indicator_state; // INDICATOR_LEFT = 0, INDICATOR_RIGHT = 1, INDICATOR_BOTH = 2, INDICATOR_NONE = 3
    //Behavior State of the Detected Object
    int behavior_state; // FORWARD_STATE = 0, STOPPING_STATE = 1, BRANCH_LEFT_STATE = 2, BRANCH_RIGHT_STATE = 3, YIELDING_STATE = 4, ACCELERATING_STATE = 5, SLOWDOWN_STATE = 6
};

#endif //PERCEPTION_ROS_TYPES_H

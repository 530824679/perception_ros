#ifndef _VISUALIZEDETECTEDOBJECTS_H_
#define _VISUALIZEDETECTEDOBJECTS_H_

#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <jsoncpp/json/json.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/Header.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "common/logging.h"
#include "perception_ros/DetectedObject.h"
#include "perception_ros/DetectedObjectArray.h"

#define __APP_NAME__ "visualize_detected_objects"

extern Logging logger;

class VisualizeDetectedObjects
{
private:
  const double arrow_height_;
  const double label_height_;
  const double object_max_linear_size_ = 50.;
  double object_speed_threshold_;
  double arrow_speed_threshold_;
  double marker_display_duration_;

  int marker_id_;

  std_msgs::ColorRGBA label_color_, box_color_, hull_color_, arrow_color_, centroid_color_, model_color_;

  std::string input_topic_, ros_namespace_;

  ros::NodeHandle node_handle_;
  ros::Subscriber subscriber_detected_objects_;

  ros::Publisher publisher_markers_;
  
  visualization_msgs::MarkerArray ObjectsToLabels(const perception_ros::DetectedObjectArray &in_objects);

  visualization_msgs::MarkerArray ObjectsToArrows(const perception_ros::DetectedObjectArray &in_objects);

  visualization_msgs::MarkerArray ObjectsToBoxes(const perception_ros::DetectedObjectArray &in_objects);

  visualization_msgs::MarkerArray ObjectsToModels(const perception_ros::DetectedObjectArray &in_objects);

  visualization_msgs::MarkerArray ObjectsToHulls(const perception_ros::DetectedObjectArray &in_objects);

  visualization_msgs::MarkerArray ObjectsToCentroids(const perception_ros::DetectedObjectArray &in_objects);

  std::string ColorToString(const std_msgs::ColorRGBA &in_color);

  bool IsObjectValid(const perception_ros::DetectedObject &in_object);

  float CheckColor(double value);

  float CheckAlpha(double value);

  std_msgs::ColorRGBA ParseColor(const std::vector<double> &in_color);

  public:

  VisualizeDetectedObjects();
  ~VisualizeDetectedObjects();
  bool Init(Json::Value params, std::string key);
  void run(const perception_ros::DetectedObjectArray in_objects,visualization_msgs::MarkerArray &visualization_markers);
};

#endif  // _VISUALIZEDETECTEDOBJECTS_H_

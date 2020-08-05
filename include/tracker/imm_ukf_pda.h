/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef OBJECT_TRACKING_IMM_UKF_JPDAF_H
#define OBJECT_TRACKING_IMM_UKF_JPDAF_H


#include <vector>
#include <chrono>
#include <stdio.h>

#include <jsoncpp/json/json.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>

#include "common/logging.h"
#include "perception_ros/DetectedObject.h"
#include "perception_ros/DetectedObjectArray.h"
#include "tracker/munkres.h"

#include "ukf.h"

extern Logging logger;

class ImmUkfPda
{
private:
  int target_id_;
  bool init_check_;
  double timestamp_;

  std::vector<UKF> targets_;

  // probabilistic data association params
  double gating_threshold_;
  double gate_probability_;
  double detection_probability_;

  // object association param
  int life_time_threshold_;

  // static classification param
  double static_velocity_threshold_;
  int static_num_history_threshold_;

  // switch sukf and ImmUkfPda
  bool use_sukf_;

  // whether if benchmarking tracking result
  // bool is_benchmark_;
  int frame_count_;

  double prevent_explosion_threshold_;


  double lane_direction_chi_threshold_;


 double merge_distance_threshold_;
 const double CENTROID_DISTANCE = 0.2;//distance to consider centroids the same  如果在这个直径范围内，就不认为是age新目标了，也就不会被追踪，当这个值为0时，
                                      //所有的检测输入都会被追踪


  geometry_msgs::Pose getTransformedPose(const geometry_msgs::Pose& in_pose,
                                                const tf::StampedTransform& tf_stamp);

  bool updateNecessaryTransform();

  void measurementValidation(const perception_ros::DetectedObjectArray& input, UKF& target, const bool second_init,
                             const Eigen::VectorXd& max_det_z, const Eigen::MatrixXd& max_det_s,
                             std::vector<perception_ros::DetectedObject>& object_vec, std::vector<bool>& matching_vec);
  perception_ros::DetectedObject getNearestObject(UKF& target,
                                                 const std::vector<perception_ros::DetectedObject>& object_vec);
  void updateBehaviorState(const UKF& target, const bool use_sukf, perception_ros::DetectedObject& object);

  void initTracker(const perception_ros::DetectedObjectArray& input, double timestamp);
  void secondInit(UKF& target, const std::vector<perception_ros::DetectedObject>& object_vec, double dt);

  void updateTrackingNum(const std::vector<perception_ros::DetectedObject>& object_vec, UKF& target);

  bool probabilisticDataAssociation(const perception_ros::DetectedObjectArray& input, const double dt,
                                    std::vector<bool>& matching_vec,
                                    std::vector<perception_ros::DetectedObject>& object_vec, UKF& target);

  //float CalculateIou(const BBox& det, const Tracker& track);
  void HungarianMatching(const std::vector<std::vector<float>>& iou_matrix, size_t nrows, size_t ncols, std::vector<std::vector<float>>& association);

  bool hungarianDataAssocaition(const perception_ros::DetectedObjectArray& input, const double dt,
                                    std::vector<bool>& matching_vec,
                                    std::vector<perception_ros::DetectedObject>& object_vec, UKF& target);

  void makeNewTargets(const double timestamp, const perception_ros::DetectedObjectArray& input,
                      const std::vector<bool>& matching_vec);

  void staticClassification();

  void makeOutput(const perception_ros::DetectedObjectArray& input,
                  const std::vector<bool>& matching_vec,
                  perception_ros::DetectedObjectArray& detected_objects_output);

  void removeUnnecessaryTarget();

  void dumpResultText(perception_ros::DetectedObjectArray& detected_objects);

  void tracker(const perception_ros::DetectedObjectArray& transformed_input,
               perception_ros::DetectedObjectArray& detected_objects_output);

  bool updateDirection(const double smallest_nis, const perception_ros::DetectedObject& in_object,
                           perception_ros::DetectedObject& out_object, UKF& target);

  bool storeObjectWithNearestLaneDirection(const perception_ros::DetectedObject& in_object,
                                       perception_ros::DetectedObject& out_object);

  perception_ros::DetectedObjectArray
  removeRedundantObjects(const perception_ros::DetectedObjectArray& in_detected_objects,
                         const std::vector<size_t> in_tracker_indices);

  perception_ros::DetectedObjectArray
  forwardNonMatchedObject(const perception_ros::DetectedObjectArray& tmp_objects,
                          const perception_ros::DetectedObjectArray&  input,
                          const std::vector<bool>& matching_vec);

  bool
  arePointsClose(const geometry_msgs::Point& in_point_a,
                 const geometry_msgs::Point& in_point_b,
                 float in_radius);

  bool
  arePointsEqual(const geometry_msgs::Point& in_point_a,
                 const geometry_msgs::Point& in_point_b);

  bool
  isPointInPool(const std::vector<geometry_msgs::Point>& in_pool,
                const geometry_msgs::Point& in_point);

  void updateTargetWithAssociatedObject(const std::vector<perception_ros::DetectedObject>& object_vec,
                                        UKF& target);

public:
  ImmUkfPda();
  ~ImmUkfPda();
  bool Init(Json::Value params, std::string key);
  void run(const perception_ros::DetectedObjectArray input,std::vector<InfoTracker> &trackerinfo,
              perception_ros::DetectedObjectArray &detected_objects_output);
};

#endif /* OBJECT_TRACKING_IMM_UKF_JPDAF_H */

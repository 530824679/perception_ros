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


#include <tracker/imm_ukf_pda.h>

ImmUkfPda::ImmUkfPda():
target_id_(0),
init_check_(false),
frame_count_(0){
}

ImmUkfPda::~ImmUkfPda(){

}

bool ImmUkfPda::Init(Json::Value params, std::string key){
  //std::cout<<"params"<<params<<std::endl;
    if(params.isMember(key)) {
        Json::Value ukf_tracker_param = params[key];

        if (ukf_tracker_param.isMember("life_time_threshold") && ukf_tracker_param["life_time_threshold"].isInt()) {
            life_time_threshold_ = ukf_tracker_param["life_time_threshold"].asInt();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named life_time_threshold in the tracker config.\n", __func__);
            return false;
        }
        if (ukf_tracker_param.isMember("gating_threshold") && ukf_tracker_param["gating_threshold"].isDouble()) {
            gating_threshold_ = ukf_tracker_param["gating_threshold"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named gating_threshold in the tracker config.\n", __func__);
            return false;
        }
        if (ukf_tracker_param.isMember("gate_probability") && ukf_tracker_param["gate_probability"].isDouble()) {
            gate_probability_ = ukf_tracker_param["gate_probability"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named gate_probability in the tracker config.\n", __func__);
            return false;
        }
        if (ukf_tracker_param.isMember("detection_probability") && ukf_tracker_param["detection_probability"].isDouble()) {
            detection_probability_ = ukf_tracker_param["detection_probability"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named detection_probability in the tracker config.\n", __func__);
            return false;
        }
        if (ukf_tracker_param.isMember("static_velocity_threshold") && ukf_tracker_param["static_velocity_threshold"].isDouble()) {
            static_velocity_threshold_ = ukf_tracker_param["static_velocity_threshold"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named static_velocity_threshold in the tracker config.\n", __func__);
            return false;
        }
        if (ukf_tracker_param.isMember("static_num_history_threshold") && ukf_tracker_param["static_num_history_threshold"].isDouble()) {
            static_num_history_threshold_ = ukf_tracker_param["static_num_history_threshold"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named static_num_history_threshold in the tracker config.\n", __func__);
            return false;
        }
        if (ukf_tracker_param.isMember("prevent_explosion_threshold") && ukf_tracker_param["prevent_explosion_threshold"].isInt()) {
            prevent_explosion_threshold_ = ukf_tracker_param["prevent_explosion_threshold"].asInt();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named prevent_explosion_threshold in the tracker config.\n", __func__);
            return false;
        }
        if (ukf_tracker_param.isMember("merge_distance_threshold") && ukf_tracker_param["merge_distance_threshold"].isDouble()) {
            merge_distance_threshold_ = ukf_tracker_param["merge_distance_threshold"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named merge_distance_threshold in the tracker config.\n", __func__);
            return false;
        }
        if (ukf_tracker_param.isMember("use_sukf") && ukf_tracker_param["use_sukf"].isDouble()) {
            use_sukf_ = ukf_tracker_param["use_sukf"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named use_sukf in the tracker config.\n", __func__);
            return false;
        }
    }else{
        logger.Log(ERROR, "[%s]: Has not key named tracker in the perception config.\n", __func__);
    }
}

void ImmUkfPda::run(const perception_ros::DetectedObjectArray input,std::vector<InfoTracker> &trackerinfo,perception_ros::DetectedObjectArray &detected_objects_output)
{
  
  tracker(input, detected_objects_output);
   
  for(int i=0;i<detected_objects_output.objects.size();i++){
    InfoTracker tracker;
    tracker.id=detected_objects_output.objects[i].id;
    tracker.x=detected_objects_output.objects[i].pose.position.x;
    tracker.y=detected_objects_output.objects[i].pose.position.y;
    tracker.z=detected_objects_output.objects[i].pose.position.z;
    tracker.length=detected_objects_output.objects[i].dimensions.x;
    tracker.width=detected_objects_output.objects[i].dimensions.y;
    tracker.height=detected_objects_output.objects[i].dimensions.z;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(detected_objects_output.objects[i].pose.orientation,quat);
    double roll=0,pitch=0,yaw=0;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    tracker.yaw=M_PI/2-yaw;
    trackerinfo.push_back(tracker);
  }

}


bool ImmUkfPda::updateNecessaryTransform()
{
  bool success = true;
  return success;
}


geometry_msgs::Pose ImmUkfPda::getTransformedPose(const geometry_msgs::Pose& in_pose,
                                                  const tf::StampedTransform& tf_stamp)
{
  tf::Transform transform;
  geometry_msgs::PoseStamped out_pose;
  transform.setOrigin(tf::Vector3(in_pose.position.x, in_pose.position.y, in_pose.position.z));
  transform.setRotation(
      tf::Quaternion(in_pose.orientation.x, in_pose.orientation.y, in_pose.orientation.z, in_pose.orientation.w));
  geometry_msgs::PoseStamped pose_out;
  tf::poseTFToMsg(tf_stamp * transform, out_pose.pose);
  return out_pose.pose;
}

void ImmUkfPda::measurementValidation(const perception_ros::DetectedObjectArray& input, UKF& target,
                                      const bool second_init, const Eigen::VectorXd& max_det_z,
                                      const Eigen::MatrixXd& max_det_s,
                                      std::vector<perception_ros::DetectedObject>& object_vec,
                                      std::vector<bool>& matching_vec)
{
  // alert: different from original imm-pda filter, here picking up most likely measurement
  // if making it allows to have more than one measurement, you will see non semipositive definite covariance
  bool exists_smallest_nis_object = false;
  double smallest_nis = std::numeric_limits<double>::max();
  int smallest_nis_ind = 0;
  for (size_t i = 0; i < input.objects.size(); i++)
  {
    double x = input.objects[i].pose.position.x;
    double y = input.objects[i].pose.position.y;

    Eigen::VectorXd meas = Eigen::VectorXd(2);
    meas << x, y;

    Eigen::VectorXd diff = meas - max_det_z;
    double nis = diff.transpose() * max_det_s.inverse() * diff;

    if (nis < gating_threshold_)
    {
      if (nis < smallest_nis)
      {
        smallest_nis = nis;
        target.object_ = input.objects[i];
        smallest_nis_ind = i;
        exists_smallest_nis_object = true;
      }
    }
  }
  if (exists_smallest_nis_object)
  {
    matching_vec[smallest_nis_ind] = true;
    if (false && false)
    {
      perception_ros::DetectedObject direction_updated_object;
      bool use_direction_meas =
          updateDirection(smallest_nis, target.object_, direction_updated_object, target);
      if (use_direction_meas)
      {
        object_vec.push_back(direction_updated_object);
      }
      else
      {
        object_vec.push_back(target.object_);
      }
    }
    else
    {
      object_vec.push_back(target.object_);
    }
  }
}

bool ImmUkfPda::updateDirection(const double smallest_nis, const perception_ros::DetectedObject& in_object,
                                    perception_ros::DetectedObject& out_object, UKF& target)
{
  bool use_lane_direction = false;
  target.is_direction_cv_available_ = false;
  target.is_direction_ctrv_available_ = false;

  return use_lane_direction;
}


void ImmUkfPda::updateTargetWithAssociatedObject(const std::vector<perception_ros::DetectedObject>& object_vec,
                                                 UKF& target)
{
  target.lifetime_++;
  if (!target.object_.label.empty() && target.object_.label !="unknown")
  {
    target.label_ = target.object_.label;
  }
  updateTrackingNum(object_vec, target);
  if (target.tracking_num_ == TrackingState::Stable || target.tracking_num_ == TrackingState::Occlusion)
  {
    target.is_stable_ = true;
  }
}

void ImmUkfPda::updateBehaviorState(const UKF& target, const bool use_sukf, perception_ros::DetectedObject& object)
{
  if(use_sukf)
  {
    object.behavior_state = MotionModel::CTRV;
  }
  else if (target.mode_prob_cv_ > target.mode_prob_ctrv_ && target.mode_prob_cv_ > target.mode_prob_rm_)
  {
    object.behavior_state = MotionModel::CV;
  }
  else if (target.mode_prob_ctrv_ > target.mode_prob_cv_ && target.mode_prob_ctrv_ > target.mode_prob_rm_)
  {
    object.behavior_state = MotionModel::CTRV;
  }
  else
  {
    object.behavior_state = MotionModel::RM;
  }
}

void ImmUkfPda::initTracker(const perception_ros::DetectedObjectArray& input, double timestamp)
{
  for (size_t i = 0; i < input.objects.size(); i++)
  {
    double px = input.objects[i].pose.position.x;
    double py = input.objects[i].pose.position.y;
    Eigen::VectorXd init_meas = Eigen::VectorXd(2);
    init_meas << px, py;

    UKF ukf;
    ukf.initialize(init_meas, timestamp, target_id_);
    targets_.push_back(ukf);
    target_id_++;
  }
  timestamp_ = timestamp;
  init_check_ = true;
}

void ImmUkfPda::secondInit(UKF& target, const std::vector<perception_ros::DetectedObject>& object_vec, double dt)
{
  if (object_vec.size() == 0)
  {
    target.tracking_num_ = TrackingState::Die;
    return;
  }
  // record init measurement for env classification
  target.init_meas_ << target.x_merge_(0), target.x_merge_(1);

  // state update
  double target_x = object_vec[0].pose.position.x;
  double target_y = object_vec[0].pose.position.y;
  double target_diff_x = target_x - target.x_merge_(0);
  double target_diff_y = target_y - target.x_merge_(1);
  double target_yaw = atan2(target_diff_y, target_diff_x);
  double dist = sqrt(target_diff_x * target_diff_x + target_diff_y * target_diff_y);
  double target_v = dist / dt;

  while (target_yaw > M_PI)
    target_yaw -= 2. * M_PI;
  while (target_yaw < -M_PI)
    target_yaw += 2. * M_PI;

  target.x_merge_(0) = target.x_cv_(0) = target.x_ctrv_(0) = target.x_rm_(0) = target_x;
  target.x_merge_(1) = target.x_cv_(1) = target.x_ctrv_(1) = target.x_rm_(1) = target_y;
  target.x_merge_(2) = target.x_cv_(2) = target.x_ctrv_(2) = target.x_rm_(2) = target_v;
  target.x_merge_(3) = target.x_cv_(3) = target.x_ctrv_(3) = target.x_rm_(3) = target_yaw;

  target.tracking_num_++;
  return;
}

void ImmUkfPda::updateTrackingNum(const std::vector<perception_ros::DetectedObject>& object_vec, UKF& target)
{
  if (object_vec.size() > 0)
  {
    if (target.tracking_num_ < TrackingState::Stable)
    {
      target.tracking_num_++;
    }
    else if (target.tracking_num_ == TrackingState::Stable)
    {
      target.tracking_num_ = TrackingState::Stable;
    }
    else if (target.tracking_num_ >= TrackingState::Stable && target.tracking_num_ < TrackingState::Lost)
    {
      target.tracking_num_ = TrackingState::Stable;
    }
    else if (target.tracking_num_ == TrackingState::Lost)
    {
      target.tracking_num_ = TrackingState::Die;
    }
  }
  else
  {
    if (target.tracking_num_ < TrackingState::Stable)
    {
      target.tracking_num_ = TrackingState::Die;
    }
    else if (target.tracking_num_ >= TrackingState::Stable && target.tracking_num_ < TrackingState::Lost)
    {
      target.tracking_num_++;
    }
    else if (target.tracking_num_ == TrackingState::Lost)
    {
      target.tracking_num_ = TrackingState::Die;
    }
  }

  return;
}

bool ImmUkfPda::probabilisticDataAssociation(const perception_ros::DetectedObjectArray& input, const double dt,
                                             std::vector<bool>& matching_vec,
                                             std::vector<perception_ros::DetectedObject>& object_vec, UKF& target)
{
  double det_s = 0;
  Eigen::VectorXd max_det_z;
  Eigen::MatrixXd max_det_s;
  bool success = true;
  
  if (use_sukf_)
  {
    max_det_z = target.z_pred_ctrv_;
    max_det_s = target.s_ctrv_;
    det_s = max_det_s.determinant();
  }
  else
  {
    // find maxDetS associated with predZ
    target.findMaxZandS(max_det_z, max_det_s);
    det_s = max_det_s.determinant();
  }

  // prevent ukf not to explode
  if (std::isnan(det_s) || det_s > prevent_explosion_threshold_)
  {
    target.tracking_num_ = TrackingState::Die;
    success = false;
    return success;
  }

  bool is_second_init;
  if (target.tracking_num_ == TrackingState::Init)
  {
    is_second_init = true;
  }
  else
  {
    is_second_init = false;
  }

  // measurement gating
  measurementValidation(input, target, is_second_init, max_det_z, max_det_s, object_vec, matching_vec);

  // second detection for a target: update v and yaw
  if (is_second_init)
  {
    secondInit(target, object_vec, dt);
    success = false;
    return success;
  }

  updateTargetWithAssociatedObject(object_vec, target);

  if (target.tracking_num_ == TrackingState::Die)
  {
    success = false;
    return success;
  }
  return success;
}

void ImmUkfPda::makeNewTargets(const double timestamp, const perception_ros::DetectedObjectArray& input,
                               const std::vector<bool>& matching_vec)
{
  for (size_t i = 0; i < input.objects.size(); i++)
  {
    if (matching_vec[i] == false)
    {
      double px = input.objects[i].pose.position.x;
      double py = input.objects[i].pose.position.y;
      Eigen::VectorXd init_meas = Eigen::VectorXd(2);
      init_meas << px, py;

      UKF ukf;
      ukf.initialize(init_meas, timestamp, target_id_);
      ukf.object_ = input.objects[i];
      targets_.push_back(ukf);
      target_id_++;
    }
  }
}

void ImmUkfPda::staticClassification()
{
  for (size_t i = 0; i < targets_.size(); i++)
  {
    // targets_[i].x_merge_(2) is referred for estimated velocity
    double current_velocity = std::abs(targets_[i].x_merge_(2));
    targets_[i].vel_history_.push_back(current_velocity);
    if (targets_[i].tracking_num_ == TrackingState::Stable && targets_[i].lifetime_ > life_time_threshold_)
    {
      int index = 0;
      double sum_vel = 0;
      double avg_vel = 0;
      for (auto rit = targets_[i].vel_history_.rbegin(); index < static_num_history_threshold_; ++rit)
      {
        index++;
        sum_vel += *rit;
      }
      avg_vel = double(sum_vel / static_num_history_threshold_);

      if(avg_vel < static_velocity_threshold_ && current_velocity < static_velocity_threshold_)
      {
        targets_[i].is_static_ = true;
      }
    }
  }
}

bool
ImmUkfPda::arePointsClose(const geometry_msgs::Point& in_point_a,
                                const geometry_msgs::Point& in_point_b,
                                float in_radius)
{
  return (fabs(in_point_a.x - in_point_b.x) <= in_radius) && (fabs(in_point_a.y - in_point_b.y) <= in_radius);
}

bool
ImmUkfPda::arePointsEqual(const geometry_msgs::Point& in_point_a,
                               const geometry_msgs::Point& in_point_b)
{
  return arePointsClose(in_point_a, in_point_b, CENTROID_DISTANCE);
}

bool
ImmUkfPda::isPointInPool(const std::vector<geometry_msgs::Point>& in_pool,
                          const geometry_msgs::Point& in_point)
{
  for(size_t j=0; j<in_pool.size(); j++)//check the point that is really in_pool.
  {
    if (arePointsEqual(in_pool[j], in_point))
    {
      return true;
    }
  }
  return false;
}

perception_ros::DetectedObjectArray
ImmUkfPda::removeRedundantObjects(const perception_ros::DetectedObjectArray& in_detected_objects,
                            const std::vector<size_t> in_tracker_indices)
{
  if (in_detected_objects.objects.size() != in_tracker_indices.size())//for exception
    return in_detected_objects;
  
  //std::cout<<"in_detected_objects.objects.size()"<<in_detected_objects.objects.size()<<std::endl;

  perception_ros::DetectedObjectArray resulting_objects;
  resulting_objects.header = in_detected_objects.header;

  std::vector<geometry_msgs::Point> centroids;
  //create unique points
  for(size_t i=0; i<in_detected_objects.objects.size(); i++)
  {
    if(!isPointInPool(centroids, in_detected_objects.objects[i].pose.position))
    {
      centroids.push_back(in_detected_objects.objects[i].pose.position);//if the point is not in pool, push them into pool.
      //std::cout<<"pose.position is:"<<in_detected_objects.objects[i].pose.position<<std::endl;
    }
  }
  //assign objects to the points
  std::vector<std::vector<size_t>> matching_objects(centroids.size());
  for(size_t k=0; k<in_detected_objects.objects.size(); k++)
  {
    const auto& object=in_detected_objects.objects[k];
    for(size_t i=0; i< centroids.size(); i++)
    {
      if (arePointsClose(object.pose.position, centroids[i],merge_distance_threshold_))
      {
        matching_objects[i].push_back(k);//store index of matched object to this point
      }
    }
  }
  //get oldest object on each point
  for(size_t i=0; i< matching_objects.size(); i++)
  {
    size_t oldest_object_index = 0;
    int oldest_lifespan = -1;
    std::string best_label;
    for(size_t j=0; j<matching_objects[i].size(); j++)
    {
      size_t current_index = matching_objects[i][j];
      int current_lifespan = targets_[in_tracker_indices[current_index]].lifetime_;
      if (current_lifespan > oldest_lifespan)
      {
        oldest_lifespan = current_lifespan;
        oldest_object_index = current_index;
      }
      if (!targets_[in_tracker_indices[current_index]].label_.empty() &&
        targets_[in_tracker_indices[current_index]].label_ != "unknown")
      {
        best_label = targets_[in_tracker_indices[current_index]].label_;
      }
    }
    // delete nearby targets except for the oldest target
    for(size_t j=0; j<matching_objects[i].size(); j++)
    {
      size_t current_index = matching_objects[i][j];
      if(current_index != oldest_object_index)
      {
        targets_[in_tracker_indices[current_index]].tracking_num_= TrackingState::Die;
      }
    }
    perception_ros::DetectedObject best_object;
    best_object = in_detected_objects.objects[oldest_object_index];
    if (best_label != "unknown"
        && !best_label.empty())
    {
      best_object.label = best_label;
    }

    resulting_objects.objects.push_back(best_object);
  }

  return resulting_objects;

}

void ImmUkfPda::makeOutput(const perception_ros::DetectedObjectArray& input,
                           const std::vector<bool> &matching_vec,
                           perception_ros::DetectedObjectArray& detected_objects_output)
{
  perception_ros::DetectedObjectArray tmp_objects;
  tmp_objects.header = input.header;
  std::vector<size_t> used_targets_indices;
  for (size_t i = 0; i < targets_.size(); i++)
  {

    double tx = targets_[i].x_merge_(0);
    double ty = targets_[i].x_merge_(1);

    double tv = targets_[i].x_merge_(2);
    double tyaw = targets_[i].x_merge_(3);
    double tyaw_rate = targets_[i].x_merge_(4);

    while (tyaw > M_PI)
      tyaw -= 2. * M_PI;
    while (tyaw < -M_PI)
      tyaw += 2. * M_PI;

    tf::Quaternion q = tf::createQuaternionFromYaw(tyaw);

    perception_ros::DetectedObject dd;
    dd = targets_[i].object_;
    dd.id = targets_[i].ukf_id_;
    //std::cout<<"targets id is:"<<dd.id<<std::endl;
    dd.velocity.linear.x = tv;
    //std::cout<<"tv is:"<<tv<<std::endl;
    dd.acceleration.linear.y = tyaw_rate;
    dd.velocity_reliable = targets_[i].is_stable_;
    dd.pose_reliable = targets_[i].is_stable_;


    if (!targets_[i].is_static_ && targets_[i].is_stable_)
    {
      // Aligh the longest side of dimentions with the estimated orientation
      if(targets_[i].object_.dimensions.x < targets_[i].object_.dimensions.y)
      {
        dd.dimensions.x = targets_[i].object_.dimensions.y;
        dd.dimensions.y = targets_[i].object_.dimensions.x;
      }

      dd.pose.position.x = tx;
      dd.pose.position.y = ty;

      if (!std::isnan(q[0]))
        dd.pose.orientation.x = q[0];
      if (!std::isnan(q[1]))
        dd.pose.orientation.y = q[1];
      if (!std::isnan(q[2]))
        dd.pose.orientation.z = q[2];
      if (!std::isnan(q[3]))
        dd.pose.orientation.w = q[3];
    }
    updateBehaviorState(targets_[i], true, dd);//choose the mode of ukf 

    if (targets_[i].is_stable_ || (targets_[i].tracking_num_ >= TrackingState::Init &&
                                   targets_[i].tracking_num_ < TrackingState::Stable))
    {
      tmp_objects.objects.push_back(dd);
      //std::cout<<"After filtering, the id is:"<<dd.id<<std::endl;
      used_targets_indices.push_back(i);
    }
  }
  detected_objects_output = removeRedundantObjects(tmp_objects, used_targets_indices);
}

void ImmUkfPda::removeUnnecessaryTarget()
{
  std::vector<UKF> temp_targets;
  for (size_t i = 0; i < targets_.size(); i++)
  {
    if (targets_[i].tracking_num_ != TrackingState::Die)
    {
      temp_targets.push_back(targets_[i]);
    }
  }
  std::vector<UKF>().swap(targets_);
  targets_ = temp_targets;
}


void ImmUkfPda::tracker(const perception_ros::DetectedObjectArray& input,
                        perception_ros::DetectedObjectArray& detected_objects_output)
{
  double timestamp = input.header.stamp.toSec();
  std::vector<bool> matching_vec(input.objects.size(), false);
  //std::cout<<"input object size is:"<<input.objects.size()<<std::endl;

  if (!init_check_)
  {
    //std::cout<<"init_check_:"<<init_check_<<std::endl;
    initTracker(input, timestamp);
    makeOutput(input, matching_vec, detected_objects_output);
    return;
  }

  double dt = (timestamp - timestamp_);
  timestamp_ = timestamp;
 

   //std::cout<<"targets' size is:"<<targets_.size()<<std::endl;
  // start UKF process
  for (size_t i = 0; i < targets_.size(); i++)
  {
    targets_[i].is_stable_ = false;
    targets_[i].is_static_ = false;
    //std::cout<<"tracking_num_ is:"<<targets_[i].tracking_num_<<std::endl;
    if (targets_[i].tracking_num_ == TrackingState::Die)
    {
      continue;
    }

    // prevent ukf not to explode
    //std::cout<<"prevent_explosion_threshold_"<<prevent_explosion_threshold_<<std::endl;
    if (targets_[i].p_merge_.determinant() > prevent_explosion_threshold_ ||
        targets_[i].p_merge_(4, 4) > prevent_explosion_threshold_)
    {
      targets_[i].tracking_num_ = TrackingState::Die;
      continue;
    }
    //std::cout<<"tracking_num_ is:"<<targets_[i].tracking_num_<<std::endl;
    targets_[i].prediction(use_sukf_, false, dt);

    std::vector<perception_ros::DetectedObject> object_vec;
    bool success = probabilisticDataAssociation(input, dt, matching_vec, object_vec, targets_[i]);
    //std::cout<<"success"<<success<<std::endl;
    if (!success)
    {
      continue;
    }

    targets_[i].update(use_sukf_,  detection_probability_, gate_probability_, gating_threshold_, object_vec);
  }
  // end UKF process

  // making new ukf target for no data association objects
  makeNewTargets(timestamp, input, matching_vec);

  // static dynamic classification
  staticClassification();

  // making output for visualization
  makeOutput(input, matching_vec, detected_objects_output);
  //std::cout<<"detected_objects_output:"<<detected_objects_output<<std::endl;

  // remove unnecessary ukf object
  removeUnnecessaryTarget();
}

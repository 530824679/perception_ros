#include "tracker/tracker.h"

Tracker::Tracker() : kf_(8, 4),publish_velocity_(),coast_cycles_(0), hit_streak_(0),lidar_rate_(0),acc_threshold_(0){
    // state - center_x, center_y, length, width, v_cx, v_cy, v_length, v_width
    kf_.F_ <<
           1, 0, 0, 0, 1, 0, 0, 0,
            0, 1, 0, 0, 0, 1, 0, 0,
            0, 0, 1, 0, 0, 0, 1, 0,
            0, 0, 0, 1, 0, 0, 0, 1,
            0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 1;

    // Give high uncertainty to the unobservable initial velocities
    kf_.P_ <<
           10, 0, 0, 0, 0, 0, 0, 0,
            0, 10, 0, 0, 0, 0, 0, 0,
            0, 0, 10, 0, 0, 0, 0, 0,
            0, 0, 0, 10, 0, 0, 0, 0,
            0, 0, 0, 0, 10000, 0, 0, 0,
            0, 0, 0, 0, 0, 10000, 0, 0,
            0, 0, 0, 0, 0, 0, 10000, 0,
            0, 0, 0, 0, 0, 0, 0, 10000;

    kf_.H_ <<
           1, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0, 0;

    kf_.Q_ <<
           1, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0.01, 0, 0, 0,
            0, 0, 0, 0, 0, 0.01, 0, 0,
            0, 0, 0, 0, 0, 0, 0.0001, 0,
            0, 0, 0, 0, 0, 0, 0, 0.0001;

    kf_.R_ <<
           1, 0, 0,  0,
            0, 1, 0,  0,
            0, 0, 10, 0,
            0, 0, 0,  10;
}

Tracker::~Tracker(){

}
bool Tracker::Init(Json::Value params, std::string key){
        if(params.isMember(key)) {
        Json::Value tracker_param = params[key];

        if (tracker_param.isMember("lidar_rate") && tracker_param["lidar_rate"].isInt()) {
            lidar_rate_ = tracker_param["lidar_rate"].asInt();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named lidar_rate in the tracker config.\n", __func__);
            return false;
        }

        if (tracker_param.isMember("acceleration_threshold") && tracker_param["acceleration_threshold"].isInt()) {
            acc_threshold_ = tracker_param["acceleration_threshold"].asInt();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named acceleration_threshold in the tracker config.\n", __func__);
            return false;
        }
    }else{
        logger.Log(ERROR, "[%s]: Has not key named tracker in the perception config.\n", __func__);
    }
}

void Tracker::Predict() {
    kf_.Predict();

    // hit streak count will be reset
    if (coast_cycles_ > 0){
        hit_streak_ = 0;
    }

    // accumulate coast cycle count
    coast_cycles_++;
}

void Tracker::Update(const BBox& bbox) {
    coast_cycles_ = 0;
    hit_streak_++;

    Eigen::VectorXd observation = ConvertBboxToObservation(bbox);
    publish_velocity_.tra_velocity_.pre_p_x=publish_velocity_.tra_velocity_.p_x;
    publish_velocity_.tra_velocity_.pre_p_y=publish_velocity_.tra_velocity_.p_y;
    publish_velocity_.tra_velocity_.p_x=bbox.x;
    publish_velocity_.tra_velocity_.p_y=bbox.y;
    publish_velocity_.tra_velocity_.year++;

    float tmp_v_x=(publish_velocity_.tra_velocity_.p_x-publish_velocity_.tra_velocity_.pre_p_x)*10;
    float tmp_v_y=(publish_velocity_.tra_velocity_.p_y-publish_velocity_.tra_velocity_.pre_p_y)*10;

    publish_velocity_.tra_velocity_.pre_v_x=publish_velocity_.tra_velocity_.v_x;
    publish_velocity_.tra_velocity_.pre_v_y=publish_velocity_.tra_velocity_.v_y;
    
    //*It needs to check the coordinate of the input box. Make sure the same coordiante.
    publish_velocity_.tra_velocity_.v_x=publish_velocity_.CalculateVelocity(publish_velocity_.velocity_x_,
                                                                         tmp_v_x,publish_velocity_.tra_velocity_.sum_v_x);
    publish_velocity_.tra_velocity_.v_y=publish_velocity_.CalculateVelocity(publish_velocity_.velocity_y_,
                                                                         tmp_v_y,publish_velocity_.tra_velocity_.sum_v_y);
    if(Tracker::RejectOutlier(publish_velocity_)){
        logger.Log(WARNING, "[%s]:The acceleration is too large! Need to check!\n", __func__);
    };

    kf_.Update(observation);
}

// Create and initialize new trackers for unmatched detections, with initial bounding box
void Tracker::Init(const BBox &bbox) {
    kf_.x_.head(4) << ConvertBboxToObservation(bbox);
    publish_velocity_.tra_velocity_.pre_p_x=0;
    publish_velocity_.tra_velocity_.pre_p_y=0;
    publish_velocity_.tra_velocity_.p_x=bbox.x;
    publish_velocity_.tra_velocity_.p_y=bbox.y;
    publish_velocity_.tra_velocity_.year+=1;
    
    publish_velocity_.tra_velocity_.v_x=0;
    publish_velocity_.tra_velocity_.v_y=0;
    publish_velocity_.tra_velocity_.pre_v_x=0;
    publish_velocity_.tra_velocity_.pre_v_y=0;

    publish_velocity_.velocity_x_;
    publish_velocity_.velocity_y_;
    hit_streak_++;
}

BBox Tracker::GetStateAsBbox() const{
    return ConvertStateToBbox(kf_.x_);
}

VTracker Tracker::GetStateAsVelocity() const{
    return ConvertStateToVelocity(publish_velocity_);
}

float Tracker::GetNIS() const {
    return kf_.NIS_;
}

Eigen::VectorXd Tracker::ConvertBboxToObservation(const BBox& bbox) const{
    Eigen::VectorXd observation = Eigen::VectorXd::Zero(4);
    auto length = static_cast<float>(bbox.dx);
    auto width = static_cast<float>(bbox.dy);

    float center_x = bbox.x;
    float center_y = bbox.y;

    observation << center_x, center_y, length, width;
    return observation;
}

BBox Tracker::ConvertStateToBbox(const Eigen::VectorXd &state) const{
    BBox box;
    box.x = static_cast<int>(state[0]);
    box.y = static_cast<int>(state[1]);

    box.dx = static_cast<int>(state[2]);
    box.dy = static_cast<int>(state[3]);

    return box;
}

int Tracker::GetCoastCycles(){
    return coast_cycles_;
}

int Tracker::GetHitStreak(){
    return hit_streak_;
}

bool Tracker::RejectOutlier(Velocity& publish_velocity_){
    if(publish_velocity_.tra_velocity_.year>1){
        float acceleration_x=(publish_velocity_.tra_velocity_.v_x-publish_velocity_.tra_velocity_.pre_v_x)*lidar_rate_;
        float acceleration_y=(publish_velocity_.tra_velocity_.v_y-publish_velocity_.tra_velocity_.pre_v_y)*lidar_rate_;

        if((acceleration_x>acc_threshold_)&&(acceleration_y>acc_threshold_)){
            publish_velocity_.tra_velocity_.v_x=0;
            publish_velocity_.tra_velocity_.v_y=0;
            return true;
        }
    }
    return false;
}

VTracker Tracker::ConvertStateToVelocity(const Velocity& publish_velocity_)const{
    VTracker output;
    output.pre_p_x=publish_velocity_.tra_velocity_.pre_p_x;
    output.pre_p_y=publish_velocity_.tra_velocity_.pre_p_y;
    output.p_x=publish_velocity_.tra_velocity_.p_x;
    output.p_y=publish_velocity_.tra_velocity_.p_y;
    output.v_x=publish_velocity_.tra_velocity_.v_x;
    output.v_y=publish_velocity_.tra_velocity_.v_y;
    return output;
    }

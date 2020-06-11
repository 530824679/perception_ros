#include "tracker/tracker.h"

Tracker::Tracker() : kf_(8, 4), coast_cycles_(0), hit_streak_(0){
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
    kf_.Update(observation);
}

// Create and initialize new trackers for unmatched detections, with initial bounding box
void Tracker::Init(const BBox &bbox) {
    kf_.x_.head(4) << ConvertBboxToObservation(bbox);
    hit_streak_++;
}

BBox Tracker::GetStateAsBbox() const{
    return ConvertStateToBbox(kf_.x_);
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
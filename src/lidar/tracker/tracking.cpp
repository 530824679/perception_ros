#include "tracker/tracking.h"

Tracking::Tracking() {

}

Tracking::~Tracking() {

}

bool Tracking::Init(Json::Value params, std::string key){
    if(params.isMember(key)) {
        Json::Value tracker_param = params[key];

        if (tracker_param.isMember("max_coast_cycles") && tracker_param["max_coast_cycles"].isInt()) {
            max_coast_cycles_ = tracker_param["max_coast_cycles"].asInt();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named max_coast_cycles in the tracker config.\n", __func__);
            return false;
        }

        if (tracker_param.isMember("min_hits") && tracker_param["min_hits"].isInt()) {
            min_hits_ = tracker_param["min_hits"].asInt();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named min_hits in the tracker config.\n", __func__);
            return false;
        }

        if (tracker_param.isMember("min_confidence") && tracker_param["min_confidence"].isDouble()) {
            min_confidence_ = tracker_param["min_confidence"].asFloat();
        } else {
            logger.Log(ERROR, "[%s]: Has not key named size in the tracker config.\n", __func__);
            return false;
        }
    }else{
        logger.Log(ERROR, "[%s]: Has not key named tracker in the perception config.\n", __func__);
    }
}

float Tracking::CalculateIou(const cv::Rect& det, const Tracker& track) {
    auto trk = track.GetStateAsBbox();

    auto xx1 = std::max(det.tl().x, trk.tl().x);
    auto yy1 = std::max(det.tl().y, trk.tl().y);
    auto xx2 = std::min(det.br().x, trk.br().x);
    auto yy2 = std::min(det.br().y, trk.br().y);
    auto w = std::max(0, xx2 - xx1);
    auto h = std::max(0, yy2 - yy1);

    float det_area = det.area();
    float trk_area = trk.area();

    auto intersection_area = w * h;
    float union_area = det_area + trk_area - intersection_area;

    auto iou = intersection_area / union_area;
    return iou;
}

void Tracking::HungarianMatching(const std::vector<std::vector<float>>& iou_matrix, size_t nrows, size_t ncols, std::vector<std::vector<float>>& association) {
    Matrix<float> matrix(nrows, ncols);
    // Initialize matrix with IOU values
    for (size_t i = 0 ; i < nrows ; i++) {
        for (size_t j = 0 ; j < ncols ; j++) {
            // Multiply by -1 to find max cost
            if (iou_matrix[i][j] != 0) {
                matrix(i, j) = -iou_matrix[i][j];
            }
            else {
                matrix(i, j) = 1.0f;
            }
        }
    }

    // Apply Kuhn-Munkres algorithm to matrix.
    Munkres<float> m;
    m.solve(matrix);

    for (size_t i = 0 ; i < nrows ; i++) {
        for (size_t j = 0 ; j < ncols ; j++) {
            association[i][j] = matrix(i, j);
        }
    }

}

void Tracking::AssociateDetectionsToTrackers(const std::vector<cv::Rect>& detection,
                                             std::map<int, Tracker>& tracks,
                                             std::map<int, cv::Rect>& matched,
                                             std::vector<cv::Rect>& unmatched_det,
                                             float iou_threshold) {
    // Set all detection as unmatched if no tracks existing
    if (tracks.empty()) {
        for (const auto& det : detection) {
            unmatched_det.push_back(det);
        }
        return;
    }

    std::vector<std::vector<float>> iou_matrix;
    iou_matrix.resize(detection.size(), std::vector<float>(tracks.size()));

    std::vector<std::vector<float>> association;
    association.resize(detection.size(), std::vector<float>(tracks.size()));

    // row - detection, column - tracks
    for (size_t i = 0; i < detection.size(); i++) {
        size_t j = 0;
        for (const auto& trk : tracks) {
            iou_matrix[i][j] = CalculateIou(detection[i], trk.second);
            j++;
        }
    }

    // Find association
    HungarianMatching(iou_matrix, detection.size(), tracks.size(), association);

    for (size_t i = 0; i < detection.size(); i++) {
        bool matched_flag = false;
        size_t j = 0;
        for (const auto& trk : tracks) {
            if (0 == association[i][j]) {
                // Filter out matched with low IOU
                if (iou_matrix[i][j] >= 0.2) {
                    matched[trk.first] = detection[i];
                    matched_flag = true;
                }
                // It builds 1 to 1 association, so we can break from here
                break;
            }
            j++;
        }
        // if detection cannot match with any tracks
        if (!matched_flag) {
            unmatched_det.push_back(detection[i]);
        }
    }
}

int Tracking::track(std::map<int, Tracker> &tracks, std::vector<cv::Rect> detections, int frame_index, int &current_ID) {
    for (auto &track : tracks){
        track.second.Predict();
    }

    std::map<int, cv::Rect> matched;
    std::vector<cv::Rect> unmatched_det;
    AssociateDetectionsToTrackers(detections, tracks, matched, unmatched_det);

    // Update tracks with associated bbox
    for (const auto &match : matched) {
        const auto &ID = match.first;
        tracks[ID].Update(match.second);
    }

    // Create new tracks for unmatched detections
    for (const auto &det : unmatched_det) {
        Tracker tracker;
        tracker.Init(det);
        // Create new track and generate new ID
        tracks[current_ID++] = tracker;
    }

    // Delete lose tracked tracks
    for (auto it = tracks.begin(); it != tracks.end();) {
        if (it->second.GetCoastCycles() > max_coast_cycles_) {
            it = tracks.erase(it);
        } else {
            it++;
        }
    }


}
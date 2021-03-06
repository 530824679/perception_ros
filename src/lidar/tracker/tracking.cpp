#include "tracker/tracking.h"

Tracking::Tracking() {
    frame_index_ = 1;
    current_id_ = 1;
}

Tracking::~Tracking() {

}

void Tracking::Process(std::vector<BBox> bboxes, perception_ros::ObjectInfoArray &object_array,std::vector<InfoTracker>& trackerinfo){
    
    track(tracks_, bboxes, frame_index_, current_id_, object_array,trackerinfo);
    frame_index_++;
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

float Tracking::CalculateIou(const BBox& det, const Tracker& track) {
    auto trk = track.GetStateAsBbox();

    auto xx1 = std::min((det.x + det.dx/2), (trk.x + trk.dx/2));
    auto yy1 = std::min((det.y + det.dy/2), (trk.y + trk.dy/2));
    auto xx2 = std::max((det.x - det.dx/2), (trk.x - det.dx/2));
    auto yy2 = std::max((det.y - det.dy/2), (trk.y - det.dy/2));
    auto l = std::max(0, int(xx2 - xx1));
    auto w = std::max(0, int(yy2 - yy1));

    float det_area = det.dx * det.dy;
    float trk_area = trk.dx * trk.dy;

    auto intersection_area = l * w;
    float union_area = det_area + trk_area - intersection_area;
    auto iou = intersection_area / union_area;
    return iou;
}


float Tracking::CalculateRotateIOU(const BBox& det, const Tracker& track) {
    auto trk = track.GetStateAsBbox();
    
    cv::Point2f det_center(det.x,det.y);
    cv::Size2f det_size(det.dx,det.dy);
    float tmp_det_angle=det.yaw;
    if (tmp_det_angle < 0)
        {tmp_det_angle+=2. * M_PI;}
    float det_angle=tmp_det_angle/(2*M_PI)*360;
    cv::RotatedRect det_rectangle(det_center,det_size,det_angle);

    cv::Point2f trk_center(trk.x,trk.y);
    cv::Size2f trk_size(trk.dx,trk.dy);
    float tmp_trk_angle=trk.yaw;
    if(tmp_trk_angle<0){
        tmp_trk_angle+=2. * M_PI;
    }
    float trk_angle=tmp_trk_angle/(2*M_PI)*360;
    cv::RotatedRect trk_rectangle(trk_center,trk_size,trk_angle);

    float det_area = det_rectangle.size.width * det_rectangle.size.height;
    float trk_area = trk_rectangle.size.width * trk_rectangle.size.height;
    std::vector<cv::Point2f> vertices;
 
    int intersectionType = cv::rotatedRectangleIntersection(det_rectangle, trk_rectangle, vertices);
    if (vertices.size()==0)
        return 0.0;
    else{
        std::vector<cv::Point2f> order_pts;
        // 找到交集（交集的区域），对轮廓的各个点进行排序
        cv::convexHull(cv::Mat(vertices), order_pts, true);
        double inter_area = cv::contourArea(order_pts);
        float iou = (float) (inter_area / (det_area + trk_area - inter_area + 0.0001));
        return iou;
    }

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

void Tracking::AssociateDetectionsToTrackers(const std::vector<BBox> &bboxes,
                                             std::map<int, Tracker>& tracks,
                                             std::map<int, BBox>& matched,
                                             std::vector<BBox>& unmatched_det,
                                             float iou_threshold) {
    // Set all detection as unmatched if no tracks existing
    if (tracks.empty()) {
        for (const auto& det : bboxes) {
            unmatched_det.push_back(det);
        }
        return;
    }

    std::vector<std::vector<float>> iou_matrix;
    iou_matrix.resize(bboxes.size(), std::vector<float>(tracks.size()));

    std::vector<std::vector<float>> association;
    association.resize(bboxes.size(), std::vector<float>(tracks.size()));

    // row - detection, column - tracks
    for (size_t i = 0; i < bboxes.size(); i++) {
        size_t j = 0;
        for (const auto& trk : tracks) {
            //iou_matrix[i][j] = CalculateIou(bboxes[i], trk.second);
            iou_matrix[i][j] = CalculateRotateIOU(bboxes[i], trk.second);
            j++;
        }
    }

    // Find association
    HungarianMatching(iou_matrix, bboxes.size(), tracks.size(), association);

    for (size_t i = 0; i < bboxes.size(); i++) {
        bool matched_flag = false;
        size_t j = 0;
        for (const auto& trk : tracks) {
            if (0 == association[i][j]) {
                // Filter out matched with low IOU
                if (iou_matrix[i][j] >= 0.2) {//IOU threshold
                    matched[trk.first] = bboxes[i];
                    matched_flag = true;
                }
                // It builds 1 to 1 association, so we can break from here
                break;
            }
            j++;
        }
        // if detection cannot match with any tracks
        if (!matched_flag) {
            unmatched_det.push_back(bboxes[i]);
        }
    }
}

int Tracking::track(std::map<int, Tracker> &tracks, std::vector<BBox> bboxes, int frame_index, int &current_id, 
                    perception_ros::ObjectInfoArray &object_array_msg,std::vector<InfoTracker>& trackerinfo) {
    for (auto &track : tracks){
        track.second.Predict();
    }

    std::map<int, BBox> matched;
    std::vector<BBox> unmatched_det;
    AssociateDetectionsToTrackers(bboxes, tracks, matched, unmatched_det);

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
        tracks[current_id++] = tracker;
    }

    // Delete lose tracked tracks
    for (auto it = tracks.begin(); it != tracks.end();) {
        if (it->second.GetCoastCycles() > max_coast_cycles_) {
            it = tracks.erase(it);
        } else {
            it++;
        }
    }

    perception_ros::ObjectInfo object_info_msg;
    InfoTracker tracker_info_msg;
    int object_num = 0;
    for (auto &trk : tracks) {
        const auto &bbox = trk.second.GetStateAsBbox();
        const auto &velocity=trk.second.GetStateAsVelocity();
        bool state=trk.second.GetCoastCycles() < 5 && (trk.second.GetHitStreak() >= min_hits_||frame_index<min_hits_);
        if (trk.second.GetCoastCycles() < 5 && (trk.second.GetHitStreak() >= min_hits_||frame_index<min_hits_)){
            object_info_msg.id = (uint16_t) trk.first;
            object_info_msg.distance_xv=(uint16_t)bbox.x;
            object_info_msg.distance_yv=(uint16_t)bbox.y;
            object_info_msg.length = (uint16_t) bbox.dx;
            object_info_msg.width = (uint16_t) bbox.dy;
            object_info_msg.yaw=bbox.yaw;
            object_info_msg.velocity_xv=(int16_t)(velocity.v_x*256);
            object_info_msg.velocity_yv=(int16_t)(velocity.v_y*256);
            object_array_msg.object_info[object_num] = object_info_msg;

            tracker_info_msg.id=(uint16_t) trk.first;
            tracker_info_msg.x=bbox.x;
            tracker_info_msg.y=bbox.y;
            tracker_info_msg.z=1;
            tracker_info_msg.length=bbox.dy;
            tracker_info_msg.width=bbox.dx;
            tracker_info_msg.height=2;
            if(bbox.yaw > M_PI)
              {tracker_info_msg.yaw=bbox.yaw - 2. * M_PI;}
            if (bbox.yaw < -M_PI)
              {tracker_info_msg.yaw=bbox.yaw + 2. * M_PI;}
              
            tracker_info_msg.yaw=bbox.yaw;
            object_num++;
        }
        trackerinfo.push_back(tracker_info_msg);
    }
    object_array_msg.object_num = object_num;

}
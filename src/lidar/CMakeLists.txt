########## common_lib ##########
add_library(common_lib STATIC
        ${PROJECT_SOURCE_DIR}/src/lidar/common/logging.cpp
        ${PROJECT_SOURCE_DIR}/src/lidar/common/matrix.cpp
        tracker/munkres.cpp)

########## roi_filter_lib ##########
add_library(roi_filter_lib STATIC
        ${PROJECT_SOURCE_DIR}/src/lidar/roi_filter/roi_filter.cpp)
target_link_libraries(roi_filter_lib
        common_lib
        ${PCL_LIBRARIES})

########## calibration_lib ##########
add_library(calibration_lib STATIC
        ${PROJECT_SOURCE_DIR}/src/lidar/calibration/calibrate.cpp)
target_link_libraries(calibration_lib
        common_lib
        ${PCL_LIBRARIES})

########## curb_detect_lib ##########
add_library(curb_detect_lib STATIC
        ${PROJECT_SOURCE_DIR}/src/lidar/curb_detect/curb_detect.cpp
        ${PROJECT_SOURCE_DIR}/src/lidar/curb_detect/curb_track.cpp)
target_link_libraries(curb_detect_lib
        common_lib
        ${PCL_LIBRARIES})

########## segmentation lib ##########
add_library(segmentation_lib STATIC
        ${PROJECT_SOURCE_DIR}/src/lidar/sementation/object_segment.cpp
        ${PROJECT_SOURCE_DIR}/src/lidar/sementation/object_cluster.cpp)
target_link_libraries(segmentation_lib
        common_lib
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES})

########## object_detect_lib ##########
add_library(object_builder_lib STATIC
        ${PROJECT_SOURCE_DIR}/src/lidar/object_builder/bbox_fitting.cpp)
target_link_libraries(object_builder_lib
        common_lib
        ${PCL_LIBRARIES})



########## recognition_lib ##########




########## tracker_lib ##########
add_library(tracker_lib STATIC
        ${PROJECT_SOURCE_DIR}/src/lidar/tracker/kalman_filter.cpp
        ${PROJECT_SOURCE_DIR}/src/lidar/tracker/tracking.cpp
        ${PROJECT_SOURCE_DIR}/src/lidar/tracker/tracker.cpp
        ${PROJECT_SOURCE_DIR}/src/lidar/tracker/munkres.cpp)
target_link_libraries(tracker_lib
        ${OpenCV_LIBRARIES})

########## render lib ##########
add_library(render_lib STATIC
        ${PROJECT_SOURCE_DIR}/src/lidar/render/render.cpp)
target_link_libraries(render_lib
        ${PCL_LIBRARIES})

########## perception_lib ##########
add_library(lidar_perception_lib STATIC
        ${PROJECT_SOURCE_DIR}/src/lidar/process/lidar_process.cpp)
target_link_libraries(lidar_perception_lib
        ${PCL_LIBRARIES}
        common_lib
        roi_filter_lib
        calibration_lib
        curb_detect_lib
        segmentation_lib
        object_builder_lib
        tracker_lib
        render_lib)
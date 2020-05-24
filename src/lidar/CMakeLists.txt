########## roi_filter_lib ##########
add_library(roi_filter_lib STATIC
        ${PROJECT_SOURCE_DIR}/src/lidar/roi_filter/roi_filter.cpp)
target_link_libraries(roi_filter_lib
        ${PCL_LIBRARIES})

########## calibration_lib ##########
add_library(calibration_lib STATIC
        ${PROJECT_SOURCE_DIR}/src/lidar/calibration/calibrate.cpp)
target_link_libraries(calibration_lib
        ${PCL_LIBRARIES})

########## curb_detect_lib ##########
add_library(curb_detect_lib STATIC
        ${PROJECT_SOURCE_DIR}/src/lidar/curb_detect/grid_map.cpp)
target_link_libraries(curb_detect_lib
        ${PCL_LIBRARIES})

########## object_detect_lib ##########



########## object_filter_lib ##########



########## recognition_lib ##########



########## segmentation_lib ##########



########## tracker_lib ##########



########## perception_lib ##########
add_library(lidar_perception_lib STATIC
        ${PROJECT_SOURCE_DIR}/src/lidar/process/lidar_process.cpp)
target_link_libraries(lidar_perception_lib
        ${PCL_LIBRARIES}
        roi_filter_lib
        calibration_lib
        curb_detect_lib)
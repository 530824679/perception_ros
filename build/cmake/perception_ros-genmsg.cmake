# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "perception_ros: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iperception_ros:/home/linuxidc/perception_ros/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(perception_ros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/linuxidc/perception_ros/msg/ObjectInfo.msg" NAME_WE)
add_custom_target(_perception_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "perception_ros" "/home/linuxidc/perception_ros/msg/ObjectInfo.msg" ""
)

get_filename_component(_filename "/home/linuxidc/perception_ros/msg/ObjectInfoArray.msg" NAME_WE)
add_custom_target(_perception_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "perception_ros" "/home/linuxidc/perception_ros/msg/ObjectInfoArray.msg" "perception_ros/ObjectInfo:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(perception_ros
  "/home/linuxidc/perception_ros/msg/ObjectInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/perception_ros
)
_generate_msg_cpp(perception_ros
  "/home/linuxidc/perception_ros/msg/ObjectInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/home/linuxidc/perception_ros/msg/ObjectInfo.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/perception_ros
)

### Generating Services

### Generating Module File
_generate_module_cpp(perception_ros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/perception_ros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(perception_ros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(perception_ros_generate_messages perception_ros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/linuxidc/perception_ros/msg/ObjectInfo.msg" NAME_WE)
add_dependencies(perception_ros_generate_messages_cpp _perception_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linuxidc/perception_ros/msg/ObjectInfoArray.msg" NAME_WE)
add_dependencies(perception_ros_generate_messages_cpp _perception_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(perception_ros_gencpp)
add_dependencies(perception_ros_gencpp perception_ros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS perception_ros_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(perception_ros
  "/home/linuxidc/perception_ros/msg/ObjectInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/perception_ros
)
_generate_msg_eus(perception_ros
  "/home/linuxidc/perception_ros/msg/ObjectInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/home/linuxidc/perception_ros/msg/ObjectInfo.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/perception_ros
)

### Generating Services

### Generating Module File
_generate_module_eus(perception_ros
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/perception_ros
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(perception_ros_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(perception_ros_generate_messages perception_ros_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/linuxidc/perception_ros/msg/ObjectInfo.msg" NAME_WE)
add_dependencies(perception_ros_generate_messages_eus _perception_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linuxidc/perception_ros/msg/ObjectInfoArray.msg" NAME_WE)
add_dependencies(perception_ros_generate_messages_eus _perception_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(perception_ros_geneus)
add_dependencies(perception_ros_geneus perception_ros_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS perception_ros_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(perception_ros
  "/home/linuxidc/perception_ros/msg/ObjectInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/perception_ros
)
_generate_msg_lisp(perception_ros
  "/home/linuxidc/perception_ros/msg/ObjectInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/home/linuxidc/perception_ros/msg/ObjectInfo.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/perception_ros
)

### Generating Services

### Generating Module File
_generate_module_lisp(perception_ros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/perception_ros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(perception_ros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(perception_ros_generate_messages perception_ros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/linuxidc/perception_ros/msg/ObjectInfo.msg" NAME_WE)
add_dependencies(perception_ros_generate_messages_lisp _perception_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linuxidc/perception_ros/msg/ObjectInfoArray.msg" NAME_WE)
add_dependencies(perception_ros_generate_messages_lisp _perception_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(perception_ros_genlisp)
add_dependencies(perception_ros_genlisp perception_ros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS perception_ros_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(perception_ros
  "/home/linuxidc/perception_ros/msg/ObjectInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/perception_ros
)
_generate_msg_nodejs(perception_ros
  "/home/linuxidc/perception_ros/msg/ObjectInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/home/linuxidc/perception_ros/msg/ObjectInfo.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/perception_ros
)

### Generating Services

### Generating Module File
_generate_module_nodejs(perception_ros
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/perception_ros
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(perception_ros_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(perception_ros_generate_messages perception_ros_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/linuxidc/perception_ros/msg/ObjectInfo.msg" NAME_WE)
add_dependencies(perception_ros_generate_messages_nodejs _perception_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linuxidc/perception_ros/msg/ObjectInfoArray.msg" NAME_WE)
add_dependencies(perception_ros_generate_messages_nodejs _perception_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(perception_ros_gennodejs)
add_dependencies(perception_ros_gennodejs perception_ros_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS perception_ros_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(perception_ros
  "/home/linuxidc/perception_ros/msg/ObjectInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/perception_ros
)
_generate_msg_py(perception_ros
  "/home/linuxidc/perception_ros/msg/ObjectInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/home/linuxidc/perception_ros/msg/ObjectInfo.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/perception_ros
)

### Generating Services

### Generating Module File
_generate_module_py(perception_ros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/perception_ros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(perception_ros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(perception_ros_generate_messages perception_ros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/linuxidc/perception_ros/msg/ObjectInfo.msg" NAME_WE)
add_dependencies(perception_ros_generate_messages_py _perception_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linuxidc/perception_ros/msg/ObjectInfoArray.msg" NAME_WE)
add_dependencies(perception_ros_generate_messages_py _perception_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(perception_ros_genpy)
add_dependencies(perception_ros_genpy perception_ros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS perception_ros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/perception_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/perception_ros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(perception_ros_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/perception_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/perception_ros
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(perception_ros_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/perception_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/perception_ros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(perception_ros_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/perception_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/perception_ros
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(perception_ros_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/perception_ros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/perception_ros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/perception_ros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(perception_ros_generate_messages_py std_msgs_generate_messages_py)
endif()

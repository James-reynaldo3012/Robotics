# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "localization: 3 messages, 1 services")

set(MSG_I_FLAGS "-Ilocalization:/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(localization_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg" NAME_WE)
add_custom_target(_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localization" "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg" "geometry_msgs/Pose:geometry_msgs/Point:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightArray.msg" NAME_WE)
add_custom_target(_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localization" "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightArray.msg" "geometry_msgs/Point:std_msgs/Header:localization/PoseWithWeight:geometry_msgs/Quaternion:geometry_msgs/Pose"
)

get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightStamped.msg" NAME_WE)
add_custom_target(_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localization" "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightStamped.msg" "geometry_msgs/Point:std_msgs/Header:localization/PoseWithWeight:geometry_msgs/Quaternion:geometry_msgs/Pose"
)

get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/srv/SetInitialPose.srv" NAME_WE)
add_custom_target(_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localization" "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/srv/SetInitialPose.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(localization
  "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
)
_generate_msg_cpp(localization
  "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
)
_generate_msg_cpp(localization
  "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
)

### Generating Services
_generate_srv_cpp(localization
  "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/srv/SetInitialPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
)

### Generating Module File
_generate_module_cpp(localization
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(localization_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(localization_generate_messages localization_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg" NAME_WE)
add_dependencies(localization_generate_messages_cpp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightArray.msg" NAME_WE)
add_dependencies(localization_generate_messages_cpp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightStamped.msg" NAME_WE)
add_dependencies(localization_generate_messages_cpp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/srv/SetInitialPose.srv" NAME_WE)
add_dependencies(localization_generate_messages_cpp _localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(localization_gencpp)
add_dependencies(localization_gencpp localization_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localization_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(localization
  "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localization
)
_generate_msg_eus(localization
  "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localization
)
_generate_msg_eus(localization
  "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localization
)

### Generating Services
_generate_srv_eus(localization
  "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/srv/SetInitialPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localization
)

### Generating Module File
_generate_module_eus(localization
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localization
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(localization_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(localization_generate_messages localization_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg" NAME_WE)
add_dependencies(localization_generate_messages_eus _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightArray.msg" NAME_WE)
add_dependencies(localization_generate_messages_eus _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightStamped.msg" NAME_WE)
add_dependencies(localization_generate_messages_eus _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/srv/SetInitialPose.srv" NAME_WE)
add_dependencies(localization_generate_messages_eus _localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(localization_geneus)
add_dependencies(localization_geneus localization_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localization_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(localization
  "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
)
_generate_msg_lisp(localization
  "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
)
_generate_msg_lisp(localization
  "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
)

### Generating Services
_generate_srv_lisp(localization
  "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/srv/SetInitialPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
)

### Generating Module File
_generate_module_lisp(localization
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(localization_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(localization_generate_messages localization_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg" NAME_WE)
add_dependencies(localization_generate_messages_lisp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightArray.msg" NAME_WE)
add_dependencies(localization_generate_messages_lisp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightStamped.msg" NAME_WE)
add_dependencies(localization_generate_messages_lisp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/srv/SetInitialPose.srv" NAME_WE)
add_dependencies(localization_generate_messages_lisp _localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(localization_genlisp)
add_dependencies(localization_genlisp localization_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localization_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(localization
  "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localization
)
_generate_msg_nodejs(localization
  "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localization
)
_generate_msg_nodejs(localization
  "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localization
)

### Generating Services
_generate_srv_nodejs(localization
  "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/srv/SetInitialPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localization
)

### Generating Module File
_generate_module_nodejs(localization
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localization
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(localization_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(localization_generate_messages localization_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg" NAME_WE)
add_dependencies(localization_generate_messages_nodejs _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightArray.msg" NAME_WE)
add_dependencies(localization_generate_messages_nodejs _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightStamped.msg" NAME_WE)
add_dependencies(localization_generate_messages_nodejs _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/srv/SetInitialPose.srv" NAME_WE)
add_dependencies(localization_generate_messages_nodejs _localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(localization_gennodejs)
add_dependencies(localization_gennodejs localization_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localization_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(localization
  "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
)
_generate_msg_py(localization
  "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
)
_generate_msg_py(localization
  "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
)

### Generating Services
_generate_srv_py(localization
  "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/srv/SetInitialPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
)

### Generating Module File
_generate_module_py(localization
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(localization_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(localization_generate_messages localization_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeight.msg" NAME_WE)
add_dependencies(localization_generate_messages_py _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightArray.msg" NAME_WE)
add_dependencies(localization_generate_messages_py _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/msg/PoseWithWeightStamped.msg" NAME_WE)
add_dependencies(localization_generate_messages_py _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/sf_robotics_c/workspace_assignment4/workspace_assignment4/src/localization/srv/SetInitialPose.srv" NAME_WE)
add_dependencies(localization_generate_messages_py _localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(localization_genpy)
add_dependencies(localization_genpy localization_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localization_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(localization_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(localization_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(localization_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localization
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(localization_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(localization_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(localization_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(localization_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(localization_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(localization_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localization
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(localization_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(localization_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(localization_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(localization_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(localization_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(localization_generate_messages_py sensor_msgs_generate_messages_py)
endif()

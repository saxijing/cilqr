# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "saturn_msgs: 7 messages, 0 services")

set(MSG_I_FLAGS "-Isaturn_msgs:/home/saxijing/cilqr/src/message/saturn_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(saturn_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ControlArray.msg" NAME_WE)
add_custom_target(_saturn_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "saturn_msgs" "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ControlArray.msg" "saturn_msgs/Control:std_msgs/Header"
)

get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Control.msg" NAME_WE)
add_custom_target(_saturn_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "saturn_msgs" "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Control.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg" NAME_WE)
add_custom_target(_saturn_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "saturn_msgs" "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleStateArray.msg" NAME_WE)
add_custom_target(_saturn_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "saturn_msgs" "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleStateArray.msg" "saturn_msgs/Size:saturn_msgs/ObstacleState:saturn_msgs/StateLite:std_msgs/Header"
)

get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/State.msg" NAME_WE)
add_custom_target(_saturn_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "saturn_msgs" "/home/saxijing/cilqr/src/message/saturn_msgs/msg/State.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg" NAME_WE)
add_custom_target(_saturn_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "saturn_msgs" "/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleState.msg" NAME_WE)
add_custom_target(_saturn_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "saturn_msgs" "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleState.msg" "saturn_msgs/Size:saturn_msgs/StateLite:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ControlArray.msg"
  "${MSG_I_FLAGS}"
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Control.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/saturn_msgs
)
_generate_msg_cpp(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/saturn_msgs
)
_generate_msg_cpp(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/saturn_msgs
)
_generate_msg_cpp(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleStateArray.msg"
  "${MSG_I_FLAGS}"
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg;/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleState.msg;/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/saturn_msgs
)
_generate_msg_cpp(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/State.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/saturn_msgs
)
_generate_msg_cpp(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/saturn_msgs
)
_generate_msg_cpp(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleState.msg"
  "${MSG_I_FLAGS}"
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg;/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/saturn_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(saturn_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/saturn_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(saturn_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(saturn_msgs_generate_messages saturn_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ControlArray.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_cpp _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Control.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_cpp _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_cpp _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleStateArray.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_cpp _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/State.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_cpp _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_cpp _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleState.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_cpp _saturn_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(saturn_msgs_gencpp)
add_dependencies(saturn_msgs_gencpp saturn_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS saturn_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ControlArray.msg"
  "${MSG_I_FLAGS}"
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Control.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/saturn_msgs
)
_generate_msg_eus(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/saturn_msgs
)
_generate_msg_eus(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/saturn_msgs
)
_generate_msg_eus(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleStateArray.msg"
  "${MSG_I_FLAGS}"
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg;/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleState.msg;/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/saturn_msgs
)
_generate_msg_eus(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/State.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/saturn_msgs
)
_generate_msg_eus(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/saturn_msgs
)
_generate_msg_eus(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleState.msg"
  "${MSG_I_FLAGS}"
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg;/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/saturn_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(saturn_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/saturn_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(saturn_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(saturn_msgs_generate_messages saturn_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ControlArray.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_eus _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Control.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_eus _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_eus _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleStateArray.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_eus _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/State.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_eus _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_eus _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleState.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_eus _saturn_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(saturn_msgs_geneus)
add_dependencies(saturn_msgs_geneus saturn_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS saturn_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ControlArray.msg"
  "${MSG_I_FLAGS}"
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Control.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/saturn_msgs
)
_generate_msg_lisp(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/saturn_msgs
)
_generate_msg_lisp(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/saturn_msgs
)
_generate_msg_lisp(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleStateArray.msg"
  "${MSG_I_FLAGS}"
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg;/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleState.msg;/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/saturn_msgs
)
_generate_msg_lisp(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/State.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/saturn_msgs
)
_generate_msg_lisp(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/saturn_msgs
)
_generate_msg_lisp(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleState.msg"
  "${MSG_I_FLAGS}"
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg;/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/saturn_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(saturn_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/saturn_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(saturn_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(saturn_msgs_generate_messages saturn_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ControlArray.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_lisp _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Control.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_lisp _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_lisp _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleStateArray.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_lisp _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/State.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_lisp _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_lisp _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleState.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_lisp _saturn_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(saturn_msgs_genlisp)
add_dependencies(saturn_msgs_genlisp saturn_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS saturn_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ControlArray.msg"
  "${MSG_I_FLAGS}"
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Control.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/saturn_msgs
)
_generate_msg_nodejs(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/saturn_msgs
)
_generate_msg_nodejs(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/saturn_msgs
)
_generate_msg_nodejs(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleStateArray.msg"
  "${MSG_I_FLAGS}"
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg;/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleState.msg;/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/saturn_msgs
)
_generate_msg_nodejs(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/State.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/saturn_msgs
)
_generate_msg_nodejs(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/saturn_msgs
)
_generate_msg_nodejs(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleState.msg"
  "${MSG_I_FLAGS}"
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg;/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/saturn_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(saturn_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/saturn_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(saturn_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(saturn_msgs_generate_messages saturn_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ControlArray.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_nodejs _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Control.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_nodejs _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_nodejs _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleStateArray.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_nodejs _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/State.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_nodejs _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_nodejs _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleState.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_nodejs _saturn_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(saturn_msgs_gennodejs)
add_dependencies(saturn_msgs_gennodejs saturn_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS saturn_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ControlArray.msg"
  "${MSG_I_FLAGS}"
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Control.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/saturn_msgs
)
_generate_msg_py(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/saturn_msgs
)
_generate_msg_py(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/saturn_msgs
)
_generate_msg_py(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleStateArray.msg"
  "${MSG_I_FLAGS}"
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg;/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleState.msg;/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/saturn_msgs
)
_generate_msg_py(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/State.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/saturn_msgs
)
_generate_msg_py(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/saturn_msgs
)
_generate_msg_py(saturn_msgs
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleState.msg"
  "${MSG_I_FLAGS}"
  "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg;/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/saturn_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(saturn_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/saturn_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(saturn_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(saturn_msgs_generate_messages saturn_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ControlArray.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_py _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Control.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_py _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/Size.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_py _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleStateArray.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_py _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/State.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_py _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/StateLite.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_py _saturn_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/saxijing/cilqr/src/message/saturn_msgs/msg/ObstacleState.msg" NAME_WE)
add_dependencies(saturn_msgs_generate_messages_py _saturn_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(saturn_msgs_genpy)
add_dependencies(saturn_msgs_genpy saturn_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS saturn_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/saturn_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/saturn_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(saturn_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/saturn_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/saturn_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(saturn_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/saturn_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/saturn_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(saturn_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/saturn_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/saturn_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(saturn_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/saturn_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/saturn_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/saturn_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(saturn_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()

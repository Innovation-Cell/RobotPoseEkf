# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "localization: 3 messages, 0 services")

set(MSG_I_FLAGS "-Ilocalization:/home/rishabh/localization/src/localization/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg;-Igeographic_msgs:/opt/ros/hydro/share/geographic_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg;-Inav_msgs:/opt/ros/hydro/share/nav_msgs/cmake/../msg;-Itf:/opt/ros/hydro/share/tf/cmake/../msg;-Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg;-Iuuid_msgs:/opt/ros/hydro/share/uuid_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/hydro/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(localization_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(localization
  "/home/rishabh/localization/src/localization/msg/lp.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
)
_generate_msg_cpp(localization
  "/home/rishabh/localization/src/localization/msg/lla.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/hydro/share/geographic_msgs/cmake/../msg/GeoPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
)
_generate_msg_cpp(localization
  "/home/rishabh/localization/src/localization/msg/roboteq_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
)

### Generating Services

### Generating Module File
_generate_module_cpp(localization
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(localization_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(localization_generate_messages localization_generate_messages_cpp)

# target for backward compatibility
add_custom_target(localization_gencpp)
add_dependencies(localization_gencpp localization_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localization_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(localization
  "/home/rishabh/localization/src/localization/msg/lp.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
)
_generate_msg_lisp(localization
  "/home/rishabh/localization/src/localization/msg/lla.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/hydro/share/geographic_msgs/cmake/../msg/GeoPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
)
_generate_msg_lisp(localization
  "/home/rishabh/localization/src/localization/msg/roboteq_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
)

### Generating Services

### Generating Module File
_generate_module_lisp(localization
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(localization_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(localization_generate_messages localization_generate_messages_lisp)

# target for backward compatibility
add_custom_target(localization_genlisp)
add_dependencies(localization_genlisp localization_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localization_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(localization
  "/home/rishabh/localization/src/localization/msg/lp.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
)
_generate_msg_py(localization
  "/home/rishabh/localization/src/localization/msg/lla.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/hydro/share/geographic_msgs/cmake/../msg/GeoPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
)
_generate_msg_py(localization
  "/home/rishabh/localization/src/localization/msg/roboteq_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
)

### Generating Services

### Generating Module File
_generate_module_py(localization
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(localization_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(localization_generate_messages localization_generate_messages_py)

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
add_dependencies(localization_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(localization_generate_messages_cpp geographic_msgs_generate_messages_cpp)
add_dependencies(localization_generate_messages_cpp sensor_msgs_generate_messages_cpp)
add_dependencies(localization_generate_messages_cpp nav_msgs_generate_messages_cpp)
add_dependencies(localization_generate_messages_cpp tf_generate_messages_cpp)
add_dependencies(localization_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(localization_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(localization_generate_messages_lisp geographic_msgs_generate_messages_lisp)
add_dependencies(localization_generate_messages_lisp sensor_msgs_generate_messages_lisp)
add_dependencies(localization_generate_messages_lisp nav_msgs_generate_messages_lisp)
add_dependencies(localization_generate_messages_lisp tf_generate_messages_lisp)
add_dependencies(localization_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(localization_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(localization_generate_messages_py geographic_msgs_generate_messages_py)
add_dependencies(localization_generate_messages_py sensor_msgs_generate_messages_py)
add_dependencies(localization_generate_messages_py nav_msgs_generate_messages_py)
add_dependencies(localization_generate_messages_py tf_generate_messages_py)
add_dependencies(localization_generate_messages_py geometry_msgs_generate_messages_py)

# generated from genmsg/cmake/pkg-msg-paths.cmake.em

# message include dirs in installspace
_prepend_path("${localization_DIR}/.." "msg" localization_MSG_INCLUDE_DIRS UNIQUE)
set(localization_MSG_DEPENDENCIES std_msgs;geographic_msgs;sensor_msgs;nav_msgs;tf;geometry_msgs)

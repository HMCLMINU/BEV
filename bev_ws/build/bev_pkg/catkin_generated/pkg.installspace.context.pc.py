# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;rospy;std_msgs;message_runtime;cv_bridge;image_transport;sensor_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lbev_pkg;-lopencv".split(';') if "-lbev_pkg;-lopencv" != "" else []
PROJECT_NAME = "bev_pkg"
PROJECT_SPACE_DIR = "/home/hmcl/carla-birdeye-view/bev_ws/install"
PROJECT_VERSION = "1.0.0"

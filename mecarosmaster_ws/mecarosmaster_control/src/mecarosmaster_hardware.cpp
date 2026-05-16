// /home/rosmaster/mecarosmaster/mecarosmaster_ws/mecarosmaster_control/src/mecarosmaster_hardware.cpp

#include "mecarosmaster_control/mecarosmaster_hardware.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mecarosmaster_ros2_control::MecarosmasterHardware,
  hardware_interface::SystemInterface)

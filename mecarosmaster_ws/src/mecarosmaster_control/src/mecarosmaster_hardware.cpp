// mecarosmaster_hardware.cpp
// Enregistre MecarosmasterHardware comme plugin pluginlib.
// Toute l'implémentation est dans mecarosmaster_hardware.hpp (inline).

#include "mecarosmaster_control/mecarosmaster_hardware.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    mecarosmaster_ros2_control::MecarosmasterHardware,
    hardware_interface::SystemInterface)

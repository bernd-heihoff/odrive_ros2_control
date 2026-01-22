#include "odrive_hardware_interface/odrive_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  odrive_hardware_interface::ODriveHardwareInterface,
  hardware_interface::SystemInterface)

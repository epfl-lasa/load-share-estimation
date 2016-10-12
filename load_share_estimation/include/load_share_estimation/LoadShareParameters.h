// Author: Felix Duvallet on 8/30/16.

#ifndef _LOADSHAREPARAMS_H_
#define _LOADSHAREPARAMS_H_

namespace load_share_estimation {

class LoadShareParameters {
 public:
  LoadShareParameters() { }

  // Boilderplate code that gets parameter from server. If parameter doesn't
  // exist, prints a warning message and returns false.
  bool getParamDouble(ros::NodeHandle *nodeHandle,
                      const std::string &param_name,
                      double &param) {
    if (!nodeHandle->getParam(param_name, param)) {
      ROS_WARN_STREAM("Could not get parameter from server: " << param_name);
      return false;
    }
    return true;
  }
  bool getParamString(ros::NodeHandle *nodeHandle,
                      const std::string &param_name,
                      std::string &param) {
    if (!nodeHandle->getParam(param_name, param)) {
      ROS_WARN_STREAM("Could not get parameter from server: " << param_name);
      return false;
    }
    return true;
  }

  bool fromParamServer(ros::NodeHandle *nodeHandle) {
    if (!getParamDouble(nodeHandle, "masses/object", object_mass))
      return false;

    if (!getParamDouble(nodeHandle, "masses/tool", tool_mass))
      return false;

    if (!getParamDouble(nodeHandle, "masses/ft_plate", ft_plate_mass))
      return false;

    if (!getParamDouble(nodeHandle, "filtering_params/smoothing_load_share",
                       smoothing_load_share))
      return false;
    if (!getParamDouble(nodeHandle, "filtering_params/smoothing_force",
                       smoothing_force))
      return false;
    if (!getParamDouble(nodeHandle, "filtering_params/smoothing_torque",
                       smoothing_torque))
      return false;
    if (!getParamDouble(nodeHandle, "ft_sensor_delay", ft_delay))
      return false;

    if (!getParamString(nodeHandle, "calibration_orientation_param_name",
                       param_name_calibration_orientation))
      return false;

    if (!getParamString(nodeHandle, "topics/load_share",
                       topic_out_load_share))
      return false;
    if (!getParamString(nodeHandle, "topics/ft_sensor",
                       topic_in_ft_sensor))
      return false;
    if (!getParamString(nodeHandle, "topics/robot_ee_accel",
                       topic_in_robot_ee_accel))
      return false;


    return true;
  }

  // Print all parameters to the ros info stream.
  void log() {
    ROS_INFO_STREAM("Masses:");
    ROS_INFO_STREAM("   object: " << object_mass);
    ROS_INFO_STREAM("   tool: " << tool_mass);
    ROS_INFO_STREAM("   F/T plate: " << ft_plate_mass);

    ROS_INFO_STREAM("Topics:");
    ROS_INFO_STREAM("   Load share (out): " << topic_out_load_share);
    ROS_INFO_STREAM("   Force/torque sensor (in): " << topic_in_ft_sensor);
    ROS_INFO_STREAM("   Robot acceleration (in): " << topic_in_robot_ee_accel);
    ROS_INFO_STREAM("Parameter for calibration orientation: "
                    << param_name_calibration_orientation);

    ROS_INFO_STREAM("Filtering parameters");
    ROS_INFO_STREAM("   Smoothing, load share: " << smoothing_load_share);
    ROS_INFO_STREAM("   Smoothing, force: " << smoothing_force);
    ROS_INFO_STREAM("   Smoothing, torque: " << smoothing_torque);

    ROS_INFO_STREAM("Force/Torque sensor delay: " << ft_delay);
  }

  // Object masses.
  double object_mass;
  double tool_mass;
  double ft_plate_mass;

  // Name of parameters.
  std::string param_name_calibration_orientation;

  // Name of topics.
  std::string topic_out_load_share;
  std::string topic_in_ft_sensor;
  std::string topic_in_robot_ee_accel;

  double ft_delay;

  // Filtering parameters.
  double smoothing_load_share;
  double smoothing_force;
  double smoothing_torque;
};

}   // namespace load_share_estimation

#endif  // _LOADSHAREPARAMS_H_

#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <load_share_estimation/LoadShareEstimator.h>
#include <control_toolbox/filters.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>


LoadShareEstimator::LoadShareEstimator(ros::NodeHandle *nodeHandle)
    : nodeHandle_(nodeHandle),
      tf_listener_ft_sensor_(),
      tf_listener_robot_(){

  ft_calib_force_bias_.setZero();
  ft_calib_torque_bias_.setZero();
  ft_calib_orientation_.setIdentity();
}

bool LoadShareEstimator::init() {
  ROS_INFO_STREAM("Load Share Estimator initialization starting...");

  const std::string topic_load_share = "load_share";
  const std::string topic_ft_sensor = "ft_sensor";
  const std::string topic_ee_state = "ee_state";
  const double ft_delay = 0.01;

  // TODO Get these from yaml parameters.
  mass_object_ = 1.0;
  mass_tool_ = 1.0;
  mass_ft_plate_ = 1.0;

  if(!load_calibration()) {
    ROS_ERROR("Could not load f/t sensor calibration information.");
    ROS_ERROR("Did you run the calibration script?");  // TODO provide name.
    return false;
  }

  while(!waitForTransforms()) {
    ROS_INFO_STREAM("Still waiting for transforms...");
    ros::Duration(1.0).sleep();
  }

  if(!subscribe_ft_sensor(topic_ft_sensor, ft_delay))
    return false;

  if(!subscribe_robot_state())
    return false;

  ROS_INFO_STREAM("Load Share Estimator initialization SUCCESS!");
  return true;
}

bool LoadShareEstimator::load_calibration() {

  std::vector<double> lwr_ft_calib_orientation_list;
  if (!nodeHandle_->getParam("/lwr_ft_calib_orientation",
                             lwr_ft_calib_orientation_list)) {
    return false;
  }
  ft_calib_orientation_.x() = lwr_ft_calib_orientation_list.at(0);
  ft_calib_orientation_.y() = lwr_ft_calib_orientation_list.at(1);
  ft_calib_orientation_.z() = lwr_ft_calib_orientation_list.at(2);
  ft_calib_orientation_.w() = lwr_ft_calib_orientation_list.at(3);

  return true;
}

bool LoadShareEstimator::subscribe_ft_sensor(
  const std::string ft_topic, double ft_delay_time) {


  ft_sub.reset(new message_filters::Subscriber<geometry_msgs::WrenchStamped>(
    *nodeHandle_, ft_topic, 1, ros::TransportHints().reliable().tcpNoDelay()));
  ft_filter.reset(new message_filters::TimeSequencer<geometry_msgs::WrenchStamped>(
    *ft_sub, ros::Duration(ft_delay_time), ros::Duration(0.0001), 10000));
  ft_filter->registerCallback(
    boost::bind(&LoadShareEstimator::callback_ft_sensor_filtered, this, _1));
  return true;
}

void LoadShareEstimator::callback_ft_sensor_filtered(
  const geometry_msgs::WrenchStamped::ConstPtr &msg) {

  ft_data_ = *msg;
}

void LoadShareEstimator::computeSmoothedFTWorld() {

  // NOTE: The f/t data is filtered; in our case this is a constant delay (using
  // the TimeSequencer) to better align the f/t data with the acceleration
 // estimate, which is also slightly delayed.
  Eigen::Vector3d force;
  Eigen::Vector3d torque;
  force << ft_data_.wrench.force.x, ft_data_.wrench.force.y,
    ft_data_.wrench.force.z;
  torque << ft_data_.wrench.torque.x, ft_data_.wrench.torque.y,
    ft_data_.wrench.torque.z;

  // Transform forces and torques so they are expressed in world frame.
  Eigen::Matrix3d rot;
  tf::matrixTFToEigen(tf_ft_sensor_.getBasis(), rot);
  Eigen::Vector3d force_world = rot*(force + ft_calib_force_bias_);
  Eigen::Vector3d torque_world = rot*(torque + ft_calib_torque_bias_);

  for (size_t i=0 ; i < 3 ; i ++ ) {
    force_cur_(i) = filters::exponentialSmoothing(force_world(i), force_prev_(i), 0.25);
    torque_cur_(i) = filters::exponentialSmoothing(torque_world(i), torque_prev_(i), 0.25);
  }

  force_prev_ = force_cur_;
  torque_prev_ = torque_cur_;
}

void LoadShareEstimator::setObjectMass(double object_mass) {

  Eigen::Matrix<double,3,3> mass_object_hand;
  Eigen::Vector3d m_object_g, m_hand_ft_sensor_g, m_joint_g, f_dyn, f_obs, f_obs_dyn, f_int;


  mass_object_ = object_mass;

  m_object_g << 0, 0, -9.81 * mass_object_;

  const double joint_mass = mass_tool_ + mass_ft_plate_+ mass_object_;
  mass_object_hand.setZero();
  mass_object_hand.diagonal() << joint_mass, joint_mass, joint_mass;

  m_hand_ft_sensor_g << 0, 0, -9.81 * (mass_ft_plate_ + mass_tool_);
  m_joint_g << 0, 0, -9.81 * (joint_mass);

  // Compute the force correction term by expressing the expected force in the
  // calibration frame (when we measured the sensor bias).
  Eigen::Matrix3d rotation_world_to_orient_at_calibration;
  rotation_world_to_orient_at_calibration =
      ft_calib_orientation_.toRotationMatrix();
  Eigen::Vector3d bias_correction =
      rotation_world_to_orient_at_calibration * m_hand_ft_sensor_g;
  ft_calib_torque_bias_ = bias_correction;
}

bool LoadShareEstimator::subscribe_robot_state() {

  ROS_WARN("Implement this.");

  return true;
}

bool LoadShareEstimator::work() {
  return true;
}

bool LoadShareEstimator::waitForTransforms() {
  bool tf_ready_sensor = false;
  try {
    tf_ready_sensor = tf_listener_ft_sensor_.waitForTransform(
      "/world", "/ft_sensor", ros::Time(0), ros::Duration(3.0));
    if (!tf_ready_sensor) {
      ROS_INFO_STREAM("Still waiting for ft_sensor transform... ");
    }
  } catch (tf::TransformException ex) {
    ROS_ERROR("TF exception: %s", ex.what());
    ros::Duration(1.0).sleep();
  }

  bool tf_ready_robot = false;
  try {
    tf_ready_robot = tf_listener_robot_.waitForTransform(
      "/world", "/palm_link", ros::Time(0), ros::Duration(3.0));
    if (!tf_ready_robot) {
      ROS_INFO_STREAM("Still waiting for /palm_link transform... ");
    }
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("TF exception: %s", ex.what());
    ros::Duration(1.0).sleep();
  }

  return tf_ready_sensor && tf_ready_robot;
}

void LoadShareEstimator::updateTransforms() {
  try{
    tf_listener_ft_sensor_.lookupTransform(
      "/world", "/ft_sensor", ros::Time(0), tf_ft_sensor_);
  } catch (tf::TransformException ex) {
    ROS_ERROR("TF exception: %s", ex.what());
    ros::Duration(1.0).sleep();
  }

  try {
    tf_listener_robot_.lookupTransform(
      "/world", "/robot_root", ros::Time(0), tf_robot_root_);
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("TF exception: %s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

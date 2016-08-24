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

  ft_calibration_.orientation.setIdentity();
  ft_calibration_.force_bias.setZero();
  ft_calibration_.torque_bias.setZero();
}

bool LoadShareEstimator::init() {
  ROS_INFO_STREAM("Load Share Estimator initialization starting...");

  const std::string topic_load_share = "load_share";
  const std::string topic_ft_sensor = "ft_sensor";
  const std::string topic_ee_state = "ee_state";
  const double ft_delay = 0.01;

  // TODO Get these from yaml parameters.

  masses_.object = 1.0;
  masses_.tool = 1.0;
  masses_.ft_plate = 1.0;


  if(!loadCalibration()) {
    ROS_ERROR("Could not load f/t sensor calibration information.");
    ROS_ERROR("Did you run the calibration script?");  // TODO provide name.
    return false;
  }

  while(!waitForTransforms()) {
    ROS_INFO_STREAM("Still waiting for transforms...");
    ros::Duration(1.0).sleep();
  }

  if(!subscribeFTSensor(topic_ft_sensor, ft_delay))
    return false;

  if(!subscribeRobotState())
    return false;

  ROS_INFO_STREAM("Load Share Estimator initialization SUCCESS!");
  return true;
}

bool LoadShareEstimator::loadCalibration() {

  std::vector<double> lwr_ft_calib_orientation_list;
  if (!nodeHandle_->getParam("/lwr_ft_calib_orientation",
                             lwr_ft_calib_orientation_list)) {
    return false;
  }

  ft_calibration_.orientation.x() = lwr_ft_calib_orientation_list.at(0);
  ft_calibration_.orientation.y() = lwr_ft_calib_orientation_list.at(1);
  ft_calibration_.orientation.z() = lwr_ft_calib_orientation_list.at(2);
  ft_calibration_.orientation.w() = lwr_ft_calib_orientation_list.at(3);

  return true;
}

bool LoadShareEstimator::subscribeFTSensor(
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

void LoadShareEstimator::computeSmoothedFTInWorldFrame() {

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
  Eigen::Vector3d force_world = rot*(force + ft_calibration_.force_bias);
  Eigen::Vector3d torque_world = rot*(torque + ft_calibration_.torque_bias);

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


  masses_.object = object_mass;

  m_object_g << 0, 0, -9.81 * masses_.object;

  const double joint_mass = masses_.tool + masses_.ft_plate + masses_.object;
  mass_object_hand.setZero();
  mass_object_hand.diagonal() << joint_mass, joint_mass, joint_mass;

  m_hand_ft_sensor_g << 0, 0, -9.81 * (masses_.tool + masses_.ft_plate);
  m_joint_g << 0, 0, -9.81 * (joint_mass);

  // Compute the force correction term by expressing the expected force in the
  // calibration frame (when we measured the sensor bias).
  Eigen::Matrix3d rotation_world_to_orient_at_calibration;
  rotation_world_to_orient_at_calibration =
      ft_calibration_.orientation.toRotationMatrix();
  Eigen::Vector3d bias_correction =
      rotation_world_to_orient_at_calibration * m_hand_ft_sensor_g;
  ft_calibration_.force_bias = bias_correction;
}

bool LoadShareEstimator::subscribeRobotState() {

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

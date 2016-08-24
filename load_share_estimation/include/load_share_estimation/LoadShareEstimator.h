#ifndef __LOAD_SHARE_ESTIMATOR__
#define __LOAD_SHARE_ESTIMATOR__

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/WrenchStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_sequencer.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

class LoadShareEstimator {
 public:
  LoadShareEstimator(ros::NodeHandle *nodeHandle);

  bool init();

  bool loadCalibration();
  bool subscribeFTSensor(const std::string ft_topic, double ft_delay_time);
  bool subscribeRobotState();

  bool work();

  void computeSmoothedFTInWorldFrame();

 protected:
  ros::NodeHandle *nodeHandle_;

  // Subscriber & filter for the force/torque data.
  std::unique_ptr<
    message_filters::Subscriber<geometry_msgs::WrenchStamped> > ft_sub;
  std::unique_ptr<
    message_filters::TimeSequencer<geometry_msgs::WrenchStamped> > ft_filter;

  void callback_ft_sensor_filtered(
    const geometry_msgs::WrenchStamped::ConstPtr &msg);

  geometry_msgs::WrenchStamped ft_data_;

  Eigen::Vector3d force_cur_;
  Eigen::Vector3d force_prev_;
  Eigen::Vector3d torque_cur_;
  Eigen::Vector3d torque_prev_;
  const double ft_smoothing_constant_ = 0.25;

  // FT sensor calibration variables: the baseline forces and torques at
  // calibration time and the sensor orientation at the calibration time.
  struct ft_calibration_t {
    Eigen::Quaterniond orientation;
    Eigen::Vector3d force_bias;
    Eigen::Vector3d torque_bias;
  };
  ft_calibration_t ft_calibration_;

  // All the masses we care about in one nice struct: the object (the thing
  // we're estimating load share for), the tool (hand/griper/etc...), and the
  // force/torque sensor mounting plate (i.e. the part that's not bolted on the
  // arm).
  struct masses_t {
    double object;
    double tool;
    double ft_plate;
  };
  masses_t masses_;

  // The force/torque sensor and the robot root expressed in the world frame.
  tf::TransformListener tf_listener_ft_sensor_;
  tf::StampedTransform tf_ft_sensor_;
  tf::TransformListener tf_listener_robot_;
  tf::StampedTransform tf_robot_root_;

  bool waitForTransforms();
  void updateTransforms();


  void setObjectMass(double object_mass);

};

#endif  // __LOAD_SHARE_ESTIMATOR__

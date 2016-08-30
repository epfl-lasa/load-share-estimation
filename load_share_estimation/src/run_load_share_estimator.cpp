#include <ros/ros.h>
#include <load_share_estimation/LoadShareEstimator.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "load_share_estimator");

  // We initialize the node handle with a namespace, to keep all topics
  // together.
  ros::NodeHandle nodeHandle("load_share_estimator");

  LoadShareEstimator worker(&nodeHandle);
  if (!worker.init()) {
    ROS_ERROR("Error initializing node.");
    return -1;
  }

  // Allow the subscribers and publishers to connect.
  ros::Duration(0.5).sleep();

  ROS_INFO_STREAM("Node started");
  ros::Rate rate = ros::Rate(300);
  while (ros::ok()) {
    worker.work();

    // Don't forget to call work to process messages, then sleep.
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO_STREAM("Node finished");
}

#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <load_share_estimation/LoadShareEstimator.h>

BasicWorker::BasicWorker(ros::NodeHandle *nodeHandle)
    : nodeHandle_(nodeHandle) {
  counter_ = 0;
}

bool BasicWorker::init() {
  publisher_ = nodeHandle_->advertise<std_msgs::Int32>("foo_cpp", 10);
  subscriber_ =
      nodeHandle_->subscribe("foo_cpp", 10, &BasicWorker::MsgCB, this);
  ROS_INFO_STREAM("Started subscriber " << subscriber_);
  return true;
}

void BasicWorker::MsgCB(const std_msgs::Int32ConstPtr &msg) {
  ROS_INFO("Got callback");
  ROS_INFO_STREAM("Heard: " << msg->data);
}

bool BasicWorker::work() {
  std_msgs::Int32 msg;
  msg.data = ++counter_;

  publisher_.publish(msg);

  return true;
}

#include "state_estimation_node_cpp/state_estimation_node_impl.hpp"
#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "StateEstimation");
  ros::NodeHandle nh = ros::NodeHandle("/core/state");
  auto node = std::make_shared<stateEstimationNode<tam::core::state::EKF_3D>>(
    std::make_unique<tam::core::state::StateEstimation<tam::core::state::EKF_3D>>("single-track-model", nh),
    "single-track-model", nh);
    
  ros::spin();
  return 0;
}

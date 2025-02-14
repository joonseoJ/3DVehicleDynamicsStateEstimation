#include <ros/ros.h>
#include "state_estimation_node_cpp/state_estimation_node_impl.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "StateEstimation");
  ros::NodeHandle nh = ros::NodeHandle("/core/state");
  auto node = std::make_shared<stateEstimationNode<tam::core::state::EKF_3D>>(
    std::make_unique<tam::core::state::StateEstimation<tam::core::state::EKF_3D>>("non-holonomic", nh),
    "non-holonomic", nh);
    
  ros::spin();
  return 0;
}

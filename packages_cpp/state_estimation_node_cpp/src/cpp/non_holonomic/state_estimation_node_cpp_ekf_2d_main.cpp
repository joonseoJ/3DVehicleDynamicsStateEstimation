#include <ros/ros.h>
#include "state_estimation_node_cpp/state_estimation_node_impl.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "StateEstimation");
  ros::NodeHandle nh = ros::NodeHandle("/core/state/StateEstimation");
  auto node = std::make_shared<stateEstimationNode<tam::core::state::EKF_2D>>(
    std::make_unique<tam::core::state::StateEstimation<tam::core::state::EKF_2D>>("non-holonomic", nh),
    "non-holonomic", nh);
    
    ros::spin();
  return 0;
}

#include <ros/ros.h>
#include "ssa_estimation_node_cpp/ssa_estimation_node_impl.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "SSAEstimation");
  ros::NodeHandle nh = ros::NodeHandle("/core/ssa/SSAEstimation");
  auto node = std::make_shared<SSAEstimationNode<tam::core::ssa::UKF_STM>>(
    std::make_unique<tam::core::ssa::SSAEstimation<tam::core::ssa::UKF_STM>>(nh), nh);
    
    ros::spin();
  return 0;
}

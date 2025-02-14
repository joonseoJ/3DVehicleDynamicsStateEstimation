#include "ros1_watchdog_cpp/topic_watchdog.hpp"
namespace tam::core
{
/**
 * Create a Watchdog that periodically checks timeout of subscriptions
 *
 * @param nh ROS 1 NodeHandle that this watchdog is running in
 */
TopicWatchdog::TopicWatchdog(ros::NodeHandle nh) { this->nh = nh; }

std::function<void()> TopicWatchdog::get_update_function()
{
  return std::bind(&TopicWatchdog::check_timeouts, this);
}
void TopicWatchdog::check_timeouts()
{
  auto now = ros::Time::now();
  for (auto & watchdog_sub : this->watched_callbacks) {
    ros::Duration timeout_now = now - watchdog_sub->last_update;
    std::chrono::milliseconds timeout_ms(static_cast<int>(timeout_now.toSec() * 1000));
    watchdog_sub->timeout_callback(timeout_ms > watchdog_sub->timeout, timeout_ms);
  }
}
}  // namespace tam::core

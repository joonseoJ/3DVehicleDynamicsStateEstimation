// Copyright 2023 Simon Hoffmann
#pragma once
#include <vector>
#include <ros/ros.h>
#include <functional>

namespace tam::core
{
/**
 * @brief Container that stores multiple std::functions and executes them in the inserted order when
 * calling call() method. Currently only <void()> supported!
 */
template <typename T>
class function_queue
{
public:
  /**
   * @brief Add a new function to the container
   * 
   * @param func Function of type T
   */
  void push_back(std::function<T> func) { functions.push_back(func); }
  /**
   * @brief Execute all functions in the function_queue in the order of insertion.
   * 
   */
  void call(const ros::TimerEvent& e)
  {
    for (const auto & func : functions) {
      func();
    }
  }
  /**
   * @brief Get the function queue object. Hence, std::function of call() method.
   * 
   * @return std::function<T> of the call method.
   */
  std::function<void(const ros::TimerEvent&)> get_function_queue() { return std::bind(&function_queue::call, this, std::placeholders::_1); }

private:
  std::vector<std::function<T>> functions;
};
}  // namespace tam::core

#ifndef __UAV_UTILS_H
#define __UAV_UTILS_H

#include "rclcpp/rclcpp.hpp"

#include "converters.h"
#include "geometry_utils.h"

namespace uav_utils
{
/* judge if value belongs to [low,high] */
template <typename T, typename T2>
bool
in_range(rclcpp::Node::SharedPtr node,T value, const T2& low, const T2& high)
{
  RCLCPP_ASSERT(node->get_logger(),low < high, "%f < %f?", low, high);
  return (low <= value) && (value <= high);
}

/* judge if value belongs to [-limit, limit] */
template <typename T, typename T2>
bool
in_range(rclcpp::Node::SharedPtr node,T value, const T2& limit)
{
  RCLCPP_ASSERT(node->get_logger(),limit > 0, "%f > 0?", limit);
  return in_range(value, -limit, limit);
}

template <typename T, typename T2>
void
limit_range(rclcpp::Node::SharedPtr node,T& value, const T2& low, const T2& high)
{
  RCLCPP_ASSERT(node->get_logger(),low < high, "%f < %f?", low, high);
  if (value < low)
  {
    value = low;
  }

  if (value > high)
  {
    value = high;
  }

  return;
}

template <typename T, typename T2>
void
limit_range(rclcpp::Node::SharedPtr node,T& value, const T2& limit)
{
  RCLCPP_ASSERT(node->get_logger(),limit > 0, "%f > 0?", limit);
  limit_range(value, -limit, limit);
}

typedef std::stringstream DebugSS_t;
} // end of namespace uav_utils

#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace
{

constexpr double kDefaultTimeoutSec = 0.50;

const std::set<std::string> kSmartCarNodes = {
  "/task_node",
  "/safety_node",
  "/safety_node_cpp",
  "/direction_guard",
  "/direction_guard_node",
  "/origincar_base",
  "/ekf_filter_node",
  "/controller_server",
  "/planner_server",
  "/behavior_server",
  "/bt_navigator",
  "/smoother_server",
  "/velocity_smoother",
  "/lifecycle_manager_navigation",
  "/lifecycle_manager",
  "/aurora930_node",
  "/depth_pointcloud_relay",
  "/depth_pointcloud_to_laserscan",
  "/vision_node",
  "/barcode_reader",
  "/competition_output_display",
};

double parseTimeout(int argc, char ** argv)
{
  if (argc == 1) {
    return kDefaultTimeoutSec;
  }
  if (argc != 3 || std::string(argv[1]) != "--timeout-sec") {
    throw std::invalid_argument("usage: competition_stack_probe [--timeout-sec seconds]");
  }
  const double value = std::stod(argv[2]);
  if (!std::isfinite(value) || value <= 0.0 || value > 2.0) {
    throw std::invalid_argument("timeout must be greater than zero and at most two seconds");
  }
  return value;
}

std::string fullyQualifiedName(const std::string & name, const std::string & namespace_)
{
  if (namespace_ == "/") {
    return "/" + name;
  }
  return namespace_ + "/" + name;
}

std::set<std::string> findSmartCarNodes(const rclcpp::Node::SharedPtr & node)
{
  std::set<std::string> found;
  for (const auto & entry : node->get_node_graph_interface()->get_node_names_and_namespaces()) {
    const auto fullname = fullyQualifiedName(entry.first, entry.second);
    if (kSmartCarNodes.count(fullname) != 0U) {
      found.insert(fullname);
    }
  }
  return found;
}

void printNodes(const std::set<std::string> & nodes)
{
  bool first = true;
  for (const auto & node : nodes) {
    if (!first) {
      std::cout << ',';
    }
    std::cout << node;
    first = false;
  }
  std::cout << std::endl;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto timeout = std::chrono::duration<double>(parseTimeout(argc, argv));
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("competition_stack_probe");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::set<std::string> found;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      found = findSmartCarNodes(node);
      if (!found.empty()) {
        printNodes(found);
        executor.remove_node(node);
        rclcpp::shutdown();
        return 1;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    executor.remove_node(node);
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "competition stack probe failed: " << error.what() << std::endl;
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 2;
  }
}

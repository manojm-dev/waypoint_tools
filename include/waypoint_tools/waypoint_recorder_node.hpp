#ifndef WAYPOINT_TOOLS__WAYPOINT_RECORDER_NODE_HPP
#define WAYPOINT_TOOLS__WAYPOINT_RECORDER_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include <nlohmann/json.hpp>

class WaypointRecorder : public rclcpp::Node
{
public:
    WaypointRecorder();

private:
    // Core logic
    void callback();
    std::tuple<std::string, int, std::vector<std::string>, double, double> get_input(int index);
    char increment_alphabet(char start_ch, int increment_by);

    // Odometry handling
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void wait_for_odometry();

    // JSON 
    void save_to_json(
        const std::vector<std::string>& names,
        const std::vector<int>& ids,
        const std::vector<std::vector<std::string>>& neighbours,
        const std::vector<double>& x_pos,
        const std::vector<double>& y_pos,
        int index);

    // Parameterised variable
    std::string odom_topic_;
    int waypoint_limit_;

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Variables
    geometry_msgs::msg::Pose2D current_location_;
    bool odom_received_ = false;
    bool add_waypoint_ = true;
};


#endif // WAYPOINT_TOOLS__WAYPOINT_RECORDER_NODE_HPP
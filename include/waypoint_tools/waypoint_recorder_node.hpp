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

/**
 * @class WaypointRecorder
 * @brief A ROS2 node to interactively collect waypoints based on current odometry
 *        and save them with metadata like name, ID, neighbours to a JSON file.
 */
class WaypointRecorder : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for WaypointRecorder node
     */
    WaypointRecorder();

private:
    /** ---------------------------- Core Logic ---------------------------- **/

    /**
     * @brief Entry function that starts the waypoint collection loop
     */
    void callback();

    /**
     * @brief Gets user input to register a single waypoint
     * 
     * @param index Index of the current waypoint (used for ID and naming)
     * @return Tuple of (name, id, neighbours, x, y)
     */
    std::tuple<std::string, int, std::vector<std::string>, double, double> get_input(int index);

    /**
     * @brief Increments an alphabet letter (wraps around after 'Z')
     * 
     * @param start_ch Starting character (usually 'A')
     * @param increment_by Number of steps to increment
     * @return char Resulting character after increment
     */
    char increment_alphabet(char start_ch, int increment_by);


    /** -------------------------- Odometry Handling -------------------------- **/

    /**
     * @brief Odometry callback to update the robot's current position
     * 
     * @param msg Odometry message
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Spins the node until odometry has been received at least once
     */
    void wait_for_odometry();


    /** ---------------------------- JSON Output ---------------------------- **/

    /**
     * @brief Saves all collected waypoint data to a JSON file
     * 
     * @param names List of waypoint names
     * @param ids List of waypoint IDs
     * @param neighbours Adjacency list of neighbours
     * @param x_pos List of X coordinates
     * @param y_pos List of Y coordinates
     * @param index Number of waypoints collected
     */
    void save_to_json(
        const std::vector<std::string>& names,
        const std::vector<int>& ids,
        const std::vector<std::vector<std::string>>& neighbours,
        const std::vector<double>& x_pos,
        const std::vector<double>& y_pos,
        int index);


    /** ---------------------------- Class Members ---------------------------- **/

    // Parameters
    std::string odom_topic_;        ///< Topic to subscribe to odometry
    int waypoint_limit_;            ///< Maximum number of waypoints to collect

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;  ///< Odometry subscriber

    // State variables
    geometry_msgs::msg::Pose2D current_location_; ///< Current pose from odometry
    bool odom_received_ = false;                  ///< Flag to check if odometry was received
    bool add_waypoint_ = true;                    ///< Flag to control waypoint collection loop
};

#endif  // WAYPOINT_TOOLS__WAYPOINT_RECORDER_NODE_HPP
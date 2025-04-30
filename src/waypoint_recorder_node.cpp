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

#include <nlohmann/json.hpp>  // for JSON file handling

#include "waypoint_tools/waypoint_recorder_node.hpp"

using namespace std::chrono_literals;

// Constructor for the node
WaypointRecorder::WaypointRecorder() 
: Node("waypoint_publisher")
{
    // Declare parameters with default values
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("waypoint_limit", 100);

    // Get values from the parameter server
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    waypoint_limit_ = this->get_parameter("waypoint_limit").as_int();

    // Subscribe to odometry to track robot's position
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_,
        10,
        std::bind(&WaypointRecorder::odom_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Waypoint recorder node started!");

    // Start the waypoint collection process
    callback();
}

// Main loop to record multiple waypoints based on user input
void WaypointRecorder::callback()
{
    // Containers to store waypoint data
    std::vector<std::string> names(waypoint_limit_);
    std::vector<int> ids(waypoint_limit_);
    std::vector<std::vector<std::string>> neighbours(waypoint_limit_);
    std::vector<double> x_pos(waypoint_limit_), y_pos(waypoint_limit_);
    int i;

    // Loop until the user finishes or the limit is hit
    for (i = 0; i < waypoint_limit_ && add_waypoint_ == true; i++)
    {
        wait_for_odometry();  // Block until odometry is received
        auto [name, id, neighbour_list, x, y] = get_input(i);  // Collect input for one waypoint
        names[i] = name;
        ids[i] = id;
        neighbours[i] = neighbour_list;
        x_pos[i] = x;
        y_pos[i] = y;
    }

    // Save collected waypoints to a JSON file
    save_to_json(names, ids, neighbours, x_pos, y_pos, i);
}

// Save all waypoint data into a nicely formatted JSON file
void WaypointRecorder::save_to_json(
    const std::vector<std::string>& names,
    const std::vector<int>& ids,
    const std::vector<std::vector<std::string>>& neighbours,
    const std::vector<double>& x_pos,
    const std::vector<double>& y_pos,
    int index)
{
    nlohmann::json waypoints_json = nlohmann::json::array(); 

    for (int i = 0; i < index; ++i)
    {
        nlohmann::json waypoint;
        waypoint["name"] = names[i];
        waypoint["id"] = ids[i];
        waypoint["x"] = x_pos[i];
        waypoint["y"] = y_pos[i];
        waypoint["neighbours"] = neighbours[i];

        waypoints_json.push_back(waypoint);
    }

    // Write JSON to file
    std::ofstream file_out("waypoints.json");
    file_out << waypoints_json.dump(4);  // Pretty print with 4-space indent
    file_out.close();

    RCLCPP_INFO(this->get_logger(), "Waypoints saved to waypoints.json!");
}

// Handles user interaction to input a single waypoint's data
std::tuple<std::string, int, std::vector<std::string>, double, double> WaypointRecorder::get_input(int index)
{
    std::string confirm, waypoint_name;
    std::vector<std::string> neighbours;
    double x_pos = current_location_.x;
    double y_pos = current_location_.y;

    // Step 1: Ask user if this pose should be marked
    std::cout << "1) Mark current location (x: " << x_pos << ", y: " << y_pos << ") as a waypoint" << std::endl;
    std::cout << "Confirm (yes/no): ";
    std::cin >> confirm;
    if (confirm != "yes") {
        std::cout << "Skipping waypoint." << std::endl;
        return get_input(index);  // Retry if not confirmed
    }

    // Step 2: Set name (auto A, B, C...)
    waypoint_name = std::string(1, increment_alphabet('A', index));
    std::cout << "2) Waypoint name set as: " << waypoint_name << std::endl;

    // Step 3: Assign ID (same as index)
    int waypoint_id = index;
    std::cout << "3) Waypoint id set as: " << waypoint_id << std::endl;

    // Step 4: Input neighbours
    std::cout << "4) Enter neighbours (space separated names, press Enter to finish): ";
    std::cin.ignore();  // Clear newline in buffer
    std::string neighbour_input;
    std::getline(std::cin, neighbour_input);

    std::istringstream iss(neighbour_input);
    std::string neighbour;
    while (iss >> neighbour) {
        neighbours.push_back(neighbour);
    }

    // Log debug info
    std::cout << "LOGGING: Waypoint " << waypoint_name << " (id: " << waypoint_id << ") has neighbours: ";
    for (const auto& n : neighbours) {
        std::cout << n << " ";
    }
    std::cout << std::endl;

    // Ask if user wants to add another
    std::cout << "Added another waypoint" << std::endl;
    std::cout << "Confirm (yes/no): ";
    std::cin >> confirm;
    if (confirm != "yes") {
        std::cout << "Completed gathering data" << std::endl;
        add_waypoint_ = false;
    }

    return {waypoint_name, waypoint_id, neighbours, x_pos, y_pos};
}

// Utility: A->B->... wraparound alphabet based on index
char WaypointRecorder::increment_alphabet(char start_ch, int increment_by)
{
    int numeric_value = start_ch - 'A';
    numeric_value = (numeric_value + increment_by) % 26;
    return 'A' + numeric_value;
}

// Callback to update current position from odometry
void WaypointRecorder::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_location_.x = msg->pose.pose.position.x;
    current_location_.y = msg->pose.pose.position.y;
    odom_received_ = true;
}

// Spin until odometry is received
void WaypointRecorder::wait_for_odometry()
{
    rclcpp::Rate rate(10);  // 10 Hz
    odom_received_ = false;
    while (!odom_received_ && rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
    }
}

// Main entrypoint
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointRecorder>());
    rclcpp::shutdown();
    return 0;
}
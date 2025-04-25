#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "waypoint_tools/waypoint_recorder_node.hpp"

using namespace std::chrono_literals;

WaypointRecorder::WaypointRecorder() : 
    Node("waypoint_publisher")
    {
        WaypointRecorder::callback();
    }

void WaypointRecorder::callback()
    {
        RCLCPP_INFO(this->get_logger(), "Waypoint recorder node started!");
    }



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointRecorder>());
    rclcpp::shutdown();
    return 0;
}
#ifndef WAYPOINT_TOOLS__WAYPOINT_RECORDER_NODE_HPP
#define WAYPOINT_TOOLS__WAYPOINT_RECORDER_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class WaypointRecorder : public rclcpp::Node
{
    public:
        WaypointRecorder();

    private:
        void callback();
};

#endif // WAYPOINT_TOOLS__WAYPOINT_RECORDER_NODE_HPP
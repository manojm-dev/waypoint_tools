# waypoint_tools

![alt text](media/waypoint_json.png)

A ROS2 utility package to interactively record waypoints based on the robot's odometry and save them to a JSON file.

## [View Demo Video](https://drive.google.com/file/d/1KklIigYbCunJCCx7AhKZNd6-13tMK6ji/view?usp=sharing)

---

## 💡 What It Does

- Subscribes to odometry and logs robot's current 2D pose.
- Lets user name waypoints(#TODO) and assign neighbours interactively via terminal.
- Stores waypoint info as a structured JSON file (`waypoints.json`).
- Parameterized topic name and waypoint limit.

## 🧑‍💻 Setup

1. 📂 Clone the repository
```
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone  https://github.com/manojm-dev/waypoint_tools.git
```

2) 📦 Install dependencies
```
cd ~/ros_ws
sudo apt-get update -y && rosdep update && rosdep install --from-paths src --ignore-src -y
```

3) 🛠️ Building the packages
```
cd ~/ros_ws
colcon build
```

## 🚀 Run the Node

```bash
ros2 run waypoint_tools waypoint_recorder_node 
```

## 👨‍💻 Author
Made with ❤️ by Manoj M
# LIMO Custom Reverse Recovery Node

A custom recovery behavior for the AgileX LIMO robot in ROS Melodic. Cancels the goal, clears costmaps, reverses, spins, and resends the last goal.

## 🧩 Dependencies
- ROS Melodic
- `geometry_msgs`, `move_base_msgs`, `actionlib_msgs`,`std_srvs`

## 🚀 Launch Instructions

```bash
rosrun limo_bringup custom_recovery_node.py

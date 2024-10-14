/*
This file contains a node which creates an object of MoveGroup class.
A server is also created which accepts waypoints from client and passes on
the waypoints to the MoveGroup cartesian path planner which executes the motion.
*/

#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include "waypoint_msgs/srv/waypoint_follower.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"


// Node init

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // creating server
  auto const node = std::make_shared<rclcpp::Node>(
      "waypoints_server_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group = MoveGroupInterface(node, "ur_manipulator");
  static const std::string PLANNING_GROUP = "ur_manipulator";

  // ---------------------------------------------------------------------

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // move to initial position with vacuum gripper facing downwards
  std::vector<double> joint_group_positions;

  double deg_conversion = 3.14 / 180.0;
  joint_group_positions.push_back(304 * deg_conversion); // radians
  joint_group_positions.push_back(-120 * deg_conversion);
  joint_group_positions.push_back(-114 * deg_conversion);
  joint_group_positions.push_back(323 * deg_conversion);
  joint_group_positions.push_back(-270 * deg_conversion);
  joint_group_positions.push_back(34 * deg_conversion);
  bool within_bounds = arm_group.setJointValueTarget(joint_group_positions);

  bool success = (arm_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  arm_group.execute(my_plan);

  // -------------------------------------------------------------------

  // Callback function for the server
  auto server_cb = [&arm_group](const std::shared_ptr<waypoint_msgs::srv::WaypointFollower::Request> request,
                          std::shared_ptr<waypoint_msgs::srv::WaypointFollower::Response> response)
  {
   
    // waypoints are stored as PoseArray msg object
    std::vector<geometry_msgs::msg::Pose> pose_array;

    for (const auto &point : request->waypoints)
    {
      std::cout << "Point: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;

      geometry_msgs::msg::Pose pose1;
      pose1.position.x = point.x;
      pose1.position.y = point.y;
      pose1.position.z = point.z;
      pose1.orientation.x = -1.0;
      pose1.orientation.y = 0.00;
      pose1.orientation.z = 0.00;
      pose1.orientation.w = 0.00;
      pose_array.push_back(pose1);
    }

    moveit_msgs::msg::RobotTrajectory trajectory_approach;

    const double jump_threshold = 0.0;

    const double eef_step = 0.01;

    bool result_d;

    // fraction gives the percentage of path generated (0-1.0)
    double fraction = arm_group.computeCartesianPath(

        pose_array, eef_step, jump_threshold, trajectory_approach);

    std::cout << "Fraction: (" << fraction << ")" << std::endl;

    // if path genrated fully, only then execute. Otherwise cancel execution.
    if (fraction == 1.0)
    {
      arm_group.execute(trajectory_approach);
      result_d = true;
    }
    else
    {
      result_d = false;
    }

    response->result = result_d;

    if (response->result)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waypoints executed successfully");
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to execute waypoints");
    }
  };

  // ----------------------------------------------------------------
  rclcpp::Service<waypoint_msgs::srv::WaypointFollower>::SharedPtr service =
      node->create_service<waypoint_msgs::srv::WaypointFollower>("waypoint_follower", server_cb);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to follow waypoints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
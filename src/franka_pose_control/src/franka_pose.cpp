#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <chrono>

geometry_msgs::msg::Pose current_target_pose;
bool target_pose_updated = false;

void targetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    current_target_pose = *msg;
    target_pose_updated = true;
}

int main(int argc, char **argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("panda_cartesian_controller");

    // Subscriber for the target pose
    auto subscriber = node->create_subscription<geometry_msgs::msg::Pose>(
        "target_pose", 10, targetPoseCallback);

    // Initialize the TF2 buffer and listener
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Initialize MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm", tf_buffer);

    while (rclcpp::ok()) {
        if (target_pose_updated) {
            target_pose_updated = false; // Reset the flag

            // Plan Cartesian path
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(current_target_pose);

            moveit_msgs::msg::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

            if (fraction > 0.9) {
                // Execute the plan
                move_group.execute(trajectory);
            }
        }

        // Add a small delay and spin node
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}

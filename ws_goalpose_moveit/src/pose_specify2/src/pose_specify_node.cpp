#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread> 
int main(int argc, char * argv[])
{

    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "node1",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );


    // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("pose_specifying_node_");

    // Next step goes here
    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "newmanip_arm");




    // Get current end-effector pose
    geometry_msgs::msg::PoseStamped current_pose = move_group_interface.getCurrentPose();
    //std::cout << "Current Pose: " << current_pose << std::endl;  // won't work. ros2 msgs generally aren't defined for << operator
    // so here we are using logger to diplay it
    RCLCPP_INFO(
    rclcpp::get_logger("move_group_logger"),
    "Current Pose:\n frame_id=%s\n position=(%.3f, %.3f, %.3f)\n orientation=(%.3f, %.3f, %.3f, %.3f)",
    current_pose.header.frame_id.c_str(),
    current_pose.pose.position.x,
    current_pose.pose.position.y,
    current_pose.pose.position.z,
    current_pose.pose.orientation.x,
    current_pose.pose.orientation.y,
    current_pose.pose.orientation.z,
    current_pose.pose.orientation.w
    );




    // Set a target Pose
    auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 0.000;
    msg.orientation.y = 0.000;
    msg.orientation.z = 0.293;
    msg.orientation.w = 0.956;
    msg.position.x = 0.336;
    msg.position.y = -0.498;
    msg.position.z = 0.794;
    return msg;
    }();
    move_group_interface.setPoseTarget(target_pose);

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success) {
    move_group_interface.execute(plan);
    } else {
    RCLCPP_ERROR(logger, "Planning failed!");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    spinner.join();  // <--- Join the thread before exiting
    return 0;

}

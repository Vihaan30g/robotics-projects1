#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <string>
#include <cmath> 

// --- CONFIGURATION (Based on your output) ---
#define AXIS_LINEAR_Y  0   // Left Stick Left/Right
#define AXIS_LINEAR_X  1   // Left Stick Up/Down

// CHANGED: We are now using 2 and 3 because 4 and 5 are broken triggers
#define AXIS_ANGULAR_Z 2   // Right Stick Left/Right
#define AXIS_LINEAR_Z  3   // Right Stick Up/Down 

// --- DEADZONE SETTING ---
#define DEADZONE 0.2

class JoyToTwistNode : public rclcpp::Node
{
public:
  JoyToTwistNode() : Node("joy_to_twist_custom")
  {
    this->declare_parameter<std::string>("frame_id", "base_link");
    this->declare_parameter<std::string>("output_topic", "/servo_node/delta_twist_cmds");
    this->declare_parameter<double>("scale_linear", 0.4); // Slightly slower for safety
    this->declare_parameter<double>("scale_angular", 0.8);

    this->get_parameter("frame_id", frame_id_);
    std::string output_topic;
    this->get_parameter("output_topic", output_topic);
    this->get_parameter("scale_linear", scale_linear_);
    this->get_parameter("scale_angular", scale_angular_);

    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(output_topic, 10);
    sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyToTwistNode::joyCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "JoyToTwist Node Started. Z-Axis is now Axis 3.");
  }

private:
  double apply_deadzone(double raw_val) {
    if (std::abs(raw_val) < DEADZONE) return 0.0;
    return raw_val;
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();

    // Safety check: Ensure we have enough axes before reading
    if (msg->axes.size() < 4) return;

    // 1. Get Raw Values
    double raw_x  = msg->axes[AXIS_LINEAR_X];
    double raw_y  = msg->axes[AXIS_LINEAR_Y];
    double raw_az = msg->axes[AXIS_ANGULAR_Z];
    double raw_z  = msg->axes[AXIS_LINEAR_Z];

    // 2. Apply Deadzone & Scale
    // Note: You might need to add a minus sign (-) to some of these if directions are inverted!
    twist_msg->twist.linear.x  = apply_deadzone(raw_x) * scale_linear_;
    twist_msg->twist.linear.y  = apply_deadzone(raw_y) * scale_linear_;
    twist_msg->twist.linear.z  = apply_deadzone(raw_z) * scale_linear_; 
    twist_msg->twist.angular.z = apply_deadzone(raw_az) * scale_angular_;

    // 3. Publish
    twist_msg->header.stamp = this->now();
    twist_msg->header.frame_id = frame_id_;
    pub_->publish(std::move(twist_msg));
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  std::string frame_id_;
  double scale_linear_;
  double scale_angular_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToTwistNode>());
  rclcpp::shutdown();
  return 0;
}
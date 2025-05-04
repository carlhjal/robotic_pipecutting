#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_servo/servo.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class MoveItServoControlNode : public rclcpp::Node
{
public:
    MoveItServoControlNode() : Node("moveit_servo_control_node")
    {
        // Initialize MoveIt Servo (parameters and publishing are done here)
        moveit_servo::Servo servo(*this);

        // Set up a publisher for sending servo targets (pose or trajectory)
        servo_target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/servo_target", 10);

        // Set up a subscriber to receive joint state updates (optional, for feedback)
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&MoveItServoControlNode::jointStateCallback, this, std::placeholders::_1));

        // Example: Set a target pose (using geometry_msgs::PoseStamped)
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";  // Your reference frame
        target_pose.pose.position.x = 0.5;
        target_pose.pose.position.y = 0.0;
        target_pose.pose.position.z = 0.5;
        target_pose.pose.orientation.w = 1.0;

        // Publish the target pose to MoveIt Servo
        servo_target_pub_->publish(target_pose);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr servo_target_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    // Optional: Callback for joint state (for feedback)
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // You can use this callback to process feedback, for example
        RCLCPP_INFO(this->get_logger(), "Received joint states");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveItServoControlNode>());
    rclcpp::shutdown();
    return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class UR5HomePublisher : public rclcpp::Node
{
public:
  UR5HomePublisher() : Node("ur5_home_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_controller/commands", 10);

    // Give controller time to activate
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&UR5HomePublisher::publish_home, this));
  }

private:
  void publish_home()
  {
    std_msgs::msg::Float64MultiArray msg;

    msg.data = {
      0.0,        // shoulder_pan_joint
     -1.5708,     // shoulder_lift_joint
     -1.5708,     // elbow_joint
     -1.5708,     // wrist_1_joint
     1.5708,     // wrist_2_joint
      0.0         // wrist_3_joint
    };

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "UR5 home pose command sent.");

    // Publish once and stop
    timer_->cancel();
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UR5HomePublisher>());
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>
#include <cmath>
using namespace std::chrono_literals;

class SVGPathDrawer : public rclcpp::Node
{
public:
    SVGPathDrawer() : Node("svg_path_drawer"), index_(0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&SVGPathDrawer::followPath, this));

        // Scaled path points from the SVG
        path_ = {
            {1.0, 1.0}, {2.0, 2.0}, {3.0, 3.0}, {4.0, 5.0}, {6.0, 7.0},
            {8.0, 9.0}, {10.0, 10.0}, {11.0, 11.0}  // Simplified path points
        };
    }

private:
    void followPath()
    {
        if (index_ >= path_.size() - 1)
        {
            RCLCPP_INFO(this->get_logger(), "Finished drawing the path.");
            rclcpp::shutdown();
            return;
        }

        // Get current and next point
        auto current_point = path_[index_];
        auto next_point = path_[index_ + 1];

        // Compute the distance and angle to the next point
        double dx = next_point[0] - current_point[0];
        double dy = next_point[1] - current_point[1];
        double distance = std::sqrt(dx * dx + dy * dy);
        double angle = std::atan2(dy, dx);

        // Create a Twist message for the turtle's movement
        auto msg = geometry_msgs::msg::Twist();

        // Move the turtle towards the next point
        msg.linear.x = distance / 0.1;   // Set linear speed proportional to distance
        msg.angular.z = angle;           // Rotate towards the next point

        // Publish the movement command
        publisher_->publish(msg);

        // Move to the next point
        index_++;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::vector<double>> path_;  // Path points
    size_t index_;  // Current index in the path
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SVGPathDrawer>());
    rclcpp::shutdown();
    return 0;
}

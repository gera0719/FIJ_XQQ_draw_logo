#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class DrawLogo : public rclcpp::Node{
    private:

    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_on_off_c;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub;
    rclcpp::TimerBase::SharedPtr timer;
    std::vector<std::pair<double, double>> vectors;
    std::vector<std::pair<double, double>> coordinates_t;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_c;
    size_t current_move;
    

    void pen_on_off(bool on){
        auto on_off = std::make_shared<turtlesim::srv::SetPen::Request>();
        on_off->b = 0;
        on_off->g = 0;
        on_off->r = 0;
        on_off->width = 8;
        on_off->off = !on;
        pen_on_off_c->async_send_request(on_off, [this](rclcpp::Client<turtlesim::srv::SetPen>::SharedFuture)
        {
            RCLCPP_INFO(this->get_logger(), "pen settings has been changed");
        });
    }
    void teleport(double x, double y){
        
        auto teleport = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        teleport->x = x; 
        teleport->y = y;
        teleport->theta = 0.0;
        teleport_c->async_send_request(teleport, [this](rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedFuture)
        {
            RCLCPP_INFO(this->get_logger(), "teleportation has been completed");
            
        });
    }
    
    void move_turtle(double distance, double speed = 1.0){
        geometry_msgs::msg::Twist msg;
        msg.linear.x = speed;
        rclcpp::Time start_time = this->now();
        double duration = distance / std::abs(speed);
        while ((this->now() - start_time).seconds() < duration) {
            velocity_pub->publish(msg);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
        
        msg.linear.x = 0.0;
        velocity_pub->publish(msg);
    }

    void control_turtle(){
        if(current_move < coordinates_t.size()){
            pen_on_off(false);
            teleport(coordinates_t[current_move].first, coordinates_t[current_move].second);
            pen_on_off(true);
            move_turtle(vectors[current_move].first, 1.0);
            current_move++;
            std::cout << "move: " << current_move << " of " << vectors.size() << std::endl;
        }else{
            rclcpp::shutdown();
        }
    }

    public:

    DrawLogo() : Node("draw_logo"), current_move(0){
        
        velocity_pub = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pen_on_off_c = this->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");
        teleport_c = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
    
        vectors = {
            //lines -> {x, y of vector}
            {5.7, 0.0},
            {4.2, 0.0},
            {5.7, 0.0},
            //lower three long lines done
            {1.5, 0.0},
            {1.5, 0.0},
            {1.5, 0.0},
            {1.5, 0.0},
            {1.5, 0.0},
            {1.5, 0.0},
            {1.5, 0.0},
            {1.5, 0.0},
            {1.5, 0.0},
            //*9 short lines done

            //only the curves are left to do
        };
        //teleport coords
        coordinates_t = {
            {1.0, 2.0},
            {2.5, 2.75},
            {1.0, 3.5},
            {7.5, 2.0},
            {7.5, 2.75},
            {7.5, 3.5},
            {7.5, 4.25},
            {7.5, 5.0},
            {7.5, 5.75},
            {7.5, 6.5},
            {7.5, 7.25},
            {7.5, 8.0},
            //only the curves are left to do
        };
        //continus impulse
        timer = this->create_wall_timer(
            1.25s, std::bind(&DrawLogo::control_turtle, this));
    };
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrawLogo>());
    rclcpp::shutdown();
    return 0;
}

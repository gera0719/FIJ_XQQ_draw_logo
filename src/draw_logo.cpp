#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include <chrono>
#include <cmath>

class DrawLogo : public rclcpp::Node{
    private:

    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_on_off_c;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub;
    rclcpp::TimerBase::SharedPtr timer;
    std::vector<std::pair<double, double>> vectors;
    std::vector<std::pair<double, double>> coordinates_t;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_c;
    

    

    void pen_on_off(bool on){
        auto on_off = std::make_shared<turtlesim::srv::SetPen::Request>();
        on_off->b = 0;
        on_off->g = 0;
        on_off->r = 0;
        on_off->width = 8;
        on_off->off = !on;
        pen_on_off_c->async_send_request(on_off, [this](rclcpp::Client<turtlesim::srv::SetPen>::SharedFuture)
        {
            RCLCPP_INFO(this->get_logger(), "pen setting has been changed");
        });
    }
    void teleport(double x, double y){
        pen_on_off(false);
        auto teleport = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        teleport->x = 1.0; 
        teleport->y = 1.0;
        teleport->theta = 0.0;
        teleport_c->async_send_request(teleport, [this](rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedFuture)
        {
            RCLCPP_INFO(this->get_logger(), "teleportation has been completed");
            pen_on_off(true);
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
        // Stop the turtle
        msg.linear.x = 0.0;
        velocity_pub->publish(msg);
    }

    void control_turtle(){
        // 0.3 unit left
        pen_on_off(true);
        RCLCPP_INFO(this->get_logger(), "0.3 left, pen on");
        move_turtle(0.3);

        // pen off
        RCLCPP_INFO(this->get_logger(), "pen off");
        pen_on_off(false);
        

        
        RCLCPP_INFO(this->get_logger(), "0.3 left, pen off");
        move_turtle(0.3);

        
        RCLCPP_INFO(this->get_logger(), "pen on");
        pen_on_off(true);
        

        
        RCLCPP_INFO(this->get_logger(), "0.3 left, pen on");
        move_turtle(0.3);
    }
    
    public:

    DrawLogo() : Node("draw_logo"){
        
        velocity_pub = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pen_on_off_c = this->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");
        teleport_c = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");

        //the easiest way to control the turtle will be, if we'd stored the vectors of the movement in a vector
        

        vectors = {
            //lines -> {x, y of vector}
            /*teleport to starting point (-4.0, -3.0) */{5.7, 0.0},
            /*teleport to (-2.5, -2.25)*/{4.2, 0.0},
            /*teleport to (-4.0, -1.5)*/{5.7, 0.0},
            //lower three long lines done
            /*teleport to (2.5, -3.0)*/{1.5, 0.0},
            /*teleport to (2.5, -2.25)*/{1.5, 0.0},
            /*teleport to (2.5, -1.5)*/{1.5, 0.0},
            /*teleport to (2.5, -0.75)*/{1.5, 0.0},
            /*teleport to (2.5, 0.0)*/{1.5, 0.0},
            /*teleport to (2.5, 0.75)*/{1.5, 0.0},
            /*teleport to (2.5, 1.5)*/{1.5, 0.0},
            /*teleport to (2.5, 2.25)*/{1.5, 0.0},
            /*teleport to (2.5, 3.0)*/{1.5, 0.0},
            //*9 short lines done

            /*teleport to (-2.5, -0.75)*/{5.6569, 0,/*i don't know that yet :C*/},
            /*teleport to (-2.5, 0.0)*/{4.5962, 0}

        };
        //storing the coordinates for teleporting in a vector
        coordinates_t = {
            {-4.0, -3.0},
            {-2.5, -2.25},
            {-4.0, -1.5},

            {2.5, -3.0},
            {2.5, -2.25},
            {2.5, -1.5},
            {2.5, -0.75},
            {2.5, 0.0},
            {2.5, 0.75},
            {2.5, 1.5},
            {2.5, 2.25},
            {2.5, 3.0},

            {-2.5, -0.75},
            {-2.5, 0.0}

        };


        // cooldown after init
        timer = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&DrawLogo::control_turtle, this));

    };
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrawLogo>());
    rclcpp::shutdown();
    return 0;
}

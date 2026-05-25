#include <chrono>
#include <memory>
#include <tf2/utils.h>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

struct Waypoint {
    double x;
    double y;
};

class ILOSGuidance : public rclcpp::Node
{
public:
    ILOSGuidance() : Node("ilos_guidance")
    {
        this->declare_parameter("acceptance_radius", 5.0);
        this->declare_parameter("delta_lookahead", 25.0);
        this->declare_parameter("k", 0.001);
        this->declare_parameter("ref_u", 1.0);

        
        acceptance_radius = this->get_parameter("acceptance_radius").as_double();
        delta_lookahead = this->get_parameter("delta_lookahead").as_double();
        k = this->get_parameter("k").as_double(); 
        ref_u = this->get_parameter("ref_u").as_double();
        
        k_plos = 1.0 / delta_lookahead;
        k_ilos = k * k_plos;
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&ILOSGuidance::odometry_callback, this, std::placeholders::_1));

        
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "planned_path", 10, std::bind(&ILOSGuidance::path_callback, this, std::placeholders::_1));

        // Publish the reference for lqr_control.cpp
        pub_ref_psi_ = this->create_publisher<std_msgs::msg::Float64>("/control/psi_reference_input", 10);
        pub_ref_u_ = this->create_publisher<std_msgs::msg::Float64>("/control/u_reference_input", 10);
        // Timer for guidance
        timer_ = this->create_wall_timer(100ms, std::bind(&ILOSGuidance::guidanceLoop, this));

        
        
        RCLCPP_INFO(this->get_logger(), "Node ILOS Guidance initiated.");  
    }
    
private:
    // Parameters from control_parameters.yaml
    double delta_lookahead;
    double k;   
    double acceptance_radius; 
    double ref_u;

    //Calculated parameters
    double k_plos;
    double k_ilos;
    // state variables
    std::vector<Waypoint> waypoints;
    int current_idx = 0;
    double y_int = 0.0; 
    double last_time = 0.0;
    double x_ship = 0.0;
    double y_ship = 0.0;

    

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ref_psi_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ref_u_;
    rclcpp::TimerBase::SharedPtr timer_;

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        x_ship = msg->pose.pose.position.x;
        y_ship = msg->pose.pose.position.y;
        
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (!waypoints.empty()) {
            return; 
        }

        // if the list is not empty, fill it with all the waypoints
        for (const auto& pose_stamped : msg->poses) {
            waypoints.push_back({pose_stamped.pose.position.x, pose_stamped.pose.position.y});
        }
        
        current_idx = 0;
        RCLCPP_INFO(this->get_logger(), "Received new path with %zu waypoints.", waypoints.size());
    }

    void guidanceLoop() {
        double current_time = this->now().seconds();
        double dt = current_time - last_time;
        last_time = current_time;

        if (dt <= 0) dt = 0.1;

        // Calculate the new reference 
        double ref_psi = calculate_ilos(x_ship, y_ship, dt);
        
        
        auto msg = std_msgs::msg::Float64();
        msg.data = ref_psi;

        auto msgu= std_msgs::msg::Float64();
        msgu.data = ref_u;
        
       
        pub_ref_psi_->publish(msg);
        pub_ref_u_->publish(msgu);
    }

    double calculate_ilos(double x_s, double y_s, double dt) {
        if (waypoints.empty()) return 0.0;

        Waypoint p_k = waypoints[current_idx];
        Waypoint p_next = waypoints[current_idx + 1];

        // Angle of the path
        
        double dx = p_next.x - p_k.x;
        double dy = p_next.y - p_k.y;
        double pi_p = atan2(dy, dx); 

        // side error
        double ye = -(x_s - p_k.x) * sin(pi_p) + (y_s - p_k.y) * cos(pi_p); 

        // Integral term
        double y_dot = (delta_lookahead * ye) / 
                       (pow(delta_lookahead, 2) + pow(ye + k * y_int, 2)); 
        y_int += y_dot * dt;

        // Law of guidance
        double ref_psi = pi_p - atan(k_plos * ye + k_ilos * y_int); 

        // acceptance circle
        double dist_to_next = sqrt(pow(p_next.x - x_s, 2) + pow(p_next.y - y_s, 2));
        if (dist_to_next < acceptance_radius) { 
            if(current_idx < (int)waypoints.size() - 1)
                current_idx++;
            else
                current_idx = 0;
            
            RCLCPP_INFO(this->get_logger(), "Waypoint alcanzado. Siguiente: %d", current_idx);
            RCLCPP_INFO(this->get_logger(), "Posición: x:%.2f y:%.2f",x_ship,y_ship);
        }
        
        return ref_psi;
    }
    
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ILOSGuidance>());
    rclcpp::shutdown();
    return 0;
}
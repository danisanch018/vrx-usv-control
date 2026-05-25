#include <chrono>
#include <memory>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp" // Opcional, por si luego quieres publicar comandos
#include "std_msgs/msg/float64.hpp"
#include "ros_gz_interfaces/msg/entity_wrench.hpp"

using namespace std::chrono_literals;

class ControlUsv : public rclcpp::Node
{
public:
    ControlUsv() : Node("control_usv")
    {
        this->declare_parameter("Hi_00", 0.0);
        this->declare_parameter("Hi_01", -45.3084);
        this->declare_parameter("Hi_10", -144.2916);
        this->declare_parameter("Hi_11", 0.0);

        this->declare_parameter("Hc_00", 0.0);
        this->declare_parameter("Hc_01", 834.2);
        this->declare_parameter("Hc_02", 0.0);
        this->declare_parameter("Hc_03", 0.0);
        this->declare_parameter("Hc_10", 4682.8);
        this->declare_parameter("Hc_11", 0.0);
        this->declare_parameter("Hc_12", 115.3);
        this->declare_parameter("Hc_13", 2646.2);

        //Get the gain parameters
        Hi[0][0] = this->get_parameter("Hi_00").as_double();
        Hi[0][1] = this->get_parameter("Hi_01").as_double();
        Hi[1][0] = this->get_parameter("Hi_10").as_double();
        Hi[1][1] = this->get_parameter("Hi_11").as_double();

        Hc[0][0] = this->get_parameter("Hc_00").as_double();
        Hc[0][1] = this->get_parameter("Hc_01").as_double();
        Hc[0][2] = this->get_parameter("Hc_02").as_double();
        Hc[0][3] = this->get_parameter("Hc_03").as_double();
        Hc[1][0] = this->get_parameter("Hc_10").as_double();
        Hc[1][1] = this->get_parameter("Hc_11").as_double();
        Hc[1][2] = this->get_parameter("Hc_12").as_double();
        Hc[1][3] = this->get_parameter("Hc_13").as_double();

        RCLCPP_INFO(this->get_logger(), "LQR Gains loaded:");
        RCLCPP_INFO(this->get_logger(), "Hi: [[%f, %f], [%f, %f]]", Hi[0][0], Hi[0][1], Hi[1][0], Hi[1][1]);
        RCLCPP_INFO(this->get_logger(), "Hc row 0: [%f, %f, %f, %f]", Hc[0][0], Hc[0][1], Hc[0][2], Hc[0][3]);
        RCLCPP_INFO(this->get_logger(), "Hc row 1: [%f, %f, %f, %f]", Hc[1][0], Hc[1][1], Hc[1][2], Hc[1][3]);
        // Subscription to odometry
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&ControlUsv::odometryCallback, this, std::placeholders::_1));
        
        sub_ref_ = this->create_subscription<std_msgs::msg::Float64>(
            "/control/psi_reference_input", 10, 
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                this->ref_psi = msg->data;
                RCLCPP_INFO(this->get_logger(), "Update reference psi: %f", this->ref_psi);
            });
        sub_refu_ = this->create_subscription<std_msgs::msg::Float64>(
            "/control/u_reference_input", 10, 
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                this->ref_u = msg->data;
                RCLCPP_INFO(this->get_logger(), "Update reference u: %f", this->ref_u);
            });    
        // Timer for control loop
        timer_ = this->create_wall_timer(
            100ms, std::bind(&ControlUsv::controlLoop, this));
        pub_left_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/thrust", 10);
        pub_right_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/thrust", 10); 
        
        pub_psi_transformed_ = this->create_publisher<std_msgs::msg::Float64>("/control/psi_transformed", 10);
        pub_ref_psi_ = this->create_publisher<std_msgs::msg::Float64>("/control/psi_reference", 10);
        pub_ref_u_ = this->create_publisher<std_msgs::msg::Float64>("/control/u_reference", 10);
        //new
       pub_pert_ = this->create_publisher<ros_gz_interfaces::msg::EntityWrench>("/model/wamv/wrench", 10);
        start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Nodo Control USV iniciado.");
    }

private:
    rclcpp::Time start_time_;
    // Variables de estado
    double U = 0.0;   // Linear velocity x
    double V = 0.0;   // Linear velocity y
    double R = 0.0;   // angular velocity z
    double PSI = 0.0; // (Yaw)
    double Hi[2][2];
    double Hc[2][4];

    //  double Hi[2][2] = {{0,-45.3084},{-144.2916,0}};
    // double Hc[2][4] = {{0, 834.2, 0, 0},
    //                    {4682.8, 0, 115.3 , 2646.2}};
    //conf 1
//     double Hi[2][2] = {{0.0, -254.0265},
//                    {-278.2239, 0.0}};

// double Hc[2][4] = {{0.0, 824.2, 0.0, 0.0},
//                    {3421.1, 0.0, -0.4, 1117.7}};
//     //conf 2
//     double Hi[2][2] = {{0.0, -169.8097},
//                    {-195.2948, 0.0}};

// double Hc[2][4] = {{0.0, 850.2, 0.0, 0.0},
//                    {3493.9, 0.0, -0.5, 1173.4}};

//     //conf 3
//     double Hi[2][2] = {{0.0, -191.5911},
//                    {-293.7615, 0.0}};

// double Hc[2][4] = {{0.0, 1327.0, 0.0, 0.0},
//                    {4883.5, 0.0, -0.4, 1545.5}};               

    double ueq = 1.0;
    double Xseq = 250;
    double Nseq = 0;
    double Xs = 0;
    double Ns = 0;
    double xs = 0;
    double ns = 0;
    double ref_psi = 0;
    double ref_u = ueq;
    double ref_psi_1 = 0;
    double ref_u_1 = 0;

    double psi = 0;
    double u = 0; 
    double v = 0;
    double r = 0;

    double psi_1 = 0;
    double u_1 = 0; 
    double PSI_1 = 0;
    double U_1 = 0; 
    double v_1 = 0;
    double r_1 = 0;
    double Fi = 0;
    double Fd = 0;

    double zkpsi = 0;
    double zkpsi_1 = 0;
    bool psi_initialized_ = false;
    bool saturated = false;

    double zku = 0;
    double zku_1 = 0;
    double prev_raw_yaw = 0.0;
    int yaw_revolutions = 0;
    bool time_initialized_ = false;


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_ref_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_refu_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_psi_transformed_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ref_psi_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ref_u_;
    rclcpp::Publisher<ros_gz_interfaces::msg::EntityWrench>::SharedPtr pub_pert_;
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        U = msg->twist.twist.linear.x;
        V = msg->twist.twist.linear.y;
        R = msg->twist.twist.angular.z;

        double raw_yaw = tf2::getYaw(msg->pose.pose.orientation);
        
        // First time initilization
        if (!psi_initialized_) {
            PSI = raw_yaw;
            prev_raw_yaw = raw_yaw;
            psi_initialized_ = true;
            return;
        }

        double delta = raw_yaw - prev_raw_yaw;

        // Unwrapping (physical jump > π is imposible)
        if (delta > M_PI)       delta -= 2.0 * M_PI;
        else if (delta < -M_PI) delta += 2.0 * M_PI;

        PSI += delta;  //this allows PSI to move beyond +pi and -pi (continuous angle)
        prev_raw_yaw = raw_yaw;
    }

    void controlLoop()
    {
       
        if (!time_initialized_) {
        if (this->now().nanoseconds() > 0) {
            start_time_ = this->now();
            time_initialized_ = true;
           
        }
        return; 
    }
       double error_psi = atan2(sin(ref_psi_1 - PSI_1), cos(ref_psi_1 - PSI_1));
       
        if ((this->now() - start_time_).seconds() < 2.5)
        {
            // Xs = Xseq;
            // Ns = Nseq;
            // zkpsi = 0;
            // zku = 0;
        }
        else
        {
            if (std::abs(error_psi) < 0.005) {
                    error_psi = 0.0; //fix noise a little
                }
                zkpsi = zkpsi_1 + error_psi;
                zku = zku_1 + (ref_u_1 - U_1);
                u = U - ueq;
                v =  V - 0;
                r = R - 0;
                psi = PSI - 0;
                
                

                xs = -(Hi[0][0]*zkpsi+ Hi[0][1]*zku)-(Hc[0][0]*psi + Hc[0][1]*u + Hc[0][2]*v + Hc[0][3]*r);
                ns = -(Hi[1][0]*zkpsi+ Hi[1][1]*zku)-(Hc[1][0]*psi + Hc[1][1]*u + Hc[1][2]*v + Hc[1][3]*r);

                Xs = xs + Xseq;
                Ns = ns + Nseq;
         }
    
        publish_commands(error_psi);

        //Publication of reference and PSI
        auto msg_psi = std_msgs::msg::Float64();
        auto msg_ref = std_msgs::msg::Float64();
        msg_psi.data = PSI;
        // msg_ref.data = ref_psi;
        pub_psi_transformed_->publish(msg_psi);
        //pub_ref_psi_->publish(msg_ref);
        // Continuous reference
        double ref_psi_display = ref_psi;
        double diff = PSI - ref_psi_display;
        // Rounds to the nearest multiple of 2π 
        ref_psi_display += std::round(diff / (2.0 * M_PI)) * 2.0 * M_PI;
        msg_ref.data = ref_psi_display;
        pub_ref_psi_->publish(msg_ref);

        auto msg_refu = std_msgs::msg::Float64();
        msg_refu.data = ref_u;
        pub_ref_u_->publish(msg_refu);
        
  ///******************************** */
  //Update variables
        ref_psi_1 = ref_psi;
        ref_u_1 = ref_u;
        PSI_1 = PSI;
        U_1 = U;
        zkpsi_1 = zkpsi;
        zku_1 = zku;
        
    }

    void publish_commands(double error_psi)
    {
        Fi = Xs/2 - Ns/2.06;
        Fd = Xs/2 + Ns/2.06;

        if(Fd > 2000){
            Fd = 2000;
            saturated = true;
        }
        else if(Fd < -2000){
            Fd = -2000;
            saturated = true;
        }
        if(Fi > 2000){
            Fi = 2000;
            saturated = true;
        }
        else if(Fi < -2000){
            Fi = -2000;
            saturated = true;
        }

        if (saturated){
            zkpsi = zkpsi_1 - error_psi;
            zku = zku_1 - (ref_u_1 - U_1);
            saturated = false;
        }
        auto msg_left = std_msgs::msg::Float64();
        auto msg_right = std_msgs::msg::Float64();

        msg_left.data = Fi;
        msg_right.data = Fd;

        pub_left_->publish(msg_left);
        pub_right_->publish(msg_right);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlUsv>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/quaternion.hpp"  // Added for quaternion
#include "tf2/LinearMath/Quaternion.h"       // Added for quaternion
#include "tf2/LinearMath/Matrix3x3.h"        // Added for quaternion
#include <memory>
#include <thread>
#include <mutex>  // For std::mutex

#include <fstream> // 引入文件操作库

class OdometrySubscriber : public rclcpp::Node
{
public:
    OdometrySubscriber() : Node("odometry_subscriber")
    {
        // Declare parameters
        this->declare_parameter<double>("p_gain", 1.0);
        this->declare_parameter<double>("d_gain", 1.0);
        this->declare_parameter<double>("depth", 2.0);

        // Get parameter values
        p_ = this->get_parameter("p_gain").as_double();
        d_ = this->get_parameter("d_gain").as_double();
        depth_=this->get_parameter("depth").as_double();
        
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/equipped_tethys/odometry", 1, std::bind(&OdometrySubscriber::topic_callback, this, std::placeholders::_1));

        // Create a publisher for the cmd_vel topic with Float64 type
        cmd_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/horizontal_pos", 1);

        // Create a separate thread for the PD algorithm
        pd_thread_ = std::thread(&OdometrySubscriber::pd_algorithm, this);
        
        // 打开文件准备写入，如果文件不存在则创建
        data_file_.open("odom_data.csv", std::ios::out | std::ios::app);
        // 写入文件头
        data_file_ << "Time,z_position,linear_velocity,pd_output\n";
    }

    ~OdometrySubscriber()
    {
        // Join the thread on destruction
        pd_thread_.join();
         // 关闭文件
        if (data_file_.is_open()) {
            data_file_.close();
        }
    }

private:
    void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Accessing z position and linear velocity
        double z_position = msg->pose.pose.position.z;
        double linear_velocity = msg->twist.twist.linear.z;

        // Convert quaternion to Euler angles
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Use roll, pitch, and yaw as needed

        // Calculate PD control output
        double pd_output = p_ * (depth_-z_position) + d_ * linear_velocity;
        
        if(pd_output>0.26)  pd_output=0.26;
        if(pd_output<-0.26)  pd_output=-0.26;
        
        // Log the value using RCLCPP_INFO
       RCLCPP_INFO(this->get_logger(), "PD Output: %f", pd_output);
       RCLCPP_INFO(this->get_logger(), "z_position: %f", z_position);
       RCLCPP_INFO(this->get_logger(), "linear_veloity: %f", linear_velocity);
        
        // Create a Float64 message for publishing
        auto cmd_vel_msg = std::make_unique<std_msgs::msg::Float64>();
        cmd_vel_msg->data = pd_output;

        // Publish the PD control output
        cmd_vel_publisher_->publish(std::move(cmd_vel_msg));
        
        // 计算PD控制输出后，将数据写入文件
        data_file_ << this->now().seconds() << "," << z_position << "," << linear_velocity << "," << pd_output << "\n";
    }

    void pd_algorithm()
    {
        while (rclcpp::ok())
        {
            // Sleep for a short duration to control the PD loop frequency
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_vel_publisher_;
    std::thread pd_thread_;
    
    std::ofstream data_file_; // 文件流对象
    
    double p_;
    double d_;
    double depth_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometrySubscriber>());
    rclcpp::shutdown();
    return 0;
}


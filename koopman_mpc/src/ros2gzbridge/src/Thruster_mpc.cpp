#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/quaternion.hpp"  // Added for quaternion
#include "tf2/LinearMath/Quaternion.h"       // Added for quaternion
#include "tf2/LinearMath/Matrix3x3.h"        // Added for quaternion
#include <memory>
#include <thread>
#include <mutex>  // For std::mutex

#include <Eigen/Dense>


#include <qpOASES.hpp> //用来优化，mpc 使用
#include "/usr/local/MATLAB/R2023b/extern/include/mat.h"        // 引入MATLAB引擎库

class MPCControllerNode : public rclcpp::Node
{
public:
   MPCControllerNode() : Node("mpc_controller_node")
    {
        
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/equipped_tethys/odometry", 1, std::bind(&MPCControllerNode::topic_callback, this, std::placeholders::_1));

        // Create a publisher for the cmd_vel topic with Float64 type
        cmd_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/tethys/propeller_cml", 1);

        
       
       // 初始化 qpOASES solver
        initialize_mpc();
         
      

        // 创建一个线程用于执行MPC算法
        mpc_thread_ = std::thread(&MPCControllerNode::mpc_algorithm, this);
    }

    ~MPCControllerNode()
    {
        // Join the thread on destruction
        mpc_thread_.join();
        
       
    }

private:

  void initialize_mpc()
    {
     
        try
        {
            // 加载.mat文件
            MATFile *pmat = matOpen("/home/dingding/MatlabSimul/koopman/KoopmanMPC/mpc_matrix.mat", "r");
            if (pmat == nullptr)
            {
                RCLCPP_ERROR(this->get_logger(), "Error opening .mat file");
                return;
            }

            // 从加载的数据中获取变量并进行进一步处理
            mxArray *Alift_mat = matGetVariable(pmat, "Alift");
            mxArray *Blift_mat = matGetVariable(pmat, "Blift");
            mxArray *Clift_mat = matGetVariable(pmat, "Clift");

            // 将MATLAB矩阵数据转换为Eigen矩阵
            Eigen::MatrixXd Alift, Blift, Clift;
            // 实现从 mxArray 到 Eigen::MatrixXd 的转换
            // 请根据 mxArray 的实际类型和形状来进行适当的转换
            int Dimension_Alift_row=3;
            int Dimension_Alift_colum=3;
            int Dimension_Blift_row=3;
            int Dimension_Blift_colum=3;
            int Dimension_Clift_row=1;
            int Dimension_Clift_colum=3;
            
         if (mxIsDouble(Alift_mat) && mxGetM(Alift_mat) == Dimension_Alift_row && mxGetN(Alift_mat) == Dimension_Alift_colum)
        {
            Alift = Eigen::Map<Eigen::MatrixXd>(mxGetPr(Alift_mat), mxGetM(Alift_mat), mxGetN(Alift_mat));
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid Alift variable in .mat file");
        }
        
        if (mxIsDouble(Blift_mat) && mxGetM(Blift_mat) == Dimension_Blift_row && mxGetN(Blift_mat) == Dimension_Blift_colum)
        {
            Blift = Eigen::Map<Eigen::MatrixXd>(mxGetPr(Blift_mat), mxGetM(Blift_mat), mxGetN(Blift_mat));
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid Blift variable in .mat file");
        }
        
        if (mxIsDouble(Clift_mat) && mxGetM(Clift_mat) == Dimension_Clift_row && mxGetN(Clift_mat) == Dimension_Clift_colum)
        {
            Clift = Eigen::Map<Eigen::VectorXd>(mxGetPr(Clift_mat), mxGetM(Clift_mat) * mxGetN(Clift_mat));
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid Clift variable in .mat file");
        }
            // 释放MATLAB引擎内存
            mxDestroyArray(Alift_mat);
            mxDestroyArray(Blift_mat);
            mxDestroyArray(Clift_mat);
            matClose(pmat);

            RCLCPP_INFO(this->get_logger(), "Loaded variables from /home/dingding/MatlabSimul/koopman/KoopmanMPC/mpc_matrix.mat file");
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "Error loading .mat file");
        }
 
       
        // 这里定义QP问题的大小和相关参数
       /* int nV = Dimension_Alift_row; // 变量数量
        int nC = 0; // 约束数量

        H = new qpOASES::real_t[nV * nV] {1.0, 0.0, 0.0, 1.0}; // H矩阵
        g = new qpOASES::real_t[nV] {0.0, 0.0}; // g向量
        A = nullptr; // 没有线性约束
        lb = new qpOASES::real_t[nV] {-1.0, -1.0}; // 控制输入的下界
        ub = new qpOASES::real_t[nV] {1.0, 1.0}; // 控制输入的上界
        lbA = nullptr;
        ubA = nullptr;

        options = new qpOASES::Options();
        options->setToMPC();
        options->printLevel = qpOASES::PL_LOW;
        qp = new qpOASES::QProblem(nV, nC);
        qp->setOptions(*options);*/
    }

    void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Accessing z position and linear velocity
        double x_position = msg->pose.pose.position.x;
        double x_linear_velocity = msg->twist.twist.linear.x;
        
        RCLCPP_INFO(this->get_logger(), "x_position: %f", x_position);
        RCLCPP_INFO(this->get_logger(), "x_linear_veloity: %f", x_linear_velocity);
        //
        // 计算MPC控制器的输入
        
       /* int nWSR = 10; // 可调工作集重定位的最大次数
        qp->init(H, g, A, lb, ub, lbA, ubA, nWSR);

        qpOASES::real_t xOpt[2];
        qp->getPrimalSolution(xOpt);

        // 使用xOpt更新控制输入
        auto cmd_vel_msg = std::make_unique<std_msgs::msg::Float64>();
         cmd_vel_msg->data = xOpt[0]; // 假设我们只用第一个控制变量
        
        cmd_vel_publisher_->publish(std::move(cmd_vel_msg));
        RCLCPP_INFO(this->get_logger(), "MPC Output: %f", cmd_msg.data);
        
        // if(pd_output>0.26)  pd_output=0.26;
        // if(pd_output<-0.26)  pd_output=-0.26; 
        // Log the value using RCLCPP_INFO
       
        // Create a Float64 message for publishing
        
        // Publish the PD control output*/
        
       
    }

    void mpc_algorithm()
    {
        while (rclcpp::ok())
        {
            // Sleep for a short duration to control the PD loop frequency
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_vel_publisher_;
    std::thread mpc_thread_;
    
    
     qpOASES::QProblem *qp;
    qpOASES::Options *options;
    qpOASES::real_t *H, *g, *A, *lb, *ub, *lbA, *ubA;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCControllerNode>());
    rclcpp::shutdown();
    return 0;
}


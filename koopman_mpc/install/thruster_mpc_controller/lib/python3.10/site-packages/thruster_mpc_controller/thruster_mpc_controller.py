import rclpy
from rclpy.node import Node
from scipy.io import loadmat
from nav_msgs.msg import Odometry
import numpy as np
from std_msgs.msg import Float64
from scipy.optimize import minimize, LinearConstraint,Bounds

#添加画动态图的库
import matplotlib.pyplot as plt
import math
import matplotlib.animation as animation



class MatFileLoaderNode(Node):
    def __init__(self):
        super().__init__('thruster_mpc_controller')
        self.load_mat_file()
        self.Fig_init()
        self.subscription = self.create_subscription(
            Odometry,
            '/model/equipped_tethys/odometry',
            self.odometry_callback,
            1
        )
        self.cmd_vel_publisher = self.create_publisher(Float64, '/tethys/propeller_cml', 1)
     
    def load_mat_file(self):
        try:
            mat_data = loadmat('/home/dingding/github_project/KoopmanMpcThruster/matlab_koopman/mpc_matrix.mat')  # 加载.mat文件
            # 从加载的数据中获取变量并进行进一步处理
            self.Alift = mat_data['Alift']
            self.Blift = mat_data['Blift']
            self.Clift = mat_data['Clift']
            self.Nrbf = mat_data['Nrbf']
            self.n = mat_data['n']
            self.m = mat_data['m']
            self.cent = mat_data['cent']
            self.rbf_type=mat_data['rbf_type']
            
            self.horizon_length=5
            self.x_ref=np.zeros((self.Alift.shape[0],1))
            self.x_ref[0,0]=-0.8
            self.tau_max=150.0
            self.delta_tau_max=50.0
            self.u_init=np.zeros(self.horizon_length)
                        

            #Q=np.eye(2)
            self.Q=np.zeros((self.Alift.shape[0],self.Alift.shape[0]))
            self.Q[0,0]=3000.0
            self.R=0.01
            
            #self.state_bounds = [(-10, 10)]
            
            self.simulation_time=0
            self.deltaT=0.1
            self.x_values=[]
            self.u_values=[]
            self.ref_values=[]

            self.get_logger().info('Loaded variables from /home/dingding/MatlabSimul/koopman/KoopmanMPC/mpc_matrix.mat file')
            self.get_logger().info(f"Alift: {self.Alift}")
            self.get_logger().info(f"Blift: {self.Blift}")
            self.get_logger().info(f"Clift: {self.Clift}")
            self.get_logger().info(f"Nrbf: {self.Nrbf}")
            self.get_logger().info(f"n: {self.n}")
            self.get_logger().info(f"m: {self.m}")
            self.get_logger().info(f"cent: {self.cent}")
            self.get_logger().info(f"rbf_type: {self.rbf_type}")
        except Exception as e:
            self.get_logger().error('Error loading .mat file: {}'.format(e))
            
    def odometry_callback(self, msg):
        # Extract linear velocity information
        linear_velocity_x = msg.twist.twist.linear.x
        self.get_logger().info(f" linear_velocity_x: {linear_velocity_x}")
        #cent = np.random.rand(2, 3)  # random centers
        #rbf_type = 'gauss'  # Example RBF type
        lifted_linear_velocity_x = self.lift_function(linear_velocity_x, self.cent, self.rbf_type) 
        
        # Now you can use lifted_linear_velocity_x for further processing

        propeller_command_data = self.mpc_control(self.u_init,self.x_ref,lifted_linear_velocity_x,self.Q,self.R,self.horizon_length,self.tau_max,self.delta_tau_max)
       
        self.u_init=propeller_command_data
        cmd_vel_msg = Float64()
        cmd_vel_msg.data = propeller_command_data[0]  # 
        self.get_logger().info(f" input_value: {cmd_vel_msg.data}")
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        
        self.x_values.append(-linear_velocity_x)
        self.u_values.append(cmd_vel_msg.data)
        self.ref_values.append(-self.x_ref[0])
        
        self.simulation_time+=self.deltaT
        if self.simulation_time>=10:
           self.x_ref[0,0]=-1.6 
        
        self.update_plot(self.x_values,self.u_values,self.ref_values)
        plt.pause(0.001)  # 添加延迟以更新绘图
    
    def system_dynamics(self, x, u):
        # Define system dynamics
        A = self.Alift
        B = self.Blift
        return np.dot(A, x) + np.dot(B, u)
    
    def objective_function(self, u, x_ref, x_init, Q, R, horizon_length):
        # Define objective function for MPC
        cost = 0.0
        x_current=x_init
        for i in range(horizon_length):
            cost += np.dot((x_current - x_ref).T, np.dot(Q, (x_current - x_ref))) + np.dot(u[i].T, np.dot(R, u[i]))
            x_current = self.system_dynamics(x_current, u[i])
        return cost
    
    def mpc_control(self, u_init, x_ref,  x_init, Q, R, horizon_length, tau_max, delta_tau_max):
        # MPC controller
        # Linear constraints on the rate of change of tau: -delta_tau_max <= tau[i+1] - tau[i] <= delta_tau_max for all i in the N - 1
        # Implemented using LinearConstraint as -delta_tau_max <= delta_tau_matrix * tau <= delta_tau_max
        delta_tau_matrix = np.eye(horizon_length) - np.eye(horizon_length, k=1)
        constraint1 = LinearConstraint(delta_tau_matrix, -delta_tau_max, delta_tau_max)
        
        # We need a constraint on the rate of change of tau[0] respect to its previous value, which is tau_ini[0]
        first_element_matrix = np.zeros([horizon_length, horizon_length])
        first_element_matrix[0, 0] = 1
        constraint2 = LinearConstraint(first_element_matrix, u_init[0]-delta_tau_max, u_init[0]+delta_tau_max)
        
        ##添加这段代码用来测试状态受限的MPC
        #C_state = np.eye(horizon_length)  # Identity matrix for simplicity, adjust as needed
        #d_state_max = np.array([bound[1] for bound in state_bounds])  # Upper bound vector
        #d_state_min = np.array([bound[0] for bound in state_bounds])  # Lower bound vector
        #constraint3 = LinearConstraint(C_state, d_state_max, d_state_min)

        # Add constraints
        #delta_tau_constraint = [constraint1, constraint2, constraint3]


        # Add constraints
        delta_tau_constraint = [constraint1, constraint2]

        # Bounds --> -tau_max <= tau[idx] <= tau_max for idx = 0 to N-1
        bounds = [(-tau_max, tau_max) for idx in range(horizon_length)]

        # Starting optimisation point for theta and dtheta are the current measurements
        
        # Minimization
        result = minimize(self.objective_function, u_init, args=(x_ref, x_init, Q, R, horizon_length), bounds=bounds, constraints=delta_tau_constraint)

        return result.x
    
    def lift_function(self,X, C, rbf_type, eps=1.0, k=1.0):
        X= np.atleast_2d(X)  
        #rbf_type = rbf_type.lower()
        # Handle optional parameters
        if not np.all(np.isfinite(eps)):
            raise ValueError("eps must be finite")
        if not np.all(np.isfinite(k)):
            raise ValueError("k must be finite")

        C_big = C.copy()  # Avoid modifying the original C array创建副本

        Y = np.zeros((C.shape[1], X.shape[1])) #中心C 的 列向量 ，X 的列向量

        for i in range(C_big.shape[1]):
            center = C_big[:, i].reshape(-1, 1)
            center = np.repeat(center, X.shape[1], axis=1)  # Equivalent to repmat
            squared_distances = np.sum((X - center) ** 2, axis=0)  # Efficient distance calculation
   
            if rbf_type == 'thinplate':
                y = squared_distances * np.log(np.sqrt(squared_distances))
                y[np.isnan(y)] = 0  # Handle potential NaNs
            elif rbf_type == 'gauss':
                y = np.exp(-eps**2 * squared_distances)
            elif rbf_type == 'invquad':
                y = 1.0 / (1.0 + eps**2 * squared_distances)
            elif rbf_type == 'invmultquad':
                y = 1.0 / np.sqrt(1.0 + eps**2 * squared_distances)
            elif rbf_type == 'polyharmonic':
                y = squared_distances**(k / 2.0) * np.log(np.sqrt(squared_distances))
                y[np.isnan(y)] = 0  # Handle potential NaNs
            else:
                raise ValueError("Unsupported RBF type: {}".format(rbf_type))
            Y[i, :] = y

        lifted_Y= np.vstack((X, Y))
        return lifted_Y  
    def Fig_init(self):
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 6))
        self.line1, = self.ax1.plot([], [], label='vehicle velicity', marker='o')
        self.line2, = self.ax2.plot([], [], label='propeller RPM', marker='o')
        self.tracking_line, = self.ax1.plot([], [], linestyle='--', color='r', label='reference_velicity')

        
    def update_plot(self,x_values,u_values,ref_values):
    
        self.ax1.set_xlim(0, self.simulation_time)
        self.ax1.set_ylim(-2.5, 2.5)
        self.ax1.set_xlabel('time')
        self.ax1.set_ylabel('vehicle velicity')
        self.ax1.legend()

        self.ax2.set_xlim(0, self.simulation_time)
        self.ax2.set_ylim(-self.tau_max-10, self.tau_max+10)
        self.ax2.set_xlabel('time')
        self.ax2.set_ylabel('vehicle velicity')
        self.ax2.legend()
        
        self.line1.set_data(np.arange(len(x_values)) * self.deltaT, self.x_values)
        self.line2.set_data(np.arange(len(u_values)) * self.deltaT, self.u_values)
        self.tracking_line.set_data(np.arange(len(self.ref_values))* self.deltaT, self.ref_values)  # 跟踪信号的位置为x_ref[0]
        return self.line1, self.line2,self.tracking_line
      
def main(args=None):
    rclpy.init(args=args)
    mat_file_loader_node = MatFileLoaderNode()
   
    rclpy.spin(mat_file_loader_node)
    mat_file_loader_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




该文件夹测试与Gazebo结合的测试程序，目的是使用数据驱动的MPC方法控制GAZEBO中的水下无人航行器

在这个工作空间下，目前包含两个功能包ros2gzbridge 和 thruster_mpc_controller ，分别使用 C++ 和 python 写成

功能包 一 、ros2bridge 功能包下面包含3个可行性程序
  1. odometry_subscriber 实现gazebo 和ros2通信的基本框架，用到了ros_gz_bridge功能包，该功能包应该是在～/ws或者 ～/rosgz_ws下面，
  2. data_collect_thruster 实现 搜集 对螺旋桨速度控制实现时一些变量的数据
  3. Thruster_mpc 本来打算使用C++实现Thruster的mpc控制，但为了方便先决定使用python
功能包 二、thruster_mpc_controller 功能包主要用来测试使用 koopman mpc控制 Thruster,具体内容看里面的README
 

####启动文件

rosgzbridge_launch.py 文件主要用来测试Gazebo与 ros2的数据交互，

Thruster_collect_launch.py文件用来从Gazebo中获得 无人航行器的速度、位置和输入的螺旋桨的速度，将数据存放在odom_data_thruster.csv文件中

Thruster_Koopman_mpc.py 该启动文件是打开功能包二 在~/launch_ws目录中运行 ros2 launch Thruster_Koopman_mpc.py 
##### 20240425
今天将koopman 算法的mpc 集成到了一起，现在可以完成基于koopman的水下无人艇的仿真实验了，但是效果很不理想，
目前还没有对控制u进行限制，状态也没有限制，还没有发挥mpc的优势
还需要进一步的改进

#######
半路出家就是难，在python中实现mpc控制还没有成功
之前使用的是scipy.optimize.minimize 但是这个库应该不如 casadi好用，果然半路出家太难了。
根本不知道错误在那。还是要回到开头，重新组织，实现mpc控制的路子

###

这里面有两个功能包，ros2gzbridge是用C++ 写成，包含两个可执行的程序
thruster_mpc_controller主要使用python实现mpc控制，python实现mpc相对更简单。

################
还是尝试使用C++ 完成mpc的控制，因为C++ 也可以完成matlab得到的矩阵

今天晚上的工作，是将matlab 产生的矩阵传递到C++ 中，下一步是如何使用这些矩阵从而实现 mpc的控制任务c++方案在ros2gzbridge的功能包中

###############
在功能包中添加了mpc_python_test.py用来学习mpc的控制，可以实现控制问题

在功能包中添加了 lifgt_fun.py代码，用来测试 lift_fun的功能，现在还不能实现想要的目标

在ros2 节点添加了lift的函数，可以将状态生维了，结下来就是把mpc算法结成到该节点中，感觉看到了希望！

在python中编写mpC代码遇到了困难，有非常多的优化算法，比如基于符号的casadi,scipy.optimize import minimize,还有其他许多方法。
最后选择的 minimize ，参考了https://github.com/simorxb/MPC-Pendulum-Python/blob/main/MPC.py。可以实现对控制输入信号 u 的范围限制和变化速度的限制。还可以实现动态显示跟踪路径。





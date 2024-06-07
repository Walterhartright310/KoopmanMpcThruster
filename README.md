# KoopmanMpcThruster
This project uses Koopman theory and mpc control method for underwater vehicle, 
there are four project in this directory. More details is found in each README.md file.

1.Gazebo_model: this project is used to buil the underwater vehicle model, the Gazebo version is harmonic 
2.koopman_mpc: To run this project, you must install ROS2 , the version I used is iron. this project include several executable function, 
   "rosgzbridge_launch.py" is test the ros_gz_bride to connet the ros2 and gazebo, "Thruster_collect_launch.py" is used to collect data from gazebo, the collected 
   data is used to koopman method to get the x_k+1= Ax_k+BU_k, Thruster_Koopman_mpc.py
3.matlab_koopman: this project uses matlab to generate koopman operator
4.ros_gz_bridge: bridge connect ros2 and gazebo


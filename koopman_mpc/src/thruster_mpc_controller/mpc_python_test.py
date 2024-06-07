import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.optimize import minimize, LinearConstraint
import matplotlib.animation as animation

# Define system dynamics
A = np.array([[1.0, 0.1], [0.0, 1.0]])  # State transition matrix
B = np.array([[0.005], [0.1]])         # Input matrix


def system_dynamics(x, u):
    return np.dot(A, x) + np.dot(B, u)

# Define objective function for MPC
def mpc_cost(u, x_ref, x_init , Q, R, N):
    
    cost = 0.0
    x_current=x_init
    for i in range(N):
        #cost += np.dot((x_current - x_ref), np.dot(Q, (x_current - x_ref)))
        cost += np.dot((x_current - x_ref).T, np.dot(Q, (x_current - x_ref))) + np.dot(u[i].T, np.dot(R, u[i]))
        x_current = system_dynamics(x_current, u[i])
        
    return cost
    
# Solve MPC optimization problem
def solve_mpc(u_init, x_ref,  x_init, Q, R, N, tau_max, delta_tau_max):

    # Linear constraints on the rate of change of tau: -delta_tau_max <= tau[i+1] - tau[i] <= delta_tau_max for all i in the N - 1
    # Implemented using LinearConstraint as -delta_tau_max <= delta_tau_matrix * tau <= delta_tau_max
    delta_tau_matrix = np.eye(N) - np.eye(N, k=1)
    constraint1 = LinearConstraint(delta_tau_matrix, -delta_tau_max, delta_tau_max)

    # We need a constraint on the rate of change of tau[0] respect to its previous value, which is tau_ini[0]
    first_element_matrix = np.zeros([N, N])
    first_element_matrix[0, 0] = 1
    constraint2 = LinearConstraint(first_element_matrix, u_init[0]-delta_tau_max, u_init[0]+delta_tau_max)
    
    # Add constraints
    delta_tau_constraint = [constraint1, constraint2]

    # Bounds --> -tau_max <= tau[idx] <= tau_max for idx = 0 to N-1
    bounds = [(-tau_max, tau_max) for idx in range(N)]

    # Starting optimisation point for theta and dtheta are the current measurements
    
    # Minimization
    result = minimize(mpc_cost, u_init, args=(x_ref, x_init, Q, R, N), bounds=bounds, constraints=delta_tau_constraint)

    # Extract the optimal control sequence
    u_mpc = result.x

    return u_mpc    


 
# Simulation parameters
dt = 0.1
simulation_time = 10.0
num_steps = int(simulation_time / dt)
N = 20
tau_max=15
delta_tau_max=10

#Q=np.eye(2)
Q=np.array([[50.0, 0.0], [0.0,0.0]])
R=1
# Initial state and reference state
x_init = np.array([[0.0], [0.0]])  # Initial state

x_ref = np.array([[5.0], [0.0]])      # Reference state
u_init = np.zeros(N)
# Simulation loop
control_inputs = []
system_states = []
control_inputs.append(u_init[0])
x_current=x_init
system_states.append(x_current)


fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
line1, = ax1.plot([], [], label='Position', marker='o')
line2, = ax2.plot([], [], label='Control Input', marker='o')
tracking_line, = ax1.plot([], [], linestyle='--', color='r', label='Tracking Signal')

ax1.set_xlim(0, simulation_time)
ax1.set_ylim(-10, 15)
ax1.set_xlabel('Time')
ax1.set_ylabel('Position')
ax1.legend()

ax2.set_xlim(0, simulation_time)
ax2.set_ylim(-tau_max-1, tau_max+1)
ax2.set_xlabel('Time')
ax2.set_ylabel('Control Input')
ax2.legend()

# 初始化绘图结果
x_values = []
u_values = []
x_ref_values=[]
def update_plot():
    line1.set_data(np.arange(len(x_values)) * dt, x_values)
    line2.set_data(np.arange(len(u_values)) * dt, u_values)
    tracking_line.set_data(np.arange(len(x_values))* dt, x_ref_values)  # 跟踪信号的位置为x_ref[0]
    return line1, line2,tracking_line

for i in range(num_steps):
    if i*dt < simulation_time/2:
       x_ref = np.array([[5.0], [0.0]]) 
       
    else:
       x_ref = np.array([[10.0], [0.0]]) 
    u_init = np.zeros(N)
    u = solve_mpc(u_init, x_ref,x_current, Q,R,N,tau_max, delta_tau_max)
    x_current = system_dynamics(x_current, u[0])
    control_inputs.append(u[0])
    system_states.append(x_current)
    #u_init=u
    
    x_ref_values.append(x_ref[0])
    x_values.append(x_current[0])
    u_values.append(u[0])

    # 实时更新绘图结果
    update_plot()
    plt.pause(0.0005)  # 添加延迟以更新绘图

plt.show()

system_states = np.array(system_states)
control_inputs = np.array(control_inputs)



"""
# Plot results
time = np.arange(0, simulation_time, dt)
plt.figure(figsize=(10, 6))
plt.subplot(2, 1, 1)
plt.plot(time, system_states[:-1, 0], label='Position')
plt.plot(time, system_states[:-1, 1], label='Velocity')
plt.title('System States')
plt.xlabel('Time')
plt.ylabel('States')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(time[:num_steps], control_inputs[:-1], label='Control Input')
plt.title('Control Input')
plt.xlabel('Time')
plt.ylabel('Input')
plt.legend()

plt.tight_layout()
plt.show()
"""

  
  


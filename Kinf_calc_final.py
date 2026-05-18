import numpy as np
from scipy import linalg, signal
import matplotlib.pyplot as plt
from numpy.linalg import matrix_rank

# System Parameters
Ix = 16.571710e-06  # kg * m^2
Iy = 16.655602e-06  # kg * m^2
Iz = 29.261652e-06 # kg * m^2
m = 0.0313  # mass in kilograms
g = 9.81
dt = 0.002 # 500 Hz

A = np.array([[0,0,0,0,0,0, 1,0,0,0,0,0],
              [0,0,0,0,0,0, 0,1,0,0,0,0],
              [0,0,0,0,0,0, 0,0,1,0,0,0],
              [0,0,0,0,0,0, 0,0,0,1,0,0],
              [0,0,0,0,0,0, 0,0,0,0,1,0],
              [0,0,0,0,0,0, 0,0,0,0,0,1],
              
              [0,0,0,0,g,0,  0,0,0,0,0,0],
              [0,0,0,0,0,-g, 0,0,0,0,0,0],
              [0,0,0,0,0,0,       0,0,0,0,0,0],
              [0,0,0,0,0,0,       0,0,0,0,0,0],
              [0,0,0,0,0,0,       0,0,0,0,0,0],
              [0,0,0,0,0,0,       0,0,0,0,0,0]])

B = np.array([[0,0,0,0],
               [0,0,0,0],
               [0,0,0,0],
               [0,0,0,0],
               [0,0,0,0],
               [0,0,0,0],
               [0,0,0,0],
               [0,0,0,0],
               [(1/m),0,0,0],
               [0,(1/Ix),0,0],
               [0,0,(1/Iy),0],
               [0,0,0,(1/Iz)]])

C = np.array([[1,0,0,0,0,0, 0,0,0,0,0,0],
                [0,1,0,0,0,0, 0,0,0,0,0,0],
                [0,0,1,0,0,0, 0,0,0,0,0,0],
                [0,0,0,0,0,1, 0,0,0,0,0,0]])
D = np.zeros((4,4))

# print(np.shape(A))
# print(np.shape(B))
# print(np.shape(C))
# print(np.shape(D))

# Convert system to discrete
system = signal.StateSpace(A,B,C,D)
sys_discrete = system.to_discrete(dt, method = "foh")
A_d = sys_discrete.A
B_d = sys_discrete.B

# check if stable; if not, push eigenvlaues of A matrix to fully controllable
controllability_matrix = np.hstack([np.linalg.matrix_power(A_d, i) @ B_d for i in range(A_d.shape[0])])
if matrix_rank(controllability_matrix) < A_d.shape[0]:
    print("System is not controllable!")
    print("The system has unstable modes that are not controllable.")
    print("Shifting the eigenvalues by a small perturbation...")
    epsilon = 0.03
    A_d = A_d - epsilon * np.eye(A.shape[0])
else:
    print("System is controllable.")
print("Eigenvalues of A:", np.linalg.eigvals(A_d))


# 12:31: drone keeps flipping in the - pitch direction. 
    # Q value for pitch changed from 10 to 100

# 12:48: drone flips in the roll, some in yaw direction
    # R value for roll canged from 1e6 to 1e7, R value for yaw changed from 1e6 to 2e6

# 12:59: drone flips in the roll and yaw direction 
    # Q value for roll (10--> 100), pitch(100--> 500)


# 1:08: drone still flips, needs more thrust + roll and pitch stabilization
    # Q values for roll: 100-->1000, for pitch: 500 --> 1000
    # R values for total thrust: 100-->1000

# 1:18: drone works WAAYYYYY better! Got off the ground more, more stable. Still some roll tho, source of instability
    # Q values for roll: 1000--> 2500, for pitch: 1000--> 2500, for yaw: 200

# 1:37: the way I had it before was better, changing the values back but increasing the pitch 
    # Q values for roll: 2500-->1000, for pitch: 2500-->1250, for yaw: 200-->100

# 1:48: roll needs more tuning, achieves target height on some of them, but might need to increase the thrust 
    # Q valeus for roll: 1000-->2000
    # R values for thrust: 1000-->1250

# 1:53: too much yaw, too much pitch 
    # Q values for pitch: 1250-->1000
    # Q values for yaw: 100-->1000


# 2:08: reduce yaw, more roll and pitch control, a *little* less thrust
    # Q values for yaw: 1000-->250
    # Q values for pitch: 1000-->1500
    # Q values for roll: 2000--> 1500
    # R values for thrust: 1250-->1000

# 2:14: works great, some issues with roll; reduce thrust a little bit
    # Q values for roll: 1500-->1750
    # R values for thrust: 1000-->875

# 2:33: changes to custom_controller.c for the yaw angle stuff + fixed flow deck connnections

# 2:40: yaw angle stuff doesn't works






# Higher Q, Lower R
# reset stabilizer 2-->1-->2 each time
# make sure that the drone is always oriented in the +x away from you to tune Q and R matrices
# R = diag([1e3 1e7 1e7 1e7]);
# Q = diag([1e1 1e1 1e4 ...
#           1e2 1e2 1e3 ...
#           1e1 1e1 1e2 ...
#           1e1 1e1 1e1]);
# TUNING FOR Q and R MATRICES
# Q = np.eye(12) * np.array([1, 1, 1000, 1750, 1500, 250, 1, 1, 10, 1, 1, 1])
# R = np.eye(4) * np.array([875, 1e6, 1e6, 1e6])
Q = np.eye(12) * np.array([1, 1, 1000, 10, 10, 250, 1, 1, 10, 1, 1, 1])
R = np.eye(4) * np.array([125, 1e6, 1e6, 1e6])


P = linalg.solve_discrete_are(A_d, B_d, Q, R)
K = np.linalg.inv(B_d.T @ P @ B_d + R) @ (B_d.T @ P @ A_d)

# print K matrix in C-parsable code
K_c_array = "{\n" + ",\n".join(
    "    {" + ", ".join(f"{val:.6f}" for val in row) + "}" for row in K
) + "\n};"

print("static float K_dlqr_0[NUM_CTRL][NUM_STATE] = ", K_c_array)


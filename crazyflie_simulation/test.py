import numpy as np
import scipy.linalg as scl

ixx = 16.571710E-06  # kg * m^2
iyy = 16.655602E-06  # kg * m^2
izz = 29.261652E-06  # kg * m^2
m = 0.028 # mass in kilograms
g = 9.81

# A = np.array([ #unflipped
#     #/xyz       #/xyz_dot      #/rpy      #/rpy_dot

#     [0, 0, 0,   1, 0, 0,    0, 0, 0,    0, 0, 0], #xdot
#     [0, 0, 0,   0, 1, 0,    0, 0, 0,    0, 0, 0], #ydot
#     [0, 0, 0,   0, 0, 1,    0, 0, 0,    0, 0, 0], #zdot
    
#     [0, 0, 0,   0, 0, 0,    0,-g, 0,    0, 0, 0],  #xdotdot
#     [0, 0, 0,   0, 0, 0,    g, 0, 0,    0, 0, 0],  #ydotdot
#     [0, 0, 0,   0, 0, 0,    0, 0, 0,    0, 0, 0],  #zdotdot
    
#     [0, 0, 0,   0, 0, 0,    0, 0, 0,    1, 0, 0], #rdot
#     [0, 0, 0,   0, 0, 0,    0, 0, 0,    0, 1, 0], #pdot
#     [0, 0, 0,   0, 0, 0,    0, 0, 0,    0, 0, 1], #ydot
    
#     [0, 0, 0,   0, 0, 0,    0, 0, 0,    0, 0, 0],#roll_dotdot
#     [0, 0, 0,   0, 0, 0,    0, 0, 0,    0, 0, 0],#pitch_dotdot
#     [0, 0, 0,   0, 0, 0,    0, 0, 0,    0, 0, 0] #yaw_dotdot
# ])

# for this one, the state is: x y z roll pitch yaw xdot ydot zdot roll_dot pitch_dot yaw_dot

A = np.array([ #done: flipped state representation
    #/xyz        #/rpy     #/xyz_dot    #/rpy_dot
    [0, 0, 0,   0, 0, 0,    1, 0, 0,    0, 0, 0], #xdot
    [0, 0, 0,   0, 0, 0,    0, 1, 0,    0, 0, 0], #ydot
    [0, 0, 0,   0, 0, 0,    0, 0, 1,    0, 0, 0], #zdot
    
    [0, 0, 0,   0, 0, 0,    0, 0, 0,    1, 0, 0], #rdot
    [0, 0, 0,   0, 0, 0,    0, 0, 0,    0, 1, 0], #pdot
    [0, 0, 0,   0, 0, 0,    0, 0, 0,    0, 0, 1], #ydot
    
    [0, 0, 0,   0, -g, 0,    0, 0, 0,    0, 0, 0], #xdotdot
    [0, 0, 0,   g, 0, 0,    0, 0, 0,    0, 0, 0], #ydotdot
    [0, 0, 0,   0, 0, 0,    0, 0, 0,    0, 0, 0], #zdotdot
    
    [0, 0, 0,   0, 0, 0,    0, 0, 0,    0, 0, 0], #roll_dotdot
    [0, 0, 0,   0, 0, 0,    0, 0, 0,    0, 0, 0], #pitch_dotdot
    [0, 0, 0,   0, 0, 0,    0, 0, 0,    0, 0, 0]  #yaw_dotdot
])

B = np.array([ #unflipped state representation 
    [0, 0, 0, 0], #x
    [0, 0, 0, 0], #y
    [0, 0, 0, 0], #z
    [0, 0, 0, 0],   # roll
    [0, 0, 0, 0],   # pitch
    [0, 0, 0, 0],   # yaw
    [0, 0, 0, 0], #xdot
    [1/m, 0, 0, 0], #ydot
    [0, 0, 0, 0],   #zdot
    [0, 1/ixx, 0, 0],  #roll_dot
    [0, 0, 1/iyy, 0],  #pitch_dot
    [0, 0, 0, 1/izz]   #yaw_dot
])

# Define the system matrices
Q = 4*np.eye(12)
R = 0.03*np.eye(4)   

S = scl.solve_discrete_are(A, B, Q, R)

# Compute the optimal feedback gain
K = np.linalg.inv(R) @ B.T @ S

print("Optimal feedback gain K:", K)
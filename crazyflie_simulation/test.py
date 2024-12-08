import numpy as np
from scipy import linalg, signal
import matplotlib.pyplot as plt
from numpy.linalg import matrix_rank

# System Parameters
Ix = 2.3951e-5  # kg * m^2
Iy = 2.3951e-5  # kg * m^2
Iz = 3.2347e-5  # kg * m^2
m = 0.0313  # mass in kilograms
g = 9.81
dt = 0.002 # 500 Hz
# dt = 0.001

A = np.array([[0,0,0,0,0,0,   1,0,0,0,0,0],
                [0,0,0,0,0,0, 0,1,0,0,0,0],
                [0,0,0,0,0,0, 0,0,1,0,0,0],
                [0,0,0,0,0,0, 0,0,0,1,0,0],
                [0,0,0,0,0,0, 0,0,0,0,1,0],
                [0,0,0,0,0,0, 0,0,0,0,0,1],
                
                [0,0,0,0,g,0,  0,0,0,0,0,0],
                [0,0, 0,-g,0,0, 0,0,0,0,0,0],
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

# Convert system to discrete
system = signal.StateSpace(A,B,C,D)
sys_discrete = system.to_discrete(dt, method = "zoh")
A_d = sys_discrete.A
B_d = sys_discrete.B

# controllability_matrix = np.hstack([np.linalg.matrix_power(A_d, i) @ B_d for i in range(A_d.shape[0])])
# if matrix_rank(controllability_matrix) < A_d.shape[0]:
#     print("System is not controllable!")
# else:
#     print("System is controllable.")

# print("Eigenvalues of A:", np.linalg.eigvals(A_d))

# Calculating Q and R matrices according to Bryson's rule
max_thrust = 0.6 # max thrust
max_pos = 15.0 # maximum position allowed in workspace
max_ang = 0.2 * 3.14 # linearization angles
max_vel = 6.0 # max linear velocity of drones
max_rate = 0.015 * 3.14 # max rotational rate of drone
max_eyI = Iz # max moment of inertia
max_states = np.array([
    max_pos, max_pos, max_pos,   # x, y, z positions
    max_ang, max_ang, max_ang,   # roll, pitch, yaw angles
    max_vel, max_vel, max_vel,   # x_dot, y_dot, z_dot velocities
    max_rate, max_rate, max_rate # roll_rate, pitch_rate, yaw_rate
])

max_inputs = np.array([max_thrust, 1e-3, 1e-3, 1e-3])
Q = 1e6*np.diag(1/max_states**2)
print(Q)
#            x, y, z,    r,   p,  y, xd, yd, zd, rd, pd, yd
multiplier = np.diag([1, 1, 5, 1e-4, 1e-4, 1, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4])
Q = Q @ multiplier
R = 1e3*np.diag(1/max_inputs**2)

# Q = np.eye(12)* 0.1
# R = np.eye(4)

# print(Q.shape)
# print(R.shape)

# Initial conditions and simulation parameters
# x0 = np.zeros(A.shape[0])  # Initial state
# x_ref = np.zeros(A.shape[0])  # Desired reference state

x0 = np.random.rand(12)  # Example initial state
x_ref = np.zeros(12)     # Desired state to converge to
x_ref[2] = 0.1; # desired height for hover

P = linalg.solve_discrete_are(A_d, B_d, Q, R)
K = np.linalg.inv(B_d.T @ P @ B_d + R) @ (B_d.T @ P @ A_d)

print("LQR Gain Matrix K:\n", K)


timesteps = 100
tolerance = 1e-3
max_iterations = 100


def iterative_tuning(A, B, x0, x_ref, Q_init, R_init, max_iterations, timesteps, dt, tolerance):
    """
    Iteratively tunes the LQR controller to minimize the RMS error between system states and the desired reference values.

    Parameters:
    A (ndarray): State transition matrix.
    B (ndarray): Input matrix.
    x0 (ndarray): Initial state vector.
    x_ref (ndarray): Desired reference state vector.
    Q_init (ndarray): Initial Q matrix.
    R_init (ndarray): Initial R matrix.
    max_iterations (int): Maximum number of iterations for tuning.
    timesteps (int): Number of timesteps for simulation.
    dt (float): Time step for simulation.
    tolerance (float): Stopping criterion for RMS error.

    Returns:
    K (ndarray): Final LQR gain matrix.
    Q (ndarray): Tuned Q matrix.
    R (ndarray): Tuned R matrix.
    states (ndarray): Final states after tuning.
    inputs (ndarray): Final inputs after tuning.
    """
    Q = np.copy(Q_init)
    R = np.copy(R_init)
    best_rms_error = float("inf")
    best_K = None
    best_states = None
    best_inputs = None

    for iteration in range(max_iterations):
        # Solve the continuous algebraic Riccati equation
        P = linalg.solve_continuous_are(A, B, Q, R)
        K = np.linalg.inv(R) @ B.T @ P  # Compute LQR gain

        # Simulate the system
        x = np.copy(x0)
        states = []
        inputs = []
        for _ in range(timesteps):
            u = -K @ (x - x_ref)  # Compute control input
            x = A @ x + B @ u  # Update state
            states.append(x)
            inputs.append(u)

        states = np.array(states)
        inputs = np.array(inputs)

        # Calculate RMS error
        rms_error = np.sqrt(np.mean((states - x_ref)**2))

        print(f"Iteration {iteration}, RMS Error: {rms_error}")

        # Check for convergence
        if rms_error < tolerance:
            print("Converged to desired tolerance.")
            best_K = K
            best_states = states
            best_inputs = inputs
            break

        # Adjust Q and R matrices to reduce error
        if rms_error < best_rms_error:
            best_rms_error = rms_error
            best_K = K
            best_states = states
            best_inputs = inputs

        # Example tuning heuristic: Increase penalties on states with higher errors
        Q += np.diag(np.mean(np.abs(states - x_ref), axis=0)) * 0.1
        R += np.eye(R.shape[0]) * 0.01  # Slightly increase input penalties

    return best_K, Q, R, best_states, best_inputs

# Q_init = np.copy(Q)
# R_init = np.copy(R)
# K, Q_tuned, R_tuned, states, inputs = iterative_tuning(A, B, x0, x_ref, Q_init, R_init, max_iterations, timesteps, dt, tolerance)

# print("Final LQR Gain Matrix K:\n", K)
# print("Tuned Q Matrix:\n", Q_tuned)
# print("Tuned R Matrix:\n", R_tuned)
import matplotlib.pyplot as plt

# Initialize lists to store velocities
motor1_vel = []
motor2_vel = []
motor3_vel = []
motor4_vel = []

# Read the motor_vel.txt file
with open('motor_vel.txt', 'r') as file:
    lines = file.readlines()
    for i in range(len(lines)):
        if lines[i].strip() == 'velocity:':
            motor1_vel.append(float(lines[i+1].strip().strip('- ')))
            motor2_vel.append(float(lines[i+2].strip().strip('- ')))
            motor3_vel.append(float(lines[i+3].strip().strip('- ')))
            motor4_vel.append(float(lines[i+4].strip().strip('- ')))

# Define start and end timesteps
tstart = 0  # Replace with your desired start timestep
tend = 9600    # Replace with your desired end timestep max 12300 or sth

# Ensure tstart and tend are within the valid range
tstart = max(0, tstart)
tend = min(len(motor1_vel), tend)

# Create a list of timesteps
timesteps = list(range(tstart, tend))

# Plot the velocities between tstart and tend
plt.figure(figsize=(10, 6))
plt.plot(timesteps, motor1_vel[tstart:tend], label='Motor 1')
plt.plot(timesteps, motor2_vel[tstart:tend], label='Motor 2')
plt.plot(timesteps, motor3_vel[tstart:tend], label='Motor 3')
plt.plot(timesteps, motor4_vel[tstart:tend], label='Motor 4')

# Add labels and legend
plt.xlabel('Timestep')
plt.ylabel('Velocity')
plt.title('Motor Velocities Over Time')
plt.legend()

# Show the plot
plt.show()
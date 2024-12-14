import matplotlib.pyplot as plt

# Initialize lists to store errors
z_error = []
roll_error = []
pitch_error = []
yaw_error = []

# Read the error.txt file
with open('error.txt', 'r') as file:
    lines = file.readlines()
    for i in range(len(lines)):
        if lines[i].strip() == 'data:':
            z_error.append(float(lines[i+1].strip().strip('- ')))
            roll_error.append(float(lines[i+2].strip().strip('- ')))
            pitch_error.append(float(lines[i+3].strip().strip('- ')))
            yaw_error.append(float(lines[i+4].strip().strip('- ')))

# Create a list of timesteps
timesteps = list(range(len(z_error)))

# Plot the errors
plt.figure(figsize=(10, 6))
plt.plot(timesteps, z_error, label='z error')
plt.plot(timesteps, roll_error, label='roll error')
plt.plot(timesteps, pitch_error, label='pitch error')
plt.plot(timesteps, yaw_error, label='yaw error')

# Add labels and legend
plt.xlabel('Timestep')
plt.ylabel('Error')
plt.title('Errors Over Time')
plt.legend()

# Show the plot
plt.show()
import numpy as np
import matplotlib.pyplot as plt

rng = np.random.default_rng(1)

#0.1 second steps, 200 times
deltat = 0.1 

num_steps = 200

starting_x, starting_y = 0.0, 10.0

velocity_x, velocity_y = 5.0,2.0


#The range and bearing error
initial_range = 0.3
initial_bearing = 0.03

#Ground truth so we know what's actually going on
timestamps = np.arange(0, num_steps * deltat, deltat)
real_positions = []
radar_measures = []


for t in timestamps:
    #modeled with constant velocity, gonna keep appending stuff to real_positions n fill brackets
    real_x = starting_x + velocity_x * t
    real_y = starting_y + velocity_y * t
    real_positions.append((real_x, real_y))

    #polar coords
    real_range = np.sqrt(real_x**2 + real_y**2)
    real_bearing = np.arctan2(real_y, real_x)
    
    #Noise on range and bearing calcs for measures append
    noise_r = rng.normal(0, initial_range)
    noise_b = rng.normal(0, initial_bearing)

    noisy_range = real_range + noise_r
    noisy_bearing = real_bearing + noise_b

    radar_measures.append((noisy_range, noisy_bearing))

#Turn values into arrays with numpy

real_positions = np.array(real_positions)
radar_measures = np.array(radar_measures)

output_filename = "sensor_data.txt"
with open(output_filename, "w") as f:
    for i in range(num_steps):
        f.write(f"{timestamps[i]:.4f}"
                f"{real_positions[i, 0]: .4f} {real_positions[i,1]: .4f}"
                f"{radar_measures[i, 0]: .4f} {radar_measures[i,1]: .4f}\n")
print(f"Good job bro we created {num_steps} data points and put them in '{output_filename}")

#Sim Size
plt.figure(figsize=(10,6))

#Convert polar back to cartesian coords

measure_x = radar_measures[:,0] * np.cos(radar_measures[:, 1])
measure_y = radar_measures[:,0] * np.sin(radar_measures[:, 1])

plt.plot(real_positions[:, 0], real_positions[:,1], 'g-', label ='Ground Truth (True Path)', linewidth = 2)
# s is pixel size, alpha opacity, dot colors red
plt.scatter(measure_x, measure_y, color='red', alpha = 0.6, s = 15, label= 'Radar Noise Measures') 

plt.title("Tracking Simulation of Extended Kalman Filter (EKF) Data")
plt.xlabel("X Position (m)")
plt.ylabel("Y position (m)")
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show() 
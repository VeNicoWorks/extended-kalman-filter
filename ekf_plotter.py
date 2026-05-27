import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

#Loading in file with corrected estimates
ekf_data = pd.read_csv(
    "ekf_estimate.txt",
    sep=r"\s+",
    header = None,
    names = ["time", "position_x", "position_y", "velocity_x", "velocity_y"]
)

#Load in original file with noisy data points
measured_data = pd.read_csv(
    "sensor_data.txt",
    sep=r"\s+",
    header = None,
    names = ["time", "rho", "theta"]
)

#Convert measured data to cartesian
measured_x = measured_data["rho"] * np.cos(measured_data["theta"])
measured_y = measured_data["rho"] * np.sin(measured_data["theta"])

#Make the plot

plt.figure(figsize =(10,8))

plt.scatter(
    measured_x,
    measured_y,
    color = "red",
    alpha = 0.6,
    s = 15,
    label = "Noisy Measurements From Radar/Sensors"
)

#Plot of corrected data and formatting
plt.plot(
    ekf_data["position_x"],
    ekf_data["position_y"],
    color="blue",
    linewidth = 2,
    label = "EKF Corrected State Estimate"

)

plt.title("Extended Kalman Filter (EKF) Tracking Results", fontsize = 14)
plt.xlabel("X Position (m)", fontsize=12)
plt.ylabel("Y Position (m)", fontsize = 12)
plt.grid(True, linestyle="--", alpha=0.6)
plt.legend(loc = "best")
plt.axis("equal")

plt.savefig("ekf_comparison.png", dpi=300)
plt.show


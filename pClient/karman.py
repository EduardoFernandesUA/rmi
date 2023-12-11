import numpy as np
import matplotlib.pyplot as plt
import time

# Kalman filter class
class KalmanFilter:
    def __init__(self, initial_estimate, process_noise, measurement_noise):
        self.estimate = initial_estimate
        self.estimate_error = 1.0
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise

    def update(self, measurement):
        # Prediction step
        prediction = self.estimate
        prediction_error = self.estimate_error + self.process_noise

        # Update step
        kalman_gain = prediction_error / (prediction_error + self.measurement_noise)
        self.estimate = prediction + kalman_gain * (measurement - prediction)
        self.estimate_error = (1 - kalman_gain) * prediction_error

        return self.estimate

# True angle generation with noise
np.random.seed(42)  # for reproducibility
num_steps = 100
true_angle = np.zeros(num_steps)
measured_angle = np.zeros(num_steps)

for t in range(1, num_steps):
    true_angle[t] = true_angle[t-1] + np.random.normal(0, 0.1)  # Random noise with std dev of 0.1
    measured_angle[t] = true_angle[t] + np.random.normal(0, 0.01 * np.abs(true_angle[t]))

# Kalman filter parameters
initial_estimate = 0.0
process_noise = 0.01  # Process noise (you may need to adjust this value)
measurement_noise = 0.01  # Measurement noise (1% error)

# Create Kalman filter instance
kalman_filter = KalmanFilter(initial_estimate, process_noise, measurement_noise)

# Plot results in real-time
plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots()

for t in range(num_steps):
    # Apply Kalman filter to noisy measurement
    kalman_estimate = kalman_filter.update(measured_angle[t])

    # Plot results
    ax.clear()
    ax.plot(true_angle[:t+1], label='True Angle', linestyle='--')
    ax.plot(measured_angle[:t+1], label='Measured Angle with 1% Error', marker='o', linestyle='', alpha=0.7)
    ax.plot(range(t+1), [kalman_filter.update(measured_angle[i]) for i in range(t+1)], label='Kalman Filter Estimate', marker='o', linestyle='-', alpha=0.7)

    ax.set_title('Kalman Filter for Angle Estimation (Real-time)')
    ax.set_xlabel('Time Steps')
    ax.set_ylabel('Angle')
    ax.legend()
    plt.pause(0.1)  # Pause for a short time to create a real-time effect

plt.ioff()  # Turn off interactive mode after the loop
plt.show()

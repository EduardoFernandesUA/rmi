import numpy as np

class AngularDifferenceKalmanFilter:
    def __init__(self, initial_state, initial_variance, process_variance, measurement_variance):
        self.state = initial_state
        self.variance = initial_variance
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance

    def measurement_update(self, angular_measurement):
        # Calculate Kalman Gain
        kalman_gain = self.variance / (self.variance + self.measurement_variance)

        # Compute angular difference
        angular_difference = angular_measurement - self.state

        # Adjust angular difference to the range [-pi, pi)
        while angular_difference >= np.pi:
            angular_difference -= 2 * np.pi
        while angular_difference < -np.pi:
            angular_difference += 2 * np.pi

        # Update state estimate
        self.state = self.state + kalman_gain * angular_difference

        # Update state variance
        self.variance = (1 - kalman_gain) * self.variance

        # Output current state estimate and variance
        return self.state, self.variance

    def prediction(self, angular_velocity):
        # Predict next state based on the dynamic model (angular velocity)
        predicted_state = self.state + angular_velocity

        # Adjust predicted state to the range [-pi, pi)
        while predicted_state >= np.pi:
            predicted_state -= 2 * np.pi
        while predicted_state < -np.pi:
            predicted_state += 2 * np.pi

        # Predict next state variance
        self.variance = self.variance + self.process_variance

        # Output predicted state estimate and variance
        return predicted_state, self.variance

# Example usage:
# Initial parameters
initial_state = 0.0
initial_variance = 1.0
process_variance = 0.1
measurement_variance = 0.1

# Create AngularDifferenceKalmanFilter instance
angular_kalman_filter = AngularDifferenceKalmanFilter(initial_state, initial_variance, process_variance, measurement_variance)

# Simulate time steps
num_steps = 100
true_angular_velocity = np.linspace(0, 0.1, num_steps) + np.random.normal(0, 0.01, num_steps)  # Simulated true angular velocity with noise
angular_measurements = np.cumsum(true_angular_velocity) + np.random.normal(0, np.sqrt(measurement_variance), num_steps)  # Simulated angular measurements

# Lists to store filter outputs
filtered_angular_differences = []

# Kalman filter loop
for t in range(num_steps):
    # Step 1: Measurement Update
    angular_state_estimate, angular_state_variance = angular_kalman_filter.measurement_update(angular_measurements[t])

    # Save filter output
    filtered_angular_differences.append(angular_state_estimate)

    # Step 2: Prediction
    angular_velocity = 0.1  # Example angular velocity
    angular_kalman_filter.prediction(angular_velocity)

# Plot results
import matplotlib.pyplot as plt

plt.plot(true_angular_velocity, label='True Angular Velocity', linestyle='--')
plt.plot(angular_measurements, label='Angular Measurements', marker='o', linestyle='', alpha=0.7)
plt.plot(filtered_angular_differences, label='Filtered Angular Differences', marker='o', linestyle='-', alpha=0.7)
plt.title('Angular Difference Kalman Filter Example')
plt.xlabel('Time Steps')
plt.ylabel('Angular Values')
plt.legend()
plt.show()

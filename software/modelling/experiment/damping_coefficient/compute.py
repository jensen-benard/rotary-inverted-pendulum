"""
Calculating the damping coefficient of the pendulum using experimental data

Useful information to calculate the damping coefficient is located in useful-resources/website.md.
"""

import matplotlib.pyplot as plt
from scipy.signal import find_peaks
import numpy as np
import sympy as sp

SAMPLE_RATE = 1000

# Read the pendulum swing data (stored in degrees)
file_path = "software/modelling/experiment/damping_coefficient/pendulum_free_swing_experiment_data_01.txt"
with open(file_path) as f:
    data = [float(line.strip()) for line in f]

# Find the index of the peaks and their position in time
t = np.arange(0, len(data)) / SAMPLE_RATE
peak_indices = find_peaks(data)[0]
peak_times = t[peak_indices]

# Find the peak values from the indices
data_peaks = [data[i] for i in peak_indices]

# Create a sympy expression for the damping coefficient so that it can be solved automatically
damping_coefficient = sp.Symbol("DC", real=True)
natural_omega = sp.Symbol("OMEGA_0", real=True)
amplitude = sp.Symbol("A", real=True)

time_period = peak_times[1] - peak_times[0]
print("Time period between peaks: {:.3f} s".format(time_period))

damped_omega = 2 * 180 / time_period
print("Damped angular frequency: {:.3f} deg/s".format(damped_omega))

damped_omega_equation = sp.Eq(damped_omega, natural_omega * sp.sqrt(1 - damping_coefficient**2))
natural_omega_expression = sp.solve(damped_omega_equation, natural_omega)[0].simplify()
print(f"Natural angular frequency: {natural_omega_expression} deg/s")

damped_exponential_equation_1 = sp.Eq(data_peaks[0], amplitude * sp.exp(-damping_coefficient * natural_omega_expression * peak_times[0])).simplify()
damped_exponential_equation_2 = sp.Eq(data_peaks[1], amplitude * sp.exp(-damping_coefficient * natural_omega_expression * peak_times[1])).simplify()

damped_exponential_expression = sp.solve(damped_exponential_equation_1, amplitude)[0].simplify()
damped_exponential_equation_subbed_amplitude = damped_exponential_equation_2.subs(amplitude, damped_exponential_expression).simplify()

damping_coefficient_solved = float(sp.solve(damped_exponential_equation_subbed_amplitude, damping_coefficient, rational=False)[0])

print(f"Damping coefficient: {damping_coefficient_solved}")

natural_omega_val = natural_omega_expression.subs(damping_coefficient, damping_coefficient_solved).simplify()
print("Natural angular frequency: {:.3f} deg/s".format(natural_omega_val))

# Plot the data against the model derived from the estimated damping coefficient
A = data_peaks[0] / np.exp(-damping_coefficient_solved * float(natural_omega_val) * peak_times[0])

model = A * np.exp(-damping_coefficient_solved * float(natural_omega_val) * t)

plt.plot(t, model)
plt.plot(t, data)
plt.xlabel("Sample")
plt.ylabel("Value")
plt.show()



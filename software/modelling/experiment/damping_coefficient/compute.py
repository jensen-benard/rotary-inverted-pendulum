"""
Calculating the damping coefficient of the pendulum using experimental data

Useful information to calculate the damping coefficient is located in useful-resources/website.md.
"""

import os

import matplotlib.pyplot as plt
from scipy.signal import find_peaks
import numpy as np
import sympy as sp

SAMPLE_RATE = 1000

# Read the pendulum swing data (stored in degrees)
file_name_01 = "pendulum_free_swing_data_01.txt"
file_name_02 = "pendulum_free_swing_data_02.txt"
file_name_03 = "pendulum_free_swing_data_03.txt"

file_path_01 = os.path.join("software", "modelling", "experiment", "damping_coefficient", file_name_01)
file_path_02 = os.path.join("software", "modelling", "experiment", "damping_coefficient", file_name_02)
file_path_03 = os.path.join("software", "modelling", "experiment", "damping_coefficient", file_name_03)
# Read the pendulum swing data with header and column values
def read_data_with_header(file_path):
    with open(file_path) as f:
        lines = f.readlines()
        header = lines[0].strip().split(', ')
        column_values = lines[1].strip().split(', ')
        column_values = [float(value) for value in column_values]
        info = {header[i]: column_values[i] for i in range(len(header))}
        data = [float(line.strip()) for line in lines[2:]]
    return info, data

info_01, data_01 = read_data_with_header(file_path_01)
info_02, data_02 = read_data_with_header(file_path_02)
info_3, data_03 = read_data_with_header(file_path_03)


data = [data_01, data_02, data_03]
info = [info_01, info_02, info_3]

fig, ax = plt.subplots(len(data), 1)

amplitudes = []
damping_coefficients = []
natural_omegas = []
models = []

for index, data_set in enumerate(data):
    # Find the index of the peaks and their position in time
    t = np.arange(0, len(data_set)) / SAMPLE_RATE
    peak_indices = find_peaks(data_set)[0]
    peak_times = t[peak_indices]

    # Find the peak values from the indices
    data_peaks = [data_set[i] for i in peak_indices]

    # Create a sympy expression for the damping coefficient so that it can be solved automatically
    damping_coefficient = sp.Symbol("DC", real=True)
    natural_omega = sp.Symbol("OMEGA_0", real=True)
    amplitude = sp.Symbol("A", real=True)

    time_period = peak_times[1] - peak_times[0]
    print(f"Time period between peaks: {time_period:.3f} s")

    damped_omega = 2 * 180 / time_period
    print(f"Damped angular frequency: {damped_omega:.3f} deg/s")

    damped_omega_equation = sp.Eq(damped_omega, natural_omega * sp.sqrt(1 - damping_coefficient**2))
    natural_omega_expression = sp.solve(damped_omega_equation, natural_omega)[0].simplify()
    print(f"Natural angular frequency: {natural_omega_expression} deg/s")

    damped_exponential_equation_1 = sp.Eq(data_peaks[0], amplitude * sp.exp(-damping_coefficient * natural_omega_expression * peak_times[0])).simplify()
    damped_exponential_equation_2 = sp.Eq(data_peaks[1], amplitude * sp.exp(-damping_coefficient * natural_omega_expression * peak_times[1])).simplify()

    damped_exponential_expression = sp.solve(damped_exponential_equation_1, amplitude)[0].simplify()
    damped_exponential_equation_subbed_amplitude = damped_exponential_equation_2.subs(amplitude, damped_exponential_expression).simplify()

    damping_coefficient_solved = float(sp.solve(damped_exponential_equation_subbed_amplitude, damping_coefficient, rational=False)[0])
    damping_coefficients.append(damping_coefficient_solved)
    print(f"Damping coefficient: {damping_coefficient_solved}")

    natural_omega_val = natural_omega_expression.subs(damping_coefficient, damping_coefficient_solved).simplify()
    natural_omegas.append(natural_omega_val)
    print(f"Natural angular frequency: {natural_omega_val:.3f} deg/s")

    amplitudes.append(data_peaks[0] / np.exp(-damping_coefficient_solved * float(natural_omega_val) * peak_times[0]))

    models.append(amplitudes[index] * np.exp(-damping_coefficient_solved * float(natural_omega_val) * t))


# Plot the data against the model derived from the estimated damping coefficient
ave_damping_coefficient = sum(damping_coefficients) / len(damping_coefficients)
ave_natural_omega = sum(natural_omegas) / len(natural_omegas)
ave_amplitude = sum(amplitudes) / len(amplitudes)

print("PLOTTING")
print(f"Average damping coefficient: {ave_damping_coefficient}")
print(f"Average natural angular frequency: {ave_natural_omega:.3f} deg/s")
print(f"Average amplitude: {ave_amplitude:.3f}")

ave_model = ave_amplitude * np.exp(-ave_damping_coefficient * float(ave_natural_omega) * t)

for index, data_set in enumerate(data):
    ax[index].plot(t, ave_model, label="Average Model")
    ax[index].plot(t, models[index], label="Individual Model")
    ax[index].plot(t, data_set, label=f"Trigger Start Angle: {info[index]['trigger_start_deg']}")
    ax[index].set_title(f"Individual Estimated Damping Coefficient: {damping_coefficients[index]:.10f}")
    ax[index].set_xlabel("Time (s)")
    ax[index].set_ylabel("Pendulum Angle (deg)")
    ax[index].grid(True)
    ax[index].legend()

fig.suptitle(f"Average Estimated Damping Coefficient: {ave_damping_coefficient:.10f}", fontsize=14)
fig.tight_layout()
plt.show()
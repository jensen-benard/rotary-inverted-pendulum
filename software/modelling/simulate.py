"""
Simulate the models of the rotary inverted pendulum provided by model.py
"""

import os
import numpy as np
from scipy.integrate import odeint
from scipy import signal
import matplotlib.pyplot as plt
import model


def read_data_with_header(file_path):
    with open(file_path) as f:
        lines = f.readlines()
        header = lines[0].strip().split(', ')
        column_values = lines[1].strip().split(', ')
        column_values = [float(value) for value in column_values]
        info = {header[i]: column_values[i] for i in range(len(header))}
        data = [float(line.strip()) for line in lines[2:]]
    return info, data


def non_linear_dynamics_system(x, t, non_linear_dynamics_model, torque=0):
    theta_ddot_arm = non_linear_dynamics_model.theta_ddot_arm_lambdified(x[0], x[1], x[2], x[3], torque)
    theta_ddot_pendulum = non_linear_dynamics_model.theta_ddot_pendulum_lambdified(x[0], x[1], x[2], x[3], torque)

    dxdt = np.array([x[1], theta_ddot_arm, x[3], theta_ddot_pendulum])

    return dxdt


def simulate_non_linear_dynamics_model(t, y0, torque=0, experiment_comparison_file_path=None, set_y0_to_experiment_initial_conditions=False):
    fig, ax = plt.subplots(4, 1)

    if experiment_comparison_file_path is not None and os.path.exists(experiment_comparison_file_path):
        info, data = read_data_with_header(experiment_comparison_file_path)
        data = np.array(data)
        data_sample_rate = info["sample_rate"]
        data_t = np.arange(0, len(data)) / data_sample_rate
        angle_offset = 180 # when doing the experiment, the zero angle was set at the bottom of the pendulum swing but it should be at the top.
        
        # Finding the offset start
        first_minima = signal.find_peaks(-data)[0][0]
        first_maxima = signal.find_peaks(data)[0][0]
        data_true_start_index = min(first_minima, first_maxima)
        time_offset = data_true_start_index / data_sample_rate
        ax[2].plot(data_t - time_offset, data + angle_offset, label="experiment_data")

        if set_y0_to_experiment_initial_conditions:
            # Make the start of the simulation match the start of the experiment when the pendulum is actually let go.
            print("Setting y0 to experiment initial conditions (ignoring y0 argument).")
            data_true_start = (data[data_true_start_index] + angle_offset) / 180 * np.pi
            y0 = [0, 0, data_true_start, 0]

            fig.text(0.5, 0.04, f"Initial Conditions: {y0} (Same as experiment ICs)", ha='center', va='center')
        else:
            fig.text(0.5, 0.04, f"Initial Conditions: {y0}", ha='center', va='center')
        
        fig.text(0.5, 0.02, f"Comparison with experiment: {experiment_comparison_file_path}", ha='center', va='center', wrap=True)
        fig.suptitle("Non-Linear Dynamics Model Simulation Compared to Experiment", fontsize=14)

    elif experiment_comparison_file_path is not None:
        raise FileNotFoundError("File does not exist: {}".format(os.path.abspath(experiment_comparison_file_path)))
    else:
        fig.suptitle("Non-Linear Dynamics Model Simulation", fontsize=14)
        fig.text(0.5, 0.04, f"Initial Conditions: {y0}", ha='center', va='center')

    if y0 is None:
        raise TypeError("y0 cannot be None")

    non_linear_dynamics_model = model.load_non_linear_dynamics_model_lambdified()

    s = odeint(non_linear_dynamics_system, y0, t, args=(non_linear_dynamics_model, torque))
    
    s_deg = s / np.pi * 180 # convert from radians to degress

    ax[0].plot(t, s_deg[:, 0], label="theta_arm_model")
    ax[0].grid(True)
    ax[1].plot(t, s_deg[:, 1], label="theta_arm_dot_model")
    ax[1].grid(True)
    ax[2].plot(t, s_deg[:, 2], label="theta_pendulum_model")
    ax[2].grid(True)
    ax[3].plot(t, s_deg[:, 3], label="theta_pendulum_dot_model")
    ax[3].grid(True)
    
    ax[0].legend()
    ax[1].legend()
    ax[2].legend()
    ax[3].legend()

    ax[0].set_xlabel("Time (s)")
    ax[1].set_xlabel("Time (s)")
    ax[2].set_xlabel("Time (s)")
    ax[3].set_xlabel("Time (s)")

    ax[0].set_ylabel("theta_arm (deg)")
    ax[1].set_ylabel("theta_arm_dot (deg/s)")
    ax[2].set_ylabel("theta_pendulum (deg)")
    ax[3].set_ylabel("theta_pendulum_dot (deg/s)")
    
    fig.tight_layout()
    plt.show()


def simulate_closed_loop_linear_state_space_model(t, y0, control_method):
    closed_loop_linear_state_space_model = model.load_closed_loop_linear_state_space_model(control_method)

    sysAcl = signal.StateSpace(closed_loop_linear_state_space_model.A, 
                                closed_loop_linear_state_space_model.B, 
                                closed_loop_linear_state_space_model.C, 
                                closed_loop_linear_state_space_model.D)
    external_disturbance = 0

    T, yout, xout = signal.lsim(sysAcl, external_disturbance, t, y0)

    fig, ax = plt.subplots(4, 1)

    xout_deg = xout / np.pi * 180 # convert from radians to degress

    fig.text(0.5, 0.05, f"Initial Conditions: {y0}", ha='center', va='center')
    fig.text(0.5, 0.02, f"Control Method: {control_method}", ha='center', va='center', wrap=True)

    print(type(xout[:, 0]))
    ax[0].plot(T, xout_deg[:, 0], label="theta_arm_model")
    ax[0].grid(True)
    ax[1].plot(T, xout_deg[:, 1], label="theta_arm_dot_model")
    ax[1].grid(True)
    ax[2].plot(T, xout_deg[:, 2], label="theta_pendulum_model")
    ax[2].grid(True)
    ax[3].plot(T, xout_deg[:, 3], label="theta_pendulum_dot_model")
    ax[3].grid(True)

    ax[0].legend()
    ax[1].legend()
    ax[2].legend()
    ax[3].legend()

    ax[0].set_xlabel("Time (s)")
    ax[1].set_xlabel("Time (s)")
    ax[2].set_xlabel("Time (s)")
    ax[3].set_xlabel("Time (s)")

    ax[0].set_ylabel("theta_arm (deg)")
    ax[1].set_ylabel("theta_arm_dot (deg/s)")
    ax[2].set_ylabel("theta_pendulum (deg)")
    ax[3].set_ylabel("theta_pendulum_dot (deg/s)")

    fig.suptitle("Closed Loop Linear State Space Model Simulation", fontsize=14)
    fig.tight_layout()
    plt.show()


def run_lqr_sim():
    lqr = model.LinearQuadraticRegulator(Q = [[1, 0, 0, 0], 
                                        [0, 1, 0, 0], 
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]],
                                        R = 0.01)
    simulate_closed_loop_linear_state_space_model(t=np.arange(0, 5, 0.001), y0=[0.01, 0, 1, 0], control_method=lqr)


def run_non_linear_dynamics_sim():
    # Choose one of 3 experiment data files
    file_name_01 = "pendulum_free_swing_data_01.txt"
    file_name_02 = "pendulum_free_swing_data_02.txt"
    file_name_03 = "pendulum_free_swing_data_03.txt"
    experiment = os.path.join("software", "modelling", "experiment", "damping_coefficient", file_name_03)
    simulate_non_linear_dynamics_model(t=np.arange(0, 5, 0.001), y0=None, experiment_comparison_file_path=experiment, set_y0_to_experiment_initial_conditions=True)



if __name__ == "__main__":
    # Simulation options
    #run_non_linear_dynamics_sim()
    run_lqr_sim()
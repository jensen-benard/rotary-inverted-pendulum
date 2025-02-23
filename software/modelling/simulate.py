"""
Simulate the models of the rotary inverted pendulum provided by model.py
"""

import os
import numpy as np
from scipy.integrate import odeint
from scipy import signal
import matplotlib.pyplot as plt
import model



def non_linear_dynamics_system(x, t, non_linear_dynamics_model, torque=0):
    theta_ddot_arm = non_linear_dynamics_model.theta_ddot_arm_lambdified(x[0], x[1], x[2], x[3], torque)
    theta_ddot_pendulum = non_linear_dynamics_model.theta_ddot_pendulum_lambdified(x[0], x[1], x[2], x[3], torque)

    dxdt = np.array([x[1], theta_ddot_arm, x[3], theta_ddot_pendulum])

    return dxdt


def simulate_non_linear_dynamics_model(t, y0, torque=0, experiment_comparison_file_path=None):
    non_linear_dynamics_model = model.load_non_linear_dynamics_model_lambdified()

    s = odeint(non_linear_dynamics_system, y0, t, args=(non_linear_dynamics_model, torque))
    
    s_deg = s / np.pi * 180 # convert from radians to degress

    fig, ax = plt.subplots(4, 1)
    ax[0].plot(t, s_deg[:, 0], label="theta_arm_model")
    ax[0].grid(True)
    ax[1].plot(t, s_deg[:, 1], label="theta_arm_dot_model")
    ax[1].grid(True)
    ax[2].plot(t, s_deg[:, 2], label="theta_pendulum_model")
    ax[2].grid(True)
    ax[3].plot(t, s_deg[:, 3], label="theta_pendulum_dot_model")
    ax[3].grid(True)

    if os.path.exists(experiment_comparison_file_path):
        data = np.loadtxt(experiment_comparison_file_path)
        data_sample_rate = 1000
        data_t = np.arange(0, len(data)) / data_sample_rate
        angle_offset = 180 # when doing the experiment, the zero angle was set at the bottom of the pendulum swing but it should be at the top.
        time_offset = 342 / data_sample_rate # the experiment started 342 samples into the recording
        ax[2].plot(data_t - time_offset, data + angle_offset, label="experiment_data")
    elif experiment_comparison_file_path is not None:
        print("File does not exist: {}".format(experiment_comparison_file_path))
    
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

    plt.tight_layout()
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

    plt.tight_layout()
    plt.show()


def run_lqr_sim():
    lqr = model.LinearQuadraticRegulator(Q = [[1, 0, 0, 0], 
                                        [0, 1, 0, 0], 
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]],
                                        R = 0.01)
    simulate_closed_loop_linear_state_space_model(t=np.arange(0, 5, 0.001), y0=[0.01, 0, 1, 0], control_method=lqr)


def run_non_linear_dynamics_sim():
    experiment_01 = "software/modelling/experiment/damping_coefficient/pendulum_free_swing_experiment_data_01.txt"
    simulate_non_linear_dynamics_model(t=np.arange(0, 5, 0.001), y0=[0.1, 0, 1.125, 0], experiment_comparison_file_path=experiment_01)



if __name__ == "__main__":
    # Simulation options

    #run_non_linear_dynamics_sim()
    run_lqr_sim()
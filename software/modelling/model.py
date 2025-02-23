"""
This file derives the mathematical model of the rotary inverted pendulum.
Linearisation and control design are performed here.
This file is imported by `simulate.py`.
"""


import os
from dataclasses import dataclass
from enum import Enum
import sympy as sp
import numpy as np
import logging
import control

logger = logging.getLogger("log")
logger.addHandler(logging.StreamHandler())
logger.setLevel(logging.CRITICAL)

@dataclass
class NonLinearDynamicsModel:
    theta_arm: sp.Symbol
    theta_dot_arm: sp.Symbol
    theta_pendulum: sp.Symbol
    theta_dot_pendulum: sp.Symbol
    theta_ddot_arm: sp.Symbol
    theta_ddot_pendulum: sp.Symbol
    torque: sp.Symbol

@dataclass
class NonLinearDynamicsModelLambdified:
    theta_ddot_arm_lambdified: sp.lambdify
    theta_ddot_pendulum_lambdified: sp.lambdify

@dataclass
class LinearStateSpaceModel:
    A: np.ndarray
    B: np.ndarray
    C: np.ndarray
    D: np.ndarray


@dataclass
class LinearQuadraticRegulator:
    Q: np.ndarray
    R: float
    K: np.ndarray = None
    S: np.ndarray = None
    E: np.ndarray = None


def __get_non_linear_dynamics_model(debug=False):

    if debug:
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(logging.CRITICAL)

    logger.debug("In __get_non_linear_dynamics_model()")

    # Rig elements properties (meters and kilograms)
    mass_rod = 0.01225
    length_rod = 0.124
    radius_rod = 0.002
    mass_hinge = 0.00693
    hinge_center_offset = 0.002
    coupler_offset = 0.011
    motor_damping_factor = 100
    pendulum_damping_factor = 0.00046380052104552763 # Acquired from experiment/damping_coefficient (average)


    # motor datasheet properties
    inertia_motor = 0.0000062  # kg m^2
    motor_rig_mass_extra = 1.2


    # motor symbols
    torque = sp.Symbol("torque", real=True)

    # other
    time = sp.Symbol("time", real=True)
    G = 9.81 

    # arm properties
    mass_arm = mass_rod + mass_hinge/2 + motor_rig_mass_extra
    length_arm = coupler_offset + length_rod + hinge_center_offset
    rot_inertia_arm = inertia_motor + mass_arm * (radius_rod**2 / 4 + length_arm**2 / 12) + mass_arm * (length_arm / 2)**2  # using parallel axis theorem
    damping_coeff_arm = motor_damping_factor

    # arm symbols
    M_arm = sp.Symbol("M_arm", real=True)
    L_arm = sp.Symbol("L_arm", real=True)
    J_arm = sp.Symbol("J_arm", real=True) 
    DC_arm = sp.Symbol("DC_arm", real=True) 
    theta_arm = sp.Function("theta_arm")(time)
    theta_dot_arm = sp.diff(theta_arm, time)
    theta_ddot_arm = sp.diff(theta_dot_arm, time)

    # pendulum properties
    MASS_PENDULUM_EXTRA = 0.08 # got this by comparing model with experiment and changing by trial and error.
    mass_pendulum = (mass_rod + mass_hinge/2 + MASS_PENDULUM_EXTRA)
    length_pendulum = coupler_offset + length_rod + hinge_center_offset
    rot_inertia_pendulum = mass_pendulum * (radius_rod**2 / 4 + length_pendulum**2 / 12) + mass_pendulum * (length_pendulum / 2)**2  # using parallel axis theorem
    damping_coeff_pendulum = pendulum_damping_factor
    center_of_mass_pendulum = length_pendulum / 2
    POTENTIAL_PENDULUM_ENERGY_FACTOR = 1.93  # got this by comparing model with experiment and changing by trial and error.

    # pendulumn symbols
    M_pendulum = sp.Symbol("M_pendulum", real=True) # mass_rod + mass_hinge/2
    L_pendulum = sp.Symbol("L_pendulum", real=True) # coupler_offset + length_rod + hinge_center_offset
    J_pendulum = sp.Symbol("J_pendulum", real=True) # M_pendulum * (radius_rod**2 / 4 + L_pendulum**2 / 12) + M_pendulum * (L_pendulum / 2)**2  # using parallel axis theorem
    DC_pendulum = sp.Symbol("DC_pendulum", real=True) # 1
    COM_pendulum = L_pendulum / 2 # center of mass is half way of the pendulum rod
    theta_pendulum = sp.Function("theta_pendulum")(time)
    theta_dot_pendulum = sp.diff(theta_pendulum, time)
    theta_ddot_pendulum = sp.diff(theta_dot_pendulum, time)

    # CALCULATING KINETIC ENERGY OF THE SYSTEM
    # arm kinetic energy
    rot_kin_energy_arm = 0.5 * J_arm * theta_dot_arm ** 2
    logger.debug("rot_kin_energy_arm: \n%s", sp.pretty(rot_kin_energy_arm))

    # pendulum kinetic energy
    ## rotational
    rot_kin_energy_pendulum = 0.5 * J_pendulum * theta_dot_pendulum ** 2
    logger.debug("rot_kin_energy_pendulum: \n%s", sp.pretty(rot_kin_energy_pendulum))

    ## linear
    vel_squared_pendulumn = (L_arm * theta_dot_arm + COM_pendulum * theta_dot_pendulum * sp.cos(theta_pendulum)) ** 2 + \
                            (-COM_pendulum * theta_dot_pendulum * sp.sin(theta_pendulum)) ** 2
    lin_kin_energy_pendulum = 0.5 * M_pendulum * vel_squared_pendulumn
    logger.debug("lin_kin_energy_pendulum: \n%s", sp.pretty(lin_kin_energy_pendulum))

    # total kinetic energy
    total_kin_energy = rot_kin_energy_arm + rot_kin_energy_pendulum + lin_kin_energy_pendulum
    logger.debug("total_kin_energy: \n%s", sp.pretty(total_kin_energy))

    # CALCULATING POTENTIAL ENERGY OF THE SYSTEM
    # pendulum potential energy
    pot_energy_pendulum = M_pendulum * G * COM_pendulum * (1 + sp.cos(theta_pendulum)) * POTENTIAL_PENDULUM_ENERGY_FACTOR
    logger.debug("pot_energy_pendulum: \n%s", sp.pretty(pot_energy_pendulum))

    # total potential energy
    total_pot_energy = pot_energy_pendulum

    # CALCULATING LAGRANGIAN
    lagrangian = total_kin_energy - total_pot_energy
    logger.debug("lagrangian: \n%s", sp.pretty(lagrangian))

    # CALCULATING EQUATIONS OF MOTION
    # arm equation of motion
    equation_arm = sp.Eq(sp.diff(sp.diff(lagrangian, theta_dot_arm), time) - sp.diff(lagrangian, theta_arm), torque - DC_arm * theta_dot_arm)
    logger.debug("equation_arm (coupled): \n%s", sp.pretty(equation_arm))

    # pendulum equation of motion
    equation_pendulum = sp.Eq(sp.diff(sp.diff(lagrangian, theta_dot_pendulum), time) - sp.diff(lagrangian, theta_pendulum), -DC_pendulum * theta_dot_pendulum)
    logger.debug("equation_pendulum (coupled): \n%s", sp.pretty(equation_pendulum))

    # DEFINE AND SOLVE THE EQUATIONS
    result = sp.solve([equation_arm, equation_pendulum], [theta_ddot_arm, theta_ddot_pendulum], dict=True)
    theta_ddot_arm_solved = sp.simplify(result[0][theta_ddot_arm])
    theta_ddot_pendulum_solved = sp.simplify(result[0][theta_ddot_pendulum])
    logger.debug("theta_ddot_arm_solved: \n%s", sp.pretty(theta_ddot_arm_solved))
    logger.debug("theta_ddot_pendulum_solved: \n%s", sp.pretty(theta_ddot_pendulum_solved))

    # SUBSTITUTE WITH PHYSICAL VALUES
    theta_ddot_arm_solved = theta_ddot_arm_solved.subs(M_arm, mass_arm).subs(L_arm, length_arm).subs(J_arm, rot_inertia_arm).subs(DC_arm, damping_coeff_arm) \
                                                    .subs(M_pendulum, mass_pendulum).subs(L_pendulum, length_pendulum).subs(J_pendulum, rot_inertia_pendulum).subs(DC_pendulum, damping_coeff_pendulum)
    theta_ddot_pendulum_solved = theta_ddot_pendulum_solved.subs(M_arm, mass_arm).subs(L_arm, length_arm).subs(J_arm, rot_inertia_arm).subs(DC_arm, damping_coeff_arm) \
                                                    .subs(M_pendulum, mass_pendulum).subs(L_pendulum, length_pendulum).subs(J_pendulum, rot_inertia_pendulum).subs(DC_pendulum, damping_coeff_pendulum)
    logger.debug("theta_ddot_arm_solved subbed with physical values: \n%s", sp.pretty(theta_ddot_arm_solved))
    logger.debug("theta_ddot_pendulum_solved subbed with physical values: \n%s", sp.pretty(theta_ddot_pendulum_solved))

    return NonLinearDynamicsModel(theta_arm, theta_dot_arm, theta_pendulum, theta_dot_pendulum, theta_ddot_arm_solved, theta_ddot_pendulum_solved, torque)


def __get_linear_state_space_model(debug=False):

    if debug:
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(logging.CRITICAL)

    logger.debug("In __get_linear_state_space_model()")

    non_linear_dynamics_model = __get_non_linear_dynamics_model(debug=debug)
    
    # Create matrices
    x = sp.Matrix([[non_linear_dynamics_model.theta_arm], 
                    [non_linear_dynamics_model.theta_dot_arm],
                    [non_linear_dynamics_model.theta_pendulum], 
                    [non_linear_dynamics_model.theta_dot_pendulum]])

    u = sp.Matrix([non_linear_dynamics_model.torque])

    state_space = sp.Matrix([[non_linear_dynamics_model.theta_dot_arm], 
                            [non_linear_dynamics_model.theta_ddot_arm], 
                            [non_linear_dynamics_model.theta_dot_pendulum], 
                            [non_linear_dynamics_model.theta_ddot_pendulum]])

    # Compute Jacobian matrices
    A = state_space.jacobian(x)
    B = state_space.jacobian(u)

    # Substitute with equilibrium points
    equilibrium_points = {non_linear_dynamics_model.theta_arm: 0, 
                            non_linear_dynamics_model.theta_dot_arm: 0, 
                            non_linear_dynamics_model.theta_pendulum: 0, 
                            non_linear_dynamics_model.theta_dot_pendulum: 0, 
                            non_linear_dynamics_model.torque: 0}

    A = A.subs(equilibrium_points)
    B = B.subs(equilibrium_points)

    # Convert to numpy arrays
    A = np.array(A)
    B = np.array(B)
    C = np.ones(4)
    D = 0

    logger.debug("A: \n%s", A)
    logger.debug("B: \n%s", B)
    logger.debug("C: \n%s", C)
    logger.debug("D: \n%s", D)

    return LinearStateSpaceModel(A, B, C, D)


def __get_lqr_gains(Q, R, debug=False):

    if debug:
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(logging.CRITICAL)

    logger.debug("In __get_lqr_gains()")

    linear_state_space_model = __get_linear_state_space_model(debug=debug)
    
    K, S, E = control.lqr(linear_state_space_model.A, linear_state_space_model.B, Q, R)

    logger.debug("K: \n%s", K)
    logger.debug("S: \n%s", S)
    logger.debug("E: \n%s", E)

    return K


def load_closed_loop_linear_state_space_model(control_method,debug=False):

    if debug:
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(logging.CRITICAL)
    
    logger.debug("In load_linear_closed_loop_state_space_model()")

    linear_state_space_model = __get_linear_state_space_model(debug=debug)

    if type(control_method) == LinearQuadraticRegulator:
        control_method.K = __get_lqr_gains(control_method.Q, control_method.R,debug=debug)    

    Acl = linear_state_space_model.A - linear_state_space_model.B * control_method.K
    logger.debug("Acl: \n%s", Acl)

    Dcl = 0

    return LinearStateSpaceModel(Acl, linear_state_space_model.B, linear_state_space_model.C, Dcl)


def load_non_linear_dynamics_model_lambdified(debug=False):
    non_linear_dynamics_model = __get_non_linear_dynamics_model(debug=debug)

    theta_ddot_arm_lambdified = sp.lambdify([non_linear_dynamics_model.theta_arm, 
                                            non_linear_dynamics_model.theta_dot_arm, 
                                            non_linear_dynamics_model.theta_pendulum, 
                                            non_linear_dynamics_model.theta_dot_pendulum,
                                            non_linear_dynamics_model.torque], 
                                            non_linear_dynamics_model.theta_ddot_arm)

    theta_ddot_pendulum_lambdified = sp.lambdify([non_linear_dynamics_model.theta_arm, 
                                                non_linear_dynamics_model.theta_dot_arm, 
                                                non_linear_dynamics_model.theta_pendulum, 
                                                non_linear_dynamics_model.theta_dot_pendulum,
                                                non_linear_dynamics_model.torque],
                                                non_linear_dynamics_model.theta_ddot_pendulum)

    return NonLinearDynamicsModelLambdified(theta_ddot_arm_lambdified, theta_ddot_pendulum_lambdified)


def load_linear_state_space_model(debug=False):
    return __get_linear_state_space_model(debug=debug)


if __name__ == "__main__":
    # Run to debug
    __get_non_linear_dynamics_model(debug=True)
    __get_linear_state_space_model(debug=True)
    __get_lqr_gains(Q=np.eye(4), R=1, debug=True)
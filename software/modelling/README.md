# Rotary Inverted Pendulum Modelling

## Overview

This folder provides the code to model and simulate the rotary inverted pendulum. The code is written in Python and utilizes the `sympy` and `numpy` libraries for symbolic and numerical computations.

## Contents

* `model.py`: This file contains the mathematical model of the rotary inverted pendulum, including the equations of motion and the system parameters.
* `experiment`: This folder contains data from experiments conducted on the rotary inverted pendulum to estimate specific parameters.
    * `damping_coefficient`: This folder contains data and code related to the estimation of the damping coefficient of the pendulum.

## Usage

To use the code in this folder, follow these steps:

1. Install the required libraries by running `pip install -r "software/modelling/requirements.txt"` in your terminal.
2. Run the `model.py` file to view the mathematical model of the pendulum through the terminal, however `simulation.py` will run `model.py` anyways when simulating.
3. Run the `simulation.py` file to simulate linear and non-linear models of the system, depending on what lines you uncomment in the file.

```mermaid
flowchart LR
    A[model.py] -- imported by --> B[simulation.py] -- outputs -->  C@{ shape: lean-r, label: "Simulation graphical plots" }
```

## Example Results

Some examples of simulation plots:

![Simulation of closed loop linearised system with LQR derived gains.](figures/simulation_closed_loop_linearised_lqr.png)

![Simulation of non-linear system compared to real data of a freely swinging pendulum in experiment 01.](figures/simulation_non_linear_experiment_comparison_01.png)

![Simulation of non-linear system compared to real data of a freely swinging pendulum in experiment 02.](figures/simulation_non_linear_experiment_comparison_02.png)

![Simulation of non-linear system compared to real data of a freely swinging pendulum in experiment 03.](figures/simulation_non_linear_experiment_comparison_03.png)
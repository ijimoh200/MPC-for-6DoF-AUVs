## Duality-based Feedback Min-Max MMPC for 3D Path Following
This repository provides the MATLAB implementation of the accelerated feedback Min-Max MPC with a multi-objective line-of-sight guidance system (MO-LOSGS) for 3D path-following (PF) by coupled AUVs operating in the presence of obstacles. The code requires the Yalmip solver to be installed in the MATLAB path.

The repository is for the algorithm presented in Chapter 7 of the book: **Robust Model Predictive Control for Autonomous Underwater Vehicles**.

The following input and output variables are defined for the main function fMM-MPC that implements the accelerated feedback min-max MPC algorithm:

```markdown
Input:
      x0 - Initial state vector (includes position and velocity variables)
      yref - reference position and orientation vector
      Poq - Position of the obstacles
      Ts  - Sampling period in seconds
      Tf - Duration of simulation in seconds
      tau_wave - Ocean wave-induced disturbances
Output
      ui - Control input forces and moments of the AUV
      xi - State vector of the AUV
      yi - Output position and orientation vector
      ctime - Computational time of the controller
      tfinal - Record the duration of the task since PF is not constrained by time.
      yrefcalc - Reference computed by the MOLOSGS

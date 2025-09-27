## Velocity Form LPVMPC2 Strategy
This repository contains MATLAB implementations of the velocity-form LPVMPC2 based on a reachable set for integrated trajectory tracking and point stabilisation. The algorithm is described in detail in Sections 5.4 & 5.5 of Chapter 5 of the book **Robust Model Predictive Control for Autonomous Underwater Vehicles**.

The MATLAB code validates the control algorithm using the Naminow-D AUV and compares its performance with the delta input form algorithm for Case 1, which considered integrated trajectory tracking and dynamic positioning control of the AUV.

Input:
      eta_ini - Initial position vector of the AUV
      nv_ini - Initial velocity vector of the AUV
      yref - Reference position vector to be tracked by the AUV
      nu_c - Ocean current disturbance vector
      Ts - Sampling period in seconds
      Tf - Duration of the simulation in seconds
      N - Prediction horizon
      tau_wave - Wave-induced disturbance vector
      
Output
      ui - Input forces and moments vector 
      xi - State vector of the AUV
      yi - Output position vector of the AUV
      e_l - Linear output error of the AUV (scalar)
      e_a - Angular output error of the AUV (scalar)
      dXpred - Predicted state variables in each sampling instant.

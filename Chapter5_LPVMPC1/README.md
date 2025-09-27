# LPV-MPC1-for-AUV-Positioning-Control
The interdependence of the kinematic and dynamic models of marine vehicles is exploited to formulate a velocity form LPV-MPC algorithm to remove the tracking error at steady-state.

The repository contains the three controllers compared in Section 2 of Chapter 5 of the book **Robust Model Predictive Control for Autonomous Underwater Vehicles**
This repository provides a MATLAB function with 6 inputs and three outputs.

Input:
      
      eta - Current position vector measurement
      nu - current velocity measurement or estimate.
      yref - reference trajectory signals
      nu_c - ocean current which is assumed to be unknown to the controller
      Ts - sampling time used for the discrete controller design
      Tf - length of simulation in seconds
      noise - measurement noise
      
 Output:
  
      ui - input forces and moment signals
      xi - state vector incorporating position and velocity vectors
      yi - output vector which is the position vector.    

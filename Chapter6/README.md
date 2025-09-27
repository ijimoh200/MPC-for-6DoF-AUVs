## Tube-based-MPC-with-LOS-replanning

Provides MATLAB code for implementing the tube-based MPC with LOS replanning, using the Naminow-D AUV as a case study. The repository also contains a nonlinear MPC (NMPC) controller used as a benchmark for the tube MPC. The functions TMPC implement the Chapter 6 Tube-based MPC strategy with different tuned values of the sphere of acceptance radius Ra, to illustrate the effect of the radius on performance.

**NB:** The NMPC function is computationally expensive, which can take a considerable amount of time to run on a relatively slow PC.

The following are the inputs and outputs of the function **TMPC** that implements the developed Tube-based controller:
```markdown
Input:
      eta_ini - Initial position vector of the AUV 
      nv_ini - Initial velocity vector of the AUV
      yref - Reference position vector of the AUV
      nu_c - Ocean current disturbance
      Ts - Sampling period in seconds
      Tf - Duration of simulation in seconds
      Ra - Radius of the sphere of acceptance

Output:
      ui - Input forces and moments of the AUV 
      xi - State vector of the AUV
      yi - output position vector of the AUV
      dui - Input increment (change) applied to the AUV
      yrefcalc - Computed reference trajectory by the 3D LOS local replanning strategy
      toc_comp - Computational time of solving the QCQP used to implement the TMPC 


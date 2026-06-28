# MPC-for-6DoF-AUVs
This repository presents the MATLAB codes used to implement the MPC algorithms in the book: 

**Jimoh, I. A., 2026, "Robust Model Predictive Control for Autonomous Underwater Vehicles." _Advances in Industrial Control_, Springer.
https://doi.org/10.1007/978-3-032-16844-3**

The MATLAB implementation is structured in a modular fashion at the functional level, but it does not incorporate a class-based architectural design. While the authors acknowledge that highly modular code can be advantageous in large-scale systems, a script-driven functional approach is adopted to enhance readability and facilitate the reuse of specific components, especially in academic or instructional contexts. The m-files are organised into separate functions dedicated to the nonlinear AUV model, control algorithms, reference generation, local replanning, and predictive matrix construction.  


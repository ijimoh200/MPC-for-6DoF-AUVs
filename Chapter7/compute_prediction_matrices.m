function [A_tilde, B_tilde] = compute_prediction_matrices(eta, nu, nu_prev, N, Ts)
% Compute the A_tilde and B_tilde matrices for the prediction model
%
% Inputs:
% eta - initial state eta(k)
% nu - initial velocity nu(k)
% nu_prev - velocity at previous step nu(k-1)
% N - prediction horizon
% Ts - sampling time
%
% Outputs:
% A_tilde - stacked state transition matrix
% B_tilde - stacked control input matrix

% Formulate J as a function of eta
phi = eta(4);            
theta = eta(5);              
psi = eta(6);

% Coordinate transform term definition
cos1 = cos(phi); cos2 = cos(theta); cos3 = cos(psi);
sin1 = sin(phi); sin2 = sin(theta); sin3 = sin(psi);
tan2 = tan(theta);
% Rotation matrix
J1 = [cos2*cos3 -sin3*cos1+cos3*sin2*sin1 cos3*cos1*sin2+sin3*sin1;
    sin3*cos2 cos3*cos1+sin1*sin2*sin3 sin2*sin3*cos1-cos3*sin1
    -sin2 cos2*sin1 cos2*cos1];

J2 = [1 sin1*tan2 cos1*tan2;
    0 cos1 -sin1;
    0 (sin1/cos2) (cos1/cos2)];

J_fun = @(eta) [J1 zeros(size(J1,1),size(J2,2));
    zeros(size(J2,1),size(J1,2)) J2 ];

A_fun = @(eta) [eye(6), J_fun(eta)*Ts; zeros(6), eye(6)];
B_fun = @(eta) [J_fun(eta)*Ts; eye(6)];

% Initialize dimensions
x_dim = size(A_fun(eta), 1);
u_dim = size(B_fun(eta), 2);

% Initialize A_tilde and B_tilde with zeros
A_tilde = zeros(N * x_dim, x_dim);
B_tilde = zeros(N * x_dim, N * u_dim);

% Initialize current eta, nu, and nu_prev
eta_curr = eta;
nu_curr = nu;
nu_prev_curr = nu_prev;

% Compute the A_tilde and B_tilde matrices
for i = 1:N
    % Calculate A(eta) and B(eta) for the current eta
    A_curr = A_fun(eta_curr);
    B_curr = B_fun(eta_curr);

    % Fill the corresponding block in A_tilde
    if i == 1
        A_tilde(1:x_dim, :) = A_curr;
    else
        A_tilde((i-1)*x_dim+1:i*x_dim, :) = A_curr * A_tilde((i-2)*x_dim+1:(i-1)*x_dim, :);
    end

    % Fill the corresponding block in B_tilde
    for j = 1:i
        B_tilde((i-1)*x_dim+1:i*x_dim, (j-1)*u_dim+1:j*u_dim) = ...
            A_curr^(i-j) * B_curr;
    end

    % Update eta and nu for the next step based on the dynamics
    J_curr = J_fun(eta_curr) * Ts;
    nu_curr = nu_curr + (nu_curr - nu_prev_curr);  % Update nu using (22)
    eta_curr = eta_curr + J_curr * nu_curr;  % Update eta using equation (21)

    % Prepare for the next iteration
    nu_prev_curr = nu_curr;
end
end

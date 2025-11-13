%% ////////////////////////////////////////////////////////////////////////
%  MAIN CUBESAT ATTITUDE CONTROL SIMULATION SCRIPT (Corrected)
%  ////////////////////////////////////////////////////////////////////////
%
%  Author: Gemini AI for [Your Name]
%  Date:   August 31, 2025
%
%  Description:
%  This is the main script to simulate the attitude control of a 1U CubeSat.
%  It initializes the satellite's properties, sets the simulation
%  parameters, and runs a time-based loop to simulate the dynamics and
%  control. It is designed to be the master script from which all other
%  functions are called.
%
%  Software: GNU Octave
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 1. INITIALIZATION
%  Clears the workspace, command window, and closes all figures to ensure
%  a clean run every time.
%  ------------------------------------------------------------------------
clear all;
clc;
close all;

% Add the current folder to the path to ensure all function files are found
addpath(pwd);


%% 2. SIMULATION PARAMETERS
%  Defines the timing for the simulation.
%  ------------------------------------------------------------------------
dt = 0.01;         % [s] Simulation time step. A smaller value increases
                   %     accuracy but also computation time.
t_end = 120;      % [s] Total simulation duration.
t = 0:dt:t_end;   % [s] Time vector for the entire simulation.
num_steps = length(t); % Total number of simulation steps.


%% 3. CUBESAT PHYSICAL PARAMETERS (PLACEHOLDERS)
%  Defines the physical properties of the 1U CubeSat.
%  ------------------------------------------------------------------------
%
%  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
%  !! IMPORTANT: REPLACE THESE PLACEHOLDER VALUES WITH YOUR OWN !!
%  !! These values should come from your Fusion 360 CAD model. !!
%  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
%
%  Inertia Tensor (Moment of Inertia Matrix) about the principal axes.
%  Units are [kg*m^2].
J_xx = 1.8e-3; % Moment of inertia about the body x-axis
J_yy = 1.8e-3; % Moment of inertia about the body y-axis
J_zz = 1.8e-3; % Moment of inertia about the body z-axis

J = diag([J_xx, J_yy, J_zz]); % Inertia Tensor Matrix [kg*m^2]

%% 4. INITIAL AND DESIRED CONDITIONS
%  Sets the starting attitude and angular velocity of the CubeSat, and
%  defines the target (desired) state.
%  ------------------------------------------------------------------------

% --- Initial Conditions ---
% We are starting from the inertial frame, meaning no initial rotation
% and no initial tumble.
q0 = [1; 0; 0; 0];      % Identity quaternion (aligned with inertial frame)
w0 = [0; 0; 0];          % No initial angular velocity [rad/s]

% --- Desired (Target) Conditions ---
% The goal is to orient to a 45-degree pitch.
yaw_desired   = 45;    % [deg]
pitch_desired = 45;   % [deg]
roll_desired  = 45;    % [deg]

% Convert the desired Euler angles to a target quaternion.
q_desired = euler_to_quaternion(deg2rad(yaw_desired), deg2rad(pitch_desired), deg2rad(roll_desired));
w_desired = [0; 0; 0]; % Desired final angular velocity is zero.


%% 5. CONTROLLER SELECTION AND PARAMETERS
%  Choose which controller to use for the simulation.
%  ------------------------------------------------------------------------
CONTROLLER_TYPE = 'LQR'; % Options: 'PID' or 'LQR'

% --- Controller Gains (will be used by the controller functions) ---
if strcmp(CONTROLLER_TYPE, 'PID')
    % PID gains (Final stable values)
    Kp = 0.0008; % Proportional gain
    Ki = 0.0;    % Integral gain (OFF)
    Kd = 0.02;   % Derivative gain
end



if strcmp(CONTROLLER_TYPE, 'LQR')
    % LQR weighting matrices (Final Stable Tuning)
    Q = diag([1, 1, 1, 2, 2, 2]); 
    R = eye(3) * 500;
end

%% 6. DATA STORAGE AND STATE INITIALIZATION
%  Pre-allocate arrays to store the simulation history for plotting.
%  ------------------------------------------------------------------------
state_history = zeros(7, num_steps);
torque_history = zeros(3, num_steps);

% Set the initial state
state_history(:, 1) = [q0; w0];

% Initialize the integral error accumulator for the PID controller
pid_integral_error = [0; 0; 0];


%% 7. MAIN SIMULATION LOOP
%  This loop iterates through time, calling the controller and dynamics
%  at each step to simulate the CubeSat's motion.
%  ------------------------------------------------------------------------
disp(['Starting Simulation for ' CONTROLLER_TYPE ' controller...']);

for i = 1:num_steps-1
    % --- SENSE ---
    q_current = state_history(1:4, i);
    w_current = state_history(5:7, i);

    % --- THINK (Controller) ---
    if strcmp(CONTROLLER_TYPE, 'PID')
        [tau, pid_integral_error] = controller_PID(q_current, w_current, q_desired, w_desired, Kp, Ki, Kd, dt, pid_integral_error);

    elseif strcmp(CONTROLLER_TYPE, 'LQR')
        [tau] = controller_LQR(q_current, w_current, q_desired, w_desired, Q, R, J);
    end

    % Store the calculated torque for plotting
    torque_history(:, i) = tau;

    % --- ACT (Dynamics) ---
    current_state = state_history(:, i);
    state_derivative = cubesat_dynamics(current_state, tau, J);

    % --- INTEGRATION ---
    state_history(:, i+1) = current_state + state_derivative * dt;

    % --- NORMALIZATION ---
    state_history(1:4, i+1) = state_history(1:4, i+1) / norm(state_history(1:4, i+1));
end

disp('Simulation Complete.');

%% 8. VISUALIZATION
%  Call a separate script to plot the results.
%  ------------------------------------------------------------------------
disp('Plotting results...');
plot_results(t, state_history, torque_history, q_desired);

disp('Script finished.');
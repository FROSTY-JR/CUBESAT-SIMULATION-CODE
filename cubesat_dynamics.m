%% ////////////////////////////////////////////////////////////////////////
%  CUBESAT DYNAMICS FUNCTION
%  ////////////////////////////////////////////////////////////////////////
%
%  Author: Gemini AI for [Your Name]
%  Date:   August 31, 2025
%  
%  Description:
%  This function implements the complete rotational dynamics of the CubeSat.
%  It takes the current state (attitude quaternion and angular velocity),
%  the applied control torques, and the satellite's inertia tensor as
%  inputs. It outputs the time derivative of the state, which is then used
%  by an integrator in the main script to propagate the state forward in time.
%
%  Inputs:
%    - state: A 7x1 column vector containing the current state of the
%             CubeSat. state = [q; w], where q is the 4x1 attitude
%             quaternion [qw; qx; qy; qz] and w is the 3x1 angular
%             velocity vector [wx; wy; wz] in the body frame.
%    - tau:   A 3x1 column vector of the control torques [tau_x; tau_y; tau_z]
%             applied by the reaction wheels in the body frame.
%    - J:     The 3x3 inertia tensor of the CubeSat.
%
%  Output:
%    - state_derivative: A 7x1 column vector of the time derivative of the
%                        state, [q_dot; w_dot].
%
%  Software: GNU Octave
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function state_derivative = cubesat_dynamics(state, tau, J)

    % --- 1. Unpack the State Vector ---
    % Extract the attitude quaternion and angular velocity from the input
    % state vector for easier use.
    q = state(1:4); % Attitude Quaternion [qw, qx, qy, qz]'
    w = state(5:7); % Angular Velocity [wx, wy, wz]' in rad/s
    
    
    % --- 2. Attitude Kinematics ---
    % This section calculates the rate of change of the attitude quaternion.
    % The formula is: q_dot = 0.5 * Omega(w) * q
    % where Omega(w) is a special matrix formed from the angular velocity.
    
    % Extract individual angular velocity components for clarity
    wx = w(1);
    wy = w(2);
    wz = w(3);
    
    % Form the Omega matrix
    Omega = [ 0, -wx, -wy, -wz;
             wx,   0,  wz, -wy;
             wy, -wz,   0,  wx;
             wz,  wy, -wx,   0];
             
    % Calculate the quaternion derivative
    q_dot = 0.5 * Omega * q;
    
    
    % --- 3. Rotational Dynamics (Euler's Equations) ---
    % This section calculates the angular acceleration (w_dot) of the CubeSat.
    % The formula is the vector form of Euler's equations of motion:
    % w_dot = inv(J) * (tau - cross(w, J*w))
    
    % Calculate the gyroscopic coupling torque term: cross(w, J*w)
    gyroscopic_torque = cross(w, J * w);
    
    % Calculate the angular acceleration
    % Note: In Octave, the backslash operator '\' can be more efficient and
    % numerically stable than using inv(J)*... for solving J*w_dot = (...).
    w_dot = J \ (tau - gyroscopic_torque);
    
    
    % --- 4. Assemble the Output Vector ---
    % Combine the quaternion derivative and the angular acceleration into
    % a single 7x1 output vector.
    state_derivative = [q_dot; w_dot];

end
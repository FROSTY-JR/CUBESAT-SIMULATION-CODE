%% ////////////////////////////////////////////////////////////////////////
%  PID ATTITUDE CONTROLLER (Final Sign Correction)
%  ////////////////////////////////////////////////////////////////////////

function [tau, p_error_integral_out] = controller_PID(q, w, q_d, w_d, Kp, Ki, Kd, dt, p_error_integral_in)

    % --- 1. Calculate Attitude Error Quaternion ---
    q_conj = quat_conjugate(q);
    q_error = quat_multiply(q_conj, q_d);
    
    % --- 2. Enforce Shortest Path ---
    if q_error(1) < 0
        q_error = -q_error;
    end
    
    % --- 3. Extract Error Vectors ---
    p_error = q_error(2:4);
    d_error = w - w_d;
    
    % --- 4. Update Integral Error ---
    p_error_integral_out = p_error_integral_in + p_error * dt;
    
    % --- 5. Calculate PID Control Torque (CORRECTED SIGNS) ---
    % For this error definition, the P and I terms must be positive
    % to create negative feedback, while the D term remains negative
    % to provide damping.
    tau = Kp * p_error + Ki * p_error_integral_out - Kd * d_error;

end
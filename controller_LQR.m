%% ////////////////////////////////////////////////////////////////////////
%  LQR ATTITUDE CONTROLLER (Definitive Final Version)
%  ////////////////////////////////////////////////////////////////////////

function [tau] = controller_LQR(q, w, q_d, w_d, Q, R, J)
    % This function requires the 'control' package to be loaded.

    % --- 1. Linearized State-Space Model ---
    A = [zeros(3,3), 0.5*eye(3);
         zeros(3,3), zeros(3,3)];

    B = [zeros(3,3);
         inv(J)];

    % --- 2. Calculate the Optimal Gain 'K' ---
    K = lqr(A, B, Q, R);
    
    % --- 3. Calculate the Error Components ---
    q_conj = quat_conjugate(q);
    q_error = quat_multiply(q_conj, q_d);
    
    if q_error(1) < 0
        q_error = -q_error;
    end
    
    p_error_raw = q_error(2:4);
    w_error = w - w_d;
    
    % --- 4. Assemble the Final State Error Vector for the Control Law ---
    % The sign of the attitude error part is FLIPPED here.
    % This makes it compatible with the standard negative feedback law
    % `tau = -K*x`, ensuring the proportional term provides negative
    % feedback while the derivative term provides damping.
    x_error_for_control = [-p_error_raw; w_error];
    
    % --- 5. Calculate Optimal Control Torque ---
    % Use the standard LQR control law with the corrected state vector.
    tau = -K * x_error_for_control;
end
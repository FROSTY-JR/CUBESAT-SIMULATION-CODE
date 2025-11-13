%% ////////////////////////////////////////////////////////////////////////
%  SIMULATION RESULTS PLOTTING FUNCTION (Improved Version)
%  ////////////////////////////////////////////////////////////////////////

function plot_results(t, state_history, torque_history, q_desired)

    % --- 1. Post-Processing: Calculate Attitude Error Angle ---
    num_steps = length(t);
    error_angle_history = zeros(1, num_steps);
    q_d_conj = quat_conjugate(q_desired);

    for i = 1:num_steps
        q_current = state_history(1:4, i);
        q_err = quat_multiply(q_current, q_d_conj);
        angle = 2 * acos(min(1.0, q_err(1)));
        error_angle_history(i) = rad2deg(angle);
    end

    % --- 2. Create and Enlarge the Figure Window ---
    figure('Name', 'CubeSat Attitude Control Simulation Results', 'NumberTitle', 'off', ...
           'Position', [100, 100, 800, 900]); % Set position and size [left, bottom, width, height]

    % --- 3. Create Enhanced Plots ---
    
    % -- Subplot 1: Attitude Error --
    subplot(3, 1, 1);
    plot(t, error_angle_history, 'b-', 'LineWidth', 2.5); % Thicker blue line
    title('Attitude Error', 'FontSize', 14);
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Error Angle (deg)', 'FontSize', 12);
    grid on;
    legend({'Error Angle'}, 'FontSize', 12);
    set(gca, 'FontSize', 12); % Set axis tick font size

    % -- Subplot 2: Angular Velocity --
    subplot(3, 1, 2);
    plot(t, state_history(5,:), 'r-', 'LineWidth', 2);
    hold on;
    plot(t, state_history(6,:), 'g-', 'LineWidth', 2);
    plot(t, state_history(7,:), 'b-', 'LineWidth', 2);
    title('Angular Velocity in Body Frame', 'FontSize', 14);
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Velocity (rad/s)', 'FontSize', 12);
    grid on;
    legend({'w_x', 'w_y', 'w_z'}, 'FontSize', 12, 'Location', 'east');
    set(gca, 'FontSize', 12);
    hold off;

    % -- Subplot 3: Control Torques --
    subplot(3, 1, 3);
    plot(t, torque_history(1,:), 'r-', 'LineWidth', 2);
    hold on;
    plot(t, torque_history(2,:), 'g-', 'LineWidth', 2);
    plot(t, torque_history(3,:), 'b-', 'LineWidth', 2);
    title('Control Torques from Reaction Wheels', 'FontSize', 14);
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Torque (N*m)', 'FontSize', 12);
    grid on;
    legend({'tau_x', 'tau_y', 'tau_z'}, 'FontSize', 12, 'Location', 'east');
    set(gca, 'FontSize', 12);
    hold off;

end
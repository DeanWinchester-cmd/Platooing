% Compare Multiple Controllers for String Stability
% This script runs the braking scenario with different controllers and compares results
clc;
clear;
close all;

% List of controllers to test
%CTH= Constant time headway, PID, Sliding mode, nonlinear and cacc
controller_types = {'CTH', 'PID', 'SMC', 'Nonlinear', 'CACC'};
num_controllers = length(controller_types);

% Arrays to store performance metrics
error_amp_ratios = zeros(1, num_controllers);
min_spacings = zeros(1, num_controllers);
max_accel_follower2 = zeros(1, num_controllers);
brake_length_margins = zeros(1, num_controllers);  % New metric for brake length safety margin

% Parameters common to all simulations
sim_time = 40;
dt = 0.01;
steps = sim_time/dt;
init_velocity = 20;
init_spacing = 30;
desired_distance = 10;
h = 1.0;
all_figures = [];

% Brake parameters
reaction_time = 0.3;  % Driver reaction time in seconds
friction_coef = 0.7;  % Road friction coefficient (dry asphalt)
g = 9.81;            % Gravitational acceleration m/s^2

% Prepare figure for comparison
figure('Name', 'Controller Comparison for String Stability', 'Position', [100, 100, 1200, 600]);

% Run simulation for each controller type
for c = 1:num_controllers
    % Arrays for storing results
    t = zeros(1, steps);
    positions = zeros(3, steps);
    velocities = zeros(3, steps);
    accelerations = zeros(3, steps);
    spacings = zeros(2, steps);
    control_inputs = zeros(2, steps);
    brake_lengths = zeros(3, steps);  % Store brake lengths for each vehicle
    
    % Initialize vehicles
    vehicles = cell(1, 3);
    vehicles{1} = VehicleDynamics(1500, 0, init_velocity, 0);
    vehicles{2} = VehicleDynamics(1500, -init_spacing, init_velocity, 0);
    vehicles{3} = VehicleDynamics(1500, -2*init_spacing, init_velocity, 0);
    
    % Initialize controllers based on type
    controllers = cell(1, 2);
    ctrl_type = controller_types{c};
    
    switch ctrl_type
        case 'CTH'
            controllers{1} = CTHController(h, 0.5, 0.2, desired_distance);
            controllers{2} = CTHController(h, 0.5, 0.2, desired_distance);
        case 'PID'
            controllers{1} = PIDController(h, desired_distance, 0.6, 0.1, 0.3, dt);
            controllers{2} = PIDController(h, desired_distance, 0.6, 0.1, 0.3, dt);
        case 'SMC'
            controllers{1} = SMCController(h, desired_distance, 2.0, 0.5, 0.1);
            controllers{2} = SMCController(h, desired_distance, 2.0, 0.5, 0.1);
        case 'Nonlinear'
            controllers{1} = NonlinearController(h, desired_distance, 0.5, 0.8, 0.3, 2.0, 3.0);
            controllers{2} = NonlinearController(h, desired_distance, 0.5, 0.8, 0.3, 2.0, 3.0);
        case 'CACC'
            controllers{1} = CACCController(0.7, desired_distance, 0.45, 0.25, 0.8, 0.2, 0.05);
            controllers{2} = CACCController(0.7, desired_distance, 0.45, 0.25, 0.8, 0.2, 0.05);
    end
    
    % Simulation Loop
    for i = 1:steps
        t(i) = (i-1) * dt;
        
        % Lead vehicle control profile (braking scenario)
        if t(i) < 5
            u1 = 0;
        elseif t(i) < 10
            u1 = -3.0;
        elseif t(i) < 15
            u1 = -1.0;
        elseif t(i) < 20
            u1 = 0;
        else
            u1 = min(1.0, max(0, (init_velocity - vehicles{1}.velocity) * 0.5));
        end
        
        % Update lead vehicle
        vehicles{1}.update(u1);
        positions(1, i) = vehicles{1}.position;
        velocities(1, i) = vehicles{1}.velocity;
        accelerations(1, i) = vehicles{1}.acceleration;
        
        % Calculate brake length for lead vehicle
        % Brake length = reaction distance + braking distance
        v1 = max(0, velocities(1, i));  % Ensure velocity is non-negative
        reaction_distance = v1 * reaction_time;
        braking_distance = (v1^2) / (2 * friction_coef * g);
        brake_lengths(1, i) = reaction_distance + braking_distance;
        
        % Update following vehicles
        for j = 2:3
            prev_idx = j-1;
            ctrl_idx = j-1;
            
            % Special handling for CACC which needs more inputs
            if strcmp(ctrl_type, 'CACC') && j == 3
                % For the second follower with CACC, we need lead vehicle info too
                u = controllers{ctrl_idx}.calculate_control(vehicles{j}.position,vehicles{j}.velocity,vehicles{j}.acceleration,vehicles{prev_idx}.position,vehicles{prev_idx}.velocity,vehicles{prev_idx}.acceleration,vehicles{1}.position,vehicles{1}.velocity,vehicles{1}.acceleration,dt);
            elseif strcmp(ctrl_type, 'CACC')
                % For the first follower with CACC
                u = controllers{ctrl_idx}.calculate_control(vehicles{j}.position,vehicles{j}.velocity,vehicles{j}.acceleration,vehicles{prev_idx}.position,vehicles{prev_idx}.velocity,vehicles{prev_idx}.acceleration,vehicles{1}.position,vehicles{1}.velocity,vehicles{1}.acceleration,dt);
            else
                % Standard controller call for other types
                u = controllers{ctrl_idx}.calculate_control(vehicles{j}.position,vehicles{j}.velocity,vehicles{j}.acceleration,vehicles{prev_idx}.position,vehicles{prev_idx}.velocity,vehicles{prev_idx}.acceleration);
            end
            
            control_inputs(j-1, i) = u;
            vehicles{j}.update(u);
            
            positions(j, i) = vehicles{j}.position;
            velocities(j, i) = vehicles{j}.velocity;
            accelerations(j, i) = vehicles{j}.acceleration;
            
            spacings(j-1, i) = vehicles{prev_idx}.position - vehicles{j}.position;
            
            % Calculate brake length for following vehicles
            v_current = max(0, velocities(j, i));  % Ensure velocity is non-negative
            reaction_distance = v_current * reaction_time;
            braking_distance = (v_current^2) / (2 * friction_coef * g);
            brake_lengths(j, i) = reaction_distance + braking_distance;
        end
    end
    
    % Calculate performance metrics
    spacing_error1 = spacings(1, :) - (desired_distance + h * velocities(2, :));
    spacing_error2 = spacings(2, :) - (desired_distance + h * velocities(3, :));
    
    % Calculate minimum required spacing based on brake length difference
    % This represents how much extra space vehicle j needs compared to vehicle j-1
    brake_length_diff1 = brake_lengths(2, :) - brake_lengths(1, :);
    brake_length_diff2 = brake_lengths(3, :) - brake_lengths(2, :);
    
    % Safety margin is actual spacing minus required brake length difference
    safety_margin1 = spacings(1, :) - brake_length_diff1;
    safety_margin2 = spacings(2, :) - brake_length_diff2;
    
    brake_phase = t >= 5 & t <= 15;
    max_error1 = max(abs(spacing_error1(brake_phase)));
    max_error2 = max(abs(spacing_error2(brake_phase)));
    
    error_amp_ratios(c) = max_error2 / max_error1;
    min_spacings(c) = min(min(spacings(:, brake_phase)));
    
    % New metric: minimum safety margin during braking
    min_safety_margin1 = min(safety_margin1(brake_phase));
    min_safety_margin2 = min(safety_margin2(brake_phase));
    brake_length_margins(c) = min(min_safety_margin1, min_safety_margin2);
    
    max_accel_follower2(c) = max(abs(accelerations(3, brake_phase)));
    
    % Plot spacing errors for this controller
    subplot(2, 1, 1);
    plot(t, spacing_error1, 'DisplayName', [ctrl_type ' - Vehicle 2']);
    hold on;
    subplot(2, 1, 2);
    plot(t, spacing_error2, 'DisplayName', [ctrl_type ' - Vehicle 3']);
    hold on;
    
    % Generate individual plots for each controller
    figure('Name', ['Brake Scenario - ' ctrl_type], 'Position', [100, 100, 1200, 800]);
    fig_handle = gcf; %get curr figure handle
    all_figures = [all_figures, fig_handle]; %add to array

    % Velocity plots
    subplot(3, 2, 1);
    plot(t, velocities);
    legend('Lead', 'Follower 1', 'Follower 2');
    title(['Velocity - ' ctrl_type]);
    ylabel('Velocity (m/s)');
    grid on;
    
    % Spacing plots
    subplot(3, 2, 2);
    plot(t, spacings);
    hold on;
    plot(t, desired_distance + h * velocities(2, :), 'k--');
    plot(t, desired_distance + h * velocities(3, :), 'k--');
    legend('Spacing 1-2', 'Spacing 2-3', 'Desired spacing');
    title(['Inter-vehicle Spacing - ' ctrl_type]);
    ylabel('Spacing (m)');
    grid on;
    
    % Spacing error plots
    subplot(3, 2, 3);
    plot(t, spacing_error1, t, spacing_error2);
    legend('Error 1-2', 'Error 2-3');
    title(['Spacing Error - ' ctrl_type]);
    xlabel('Time (s)');
    ylabel('Error (m)');
    grid on;
    
    % Acceleration plots
    subplot(3, 2, 4);
    plot(t, accelerations);
    legend('Lead', 'Follower 1', 'Follower 2');
    title(['Acceleration - ' ctrl_type]);
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    grid on;
    
    % Brake length plots
    subplot(3, 2, 5);
    plot(t, brake_lengths);
    legend('Lead', 'Follower 1', 'Follower 2');
    title(['Brake Length - ' ctrl_type]);
    xlabel('Time (s)');
    ylabel('Length (m)');
    grid on;
    
    % Safety margin plots
    subplot(3, 2, 6);
    plot(t, safety_margin1, t, safety_margin2);
    hold on;
    plot(t, zeros(size(t)), 'r--');
    legend('Safety Margin 1-2', 'Safety Margin 2-3', 'Safety Threshold');
    title(['Brake Length Safety Margin - ' ctrl_type]);
    xlabel('Time (s)');
    ylabel('Margin (m)');
    grid on;
    
    fprintf('\n======== %s Controller Analysis ========\n', ctrl_type);
    fprintf('Error amplification ratio: %.2f\n', error_amp_ratios(c));
    fprintf('Minimum spacing: %.2f m\n', min_spacings(c));
    fprintf('Minimum brake length safety margin: %.2f m\n', brake_length_margins(c));
    fprintf('Maximum acceleration (Vehicle 3): %.2f m/s^2\n', max_accel_follower2(c));
    
    if error_amp_ratios(c) <= 1.0
        fprintf('String stability assessment: STABLE\n');
    else
        fprintf('String stability assessment: UNSTABLE\n');
    end
    
    if brake_length_margins(c) >= 0
        fprintf('Brake safety assessment: SAFE\n');
    else
        fprintf('Brake safety assessment: UNSAFE - insufficient stopping distance\n');
    end
end

% Finish the comparison plots
figure(1);
all_figures=[all_figures, figure(1)];
subplot(2, 1, 1);
title('Spacing Error Comparison - Vehicle 2');
ylabel('Error (m)');
grid on;
legend('show');

subplot(2, 1, 2);
title('Spacing Error Comparison - Vehicle 3');
xlabel('Time (s)');
ylabel('Error (m)');
grid on;
legend('show');

% Create bar charts for comparison
figure('Name', 'Controller Performance Comparison', 'Position', [100, 100, 1200, 400]);
all_figures = [all_figures, gcf];
subplot(1, 4, 1);
bar(error_amp_ratios);
set(gca, 'XTickLabel', controller_types);
title('Error Amplification Ratio');
ylabel('Ratio');
grid on;
hold on;
% Add horizontal line at y=1 for string stability boundary
plot([0.5 num_controllers+0.5], [1 1], 'r--', 'LineWidth', 2);
text(num_controllers/2, 1.05, 'String Stability Boundary', 'Color', 'r', 'HorizontalAlignment', 'center');

subplot(1, 4, 2);
bar(min_spacings);
set(gca, 'XTickLabel', controller_types);
title('Minimum Spacing');
ylabel('Distance (m)');
grid on;

subplot(1, 4, 3);
bar(brake_length_margins);
set(gca, 'XTickLabel', controller_types);
title('Brake Length Safety Margin');
ylabel('Margin (m)');
grid on;
hold on;
% Add horizontal line at y=0 for safety boundary
plot([0.5 num_controllers+0.5], [0 0], 'r--', 'LineWidth', 2);
text(num_controllers/2, 0.5, 'Safety Threshold', 'Color', 'r', 'HorizontalAlignment', 'center');

subplot(1, 4, 4);
bar(max_accel_follower2);
set(gca, 'XTickLabel', controller_types);
title('Max Acceleration (Vehicle 3)');
ylabel('Acceleration (m/s^2)');
grid on;


% Print summary
fprintf('\n======== Controller Comparison Summary ========\n');
[~, best_idx] = min(error_amp_ratios);
fprintf('Best controller for string stability: %s (ratio: %.2f)\n', controller_types{best_idx}, error_amp_ratios(best_idx));

[~, best_spacing_idx] = max(min_spacings);
fprintf('Best controller for maintaining spacing: %s (min spacing: %.2f m)\n', controller_types{best_spacing_idx}, min_spacings(best_spacing_idx));

[~, best_comfort_idx] = min(max_accel_follower2);
fprintf('Best controller for passenger comfort: %s (max accel: %.2f m/s^2)\n', controller_types{best_comfort_idx}, max_accel_follower2(best_comfort_idx));
% 
% timestamp = datetime('now');
% timestamp_str = datestr(timestamp,'yyyymmdd_HHMMSS');
% filename = ['controller_comparison_', timestamp_str,'.pdf'];
% 
% %Export all flles as pdf 
% for i=1:length(all_figures)
%     if i==1
%         exportgraphics(all_figures(i),filename);
%     else
%         exportgraphics(all_figures(i), filename, 'Append',true);
%     end
% end
% fprintf(filename);

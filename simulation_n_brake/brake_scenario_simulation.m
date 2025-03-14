% String Stability Braking Scenario Simulation
% This script simulates a braking scenario with 3 vehicles in a platoon
% and evaluates string stability performance using different controllers

%% Setup Simulation Parameters
clc;
clear;
close all;

% Simulation parameters
sim_time = 100;        % Simulation time (s)
dt = 0.01;            % Time step (s)
steps = sim_time/dt;  % Number of simulation steps

% Arrays for storing results
t = zeros(1, steps);
positions = zeros(3, steps);  % Position of each vehicle
velocities = zeros(3, steps); % Velocity of each vehicle
accelerations = zeros(3, steps); % Acceleration of each vehicle
spacings = zeros(2, steps);   % Inter-vehicle spacings
control_inputs = zeros(2, steps); % Control inputs for following vehicles

%% Initialize Vehicles
% Create 3 vehicles with initial positions spaced by 20m
% All vehicles start at 20 m/s (72 km/h)
init_velocity = 20;   % Initial velocity (m/s)
init_spacing = 30;    % Initial spacing between vehicles (m)

% Initialize vehicles
vehicles = cell(1, 3);
vehicles{1} = VehiclesDynamics(1500, 0, init_velocity, 0);           % Lead vehicle
vehicles{2} = VehiclesDynamics(1500, -init_spacing, init_velocity, 0);    % First follower
vehicles{3} = VehiclesDynamics(1500, -2*init_spacing, init_velocity, 0);  % Second follower

%% Initialize Controllers
% You can choose which controller to use for each following vehicle
% Options: CTHController, PIDController, SMCController, etc.

% Create controllers for each following vehicle
controllers = cell(1, 2);

% Parameters for controllers
h = 1.0;                % Time headway (s)
desired_distance = 10;  % Standstill distance (m)

% Uncomment ONE set of controllers at a time to test different configurations

% Option 1: Both following vehicles use CTH Controller
% controllers{1} = CTHController(h, 0.5, 0.2, desired_distance);
% controllers{2} = CTHController(h, 0.5, 0.2, desired_distance);
% controller_name = 'CTH';

% Option 2: Both following vehicles use PID Controller
% controllers{1} = PIDController(h, desired_distance, 0.6, 0.1, 0.3, dt);
% controllers{2} = PIDController(h, desired_distance, 0.6, 0.1, 0.3, dt);
% controller_name = 'PID';

% Option 3: Both following vehicles use Sliding Mode Controller
% controllers{1} = SMCController(h, desired_distance, 2.0, 0.5, 0.1);
% controllers{2} = SMCController(h, desired_distance, 2.0, 0.5, 0.1);
% controller_name = 'SMC';

% Option 4: Both following vehicles use Nonlinear Controller
% controllers{1} = NonlinearController(h, desired_distance, 0.5, 0.8, 0.3, 2.0, 3.0);
% controllers{2} = NonlinearController(h, desired_distance, 0.5, 0.8, 0.3, 2.0, 3.0);
% controller_name = 'Nonlinear';

% Option 5: Mixed controllers
controllers{1} = CTHController(h, 0.5, 0.2, desired_distance);
controllers{2} = PIDController(h, desired_distance, 0.6, 0.1, 0.3, dt);
controller_name = 'Mixed-CTH-PID';

%% Simulation Loop
for i = 1:steps
    t(i) = (i-1) * dt;
    
    % Lead vehicle control profile (braking scenario)
    if t(i) < 5
        % Steady cruising initially
        u1 = 0;
    elseif t(i) < 10
        % Hard braking
        u1 = -3.0;
    elseif t(i) < 15
        % Less aggressive braking
        u1 = -1.0;
    elseif t(i) < 20
        % Maintain constant speed
        u1 = 0;
    else
        % Accelerate back to cruising speed
        u1 = min(1.0, max(0, (init_velocity - vehicles{1}.velocity) * 0.5));
    end
    
    % Update lead vehicle
    vehicles{1}.update(u1);
    
    % Store lead vehicle data
    positions(1, i) = vehicles{1}.position;
    velocities(1, i) = vehicles{1}.velocity;
    accelerations(1, i) = vehicles{1}.acceleration;
    
    % Update following vehicles
    for j = 2:3
        % Previous vehicle index
        prev_idx = j-1;
        
        % Controller index
        ctrl_idx = j-1;
        
        % Calculate control input using controller
        u = controllers{ctrl_idx}.calculate_control(vehicles{j}.position, ...
            vehicles{j}.velocity,vehicles{j}.acceleration,vehicles{prev_idx}.position, ...
            vehicles{prev_idx}.velocity,vehicles{prev_idx}.acceleration);
        
        % Store control input
        control_inputs(j-1, i) = u;
        
        % Update vehicle dynamics
        vehicles{j}.update(u);
        
        % Store vehicle data
        positions(j, i) = vehicles{j}.position;
        velocities(j, i) = vehicles{j}.velocity;
        accelerations(j, i) = vehicles{j}.acceleration;
        
        % Store inter-vehicle spacing
        spacings(j-1, i) = vehicles{prev_idx}.position - vehicles{j}.position;
    end
end

%% Calculate String Stability Metrics
% 1. Spacing error amplification
spacing_error1 = spacings(1, :) - (desired_distance + h * velocities(2, :));
spacing_error2 = spacings(2, :) - (desired_distance + h * velocities(3, :));

% Find maximum errors during braking phase (between 5-15s)
brake_phase = t >= 5 & t <= 15;
max_error1 = max(abs(spacing_error1(brake_phase)));
max_error2 = max(abs(spacing_error2(brake_phase)));

% String stability metric: error amplification ratio
error_amplification = max_error2 / max_error1;
string_stable = error_amplification <= 1.0;

% 2. Acceleration amplification
acc_phase = t >= 5 & t <= 15;
max_acc1 = max(abs(accelerations(1, acc_phase)));
max_acc2 = max(abs(accelerations(2, acc_phase)));
max_acc3 = max(abs(accelerations(3, acc_phase)));

acc_amplification1 = max_acc2 / max_acc1;
acc_amplification2 = max_acc3 / max_acc2;

%% Plot Results
figure('Name', ['Brake Scenario Simulation - ' controller_name], 'Position', [100, 100, 1000, 800]);

% Position plots
subplot(3, 2, 1);
plot(t, positions);
legend('Lead', 'Follower 1', 'Follower 2');
title('Position');
ylabel('Position (m)');
grid on;

% Velocity plots
subplot(3, 2, 3);
plot(t, velocities);
legend('Lead', 'Follower 1', 'Follower 2');
title('Velocity');
ylabel('Velocity (m/s)');
grid on;

% Acceleration plots
subplot(3, 2, 5);
plot(t, accelerations);
legend('Lead', 'Follower 1', 'Follower 2');
title('Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
grid on;

% Spacing plots
subplot(3, 2, 2);
plot(t, spacings);
hold on;
% Plot desired spacing for reference
plot(t, desired_distance + h * velocities(2, :), 'k--');
plot(t, desired_distance + h * velocities(3, :), 'k--');
legend('Spacing 1-2', 'Spacing 2-3', 'Desired spacing');
title('Inter-vehicle Spacing');
ylabel('Spacing (m)');
grid on;

% Spacing error plots
subplot(3, 2, 4);
plot(t, spacing_error1, t, spacing_error2);
legend('Error 1-2', 'Error 2-3');
title('Spacing Error');
ylabel('Error (m)');
grid on;

% Control input plots
subplot(3, 2, 6);
plot(t, control_inputs);
legend('Control Input 2', 'Control Input 3');
title('Control Inputs');
xlabel('Time (s)');
ylabel('Input Force (N)');
grid on;

%% Display String Stability Metrics
fprintf('\n======== String Stability Analysis (%s) ========\n', controller_name);
fprintf('Maximum spacing error (Vehicle 2): %.2f m\n', max_error1);
fprintf('Maximum spacing error (Vehicle 3): %.2f m\n', max_error2);
fprintf('Error amplification ratio: %.2f\n', error_amplification);
if string_stable
    fprintf('String stability assessment: STABLE (error amplification <= 1.0)\n');
else
    fprintf('String stability assessment: UNSTABLE (error amplification > 1.0)\n');
end

fprintf('\nAcceleration amplification ratios:\n');
fprintf('Lead to Follower 1: %.2f\n', acc_amplification1);
fprintf('Follower 1 to Follower 2: %.2f\n', acc_amplification2);

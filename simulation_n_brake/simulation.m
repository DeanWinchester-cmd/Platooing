% Simulation Script to run all controllers
clc;
clear;

% Number of vehicles in the platoon
num_vehicles = 5;

% Time parameters
T = 60;           % Total simulation time (seconds)
dt = 0.1;         % Time step (seconds)
time_steps = T/dt;

% Initial positions, velocities, and accelerations of vehicles
initial_positions = [0, 10, 20, 30, 40]; % Starting positions (m)
initial_velocities = [20, 20, 20, 20, 20]; % Starting velocities (m/s)
initial_accelerations = zeros(1, num_vehicles); % Initial accelerations (m/s^2)

% Set up the controllers for each vehicle (use any controller)
controllers = cell(1, num_vehicles);

% Example: using the ConsensusController for all vehicles, but you can switch this up
for i = 1:num_vehicles
    % Example: initializing with ConsensusController
    controllers{i} = ConsensusController(1.0, 10, 0.5, 0.3, 0.2, 0.5, 0.3, i == 1, true);
end

% Initialize vehicle states (positions, velocities, accelerations)
positions = initial_positions;
velocities = initial_velocities;
accelerations = initial_accelerations;

% Initialize arrays to store data for analysis
data_positions = zeros(num_vehicles, time_steps);
data_velocities = zeros(num_vehicles, time_steps);
data_accelerations = zeros(num_vehicles, time_steps);
inter_vehicle_distances = zeros(num_vehicles-1, time_steps); % For storing inter-vehicular distances

% Main simulation loop
for t = 1:time_steps
    % Store current state for analysis
    data_positions(:, t) = positions;
    data_velocities(:, t) = velocities;
    data_accelerations(:, t) = accelerations;

    % Update each vehicle's state
    for i = 1:num_vehicles
        % Determine positions, velocities, and accelerations for neighbors (predecessor, follower, leader)
        prev_pos = positions(max(i-1, 1));  % Previous vehicle (or leader for first vehicle)
        prev_vel = velocities(max(i-1, 1));
        prev_acc = accelerations(max(i-1, 1));
        
        next_pos = positions(min(i+1, num_vehicles));  % Next vehicle (or follower for last vehicle)
        next_vel = velocities(min(i+1, num_vehicles));
        next_acc = accelerations(min(i+1, num_vehicles));
        
        % For the leader vehicle
        if i == 1
            lead_pos = positions(i);   % Leader's own position
            lead_vel = velocities(i);  % Leader's own velocity
            lead_acc = accelerations(i);  % Leader's own acceleration
        else
            lead_pos = positions(1);  % Leader is always the first vehicle
            lead_vel = velocities(1); 
            lead_acc = accelerations(1);
        end

        % Compute control input for the vehicle using its respective controller
        u = controllers{i}.calculate_control(positions(i), velocities(i), accelerations(i), prev_pos, prev_vel, prev_acc, next_pos, next_vel, next_acc, lead_pos, lead_vel, lead_acc);
        
        % Update vehicle dynamics (simple model: acceleration -> velocity -> position)
        accelerations(i) = u;  % Update acceleration
        velocities(i) = velocities(i) + accelerations(i) * dt;  % Update velocity (Euler method)
        positions(i) = positions(i) + velocities(i) * dt;  % Update position
    end

    % Calculate inter-vehicular distances (distance between consecutive vehicles)
    for i = 1:num_vehicles-1
        inter_vehicle_distances(i, t) = positions(i+1) - positions(i);
    end

    % % Optional: plot or visualize the results every few steps
    % if mod(t, 50) == 0
    %     plot(data_positions(:, 1:t), 'LineWidth', 2);
    %     title('Vehicle Positions Over Time');
    %     xlabel('Time (s)');
    %     ylabel('Position (m)');
    %     legend(arrayfun(@(x) ['Vehicle ' num2str(x)], 1:num_vehicles, 'UniformOutput', false));
    %     drawnow;
    % end
end

% Plot the final results (positions, velocities, accelerations, and inter-vehicular distances)
figure;

% Plot positions
subplot(4,1,1);
plot((1:time_steps)*dt, data_positions');
title('Positions of Vehicles');
xlabel('Time (s)');
ylabel('Position (m)');
legend(arrayfun(@(x) ['Vehicle ' num2str(x)], 1:num_vehicles, 'UniformOutput', false));

% Plot velocities
subplot(4,1,2);
plot((1:time_steps)*dt, data_velocities');
title('Velocities of Vehicles');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend(arrayfun(@(x) ['Vehicle ' num2str(x)], 1:num_vehicles, 'UniformOutput', false));

% Plot accelerations
subplot(4,1,3);
plot((1:time_steps)*dt, data_accelerations');
title('Accelerations of Vehicles');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend(arrayfun(@(x) ['Vehicle ' num2str(x)], 1:num_vehicles, 'UniformOutput', false));

% Plot inter-vehicular distances
subplot(4,1,4);
plot((1:time_steps)*dt, inter_vehicle_distances');
title('Inter-Vehicular Distances');
xlabel('Time (s)');
ylabel('Distance (m)');
legend(arrayfun(@(x) ['Distance between Vehicle ' num2str(x) ' and Vehicle ' num2str(x+1)], 1:num_vehicles-1, 'UniformOutput', false));


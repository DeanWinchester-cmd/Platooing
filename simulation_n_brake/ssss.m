% Define parameters for the vehicles
num_vehicles = 5;  % Number of vehicles in the platoon
time_total = 100;  % Total simulation time (s)
dt = 0.1;  % Time step (s)
initial_velocity = 20;  % Initial velocity (m/s)
desired_distance = 2.5;  % Desired safe distance between vehicles (m)
h_prev = 1.0;  % Time headway for predecessor
h_next = 0.8;  % Time headway for follower
k_prev = 0.5;  % Gain for predecessor error
k_next = 0.3;  % Gain for follower error
k_v = 0.4;  % Velocity error gain

% Initial positions and velocities for the platoon
positions = linspace(0, (num_vehicles - 1) * desired_distance, num_vehicles);  % Start positions
velocities = repmat(initial_velocity, 1, num_vehicles);  % Same initial velocity for all vehicles
accelerations = zeros(1, num_vehicles);  % No acceleration initially

% Time vector for simulation
time = 0:dt:time_total;

% Define braking event in the lead vehicle at t = 50s
brake_time = 50;  % Time when the lead vehicle starts braking
brake_deceleration = -3.0;  % Deceleration for braking (m/s^2)

% Initialize arrays to store simulation results
distance_travelled = zeros(num_vehicles, length(time));
velocities_history = zeros(num_vehicles, length(time));
accelerations_history = zeros(num_vehicles, length(time));
inter_vehicle_distances = zeros(num_vehicles - 1, length(time));  % Stores the distance between each pair of vehicles

% Simulation loop
for t_idx = 1:length(time)
    % Apply braking for the lead vehicle at brake_time
    if time(t_idx) >= brake_time
        lead_vehicle_acc = brake_deceleration;  % Apply deceleration for the lead vehicle
    else
        lead_vehicle_acc = 0;  % No braking before brake_time
    end
    
    % Update the lead vehicle dynamics (first vehicle)
    positions(1) = positions(1) + velocities(1) * dt;
    velocities(1) = velocities(1) + lead_vehicle_acc * dt;  % Apply braking
    accelerations(1) = lead_vehicle_acc;  % Store the acceleration for the lead vehicle
    
    % Update the following vehicles using the controller
    for i = 2:num_vehicles
        % Get positions and velocities for the current vehicle and its neighbors
        ego_pos = positions(i);
        ego_vel = velocities(i);
        ego_acc = accelerations(i);
        
        prev_pos = positions(i-1);
        prev_vel = velocities(i-1);
        prev_acc = accelerations(i-1);
        
        if i < num_vehicles
            next_pos = positions(i+1);
            next_vel = velocities(i+1);
            next_acc = accelerations(i+1);
        else
            next_pos = positions(i);
            next_vel = velocities(i);
            next_acc = accelerations(i);
        end
        
        % Calculate the control input using the BidirectionalController
        controller = BidirectionalController(h_prev, h_next, desired_distance, k_prev, k_next, k_v);
        control_input = controller.calculate_control(ego_pos, ego_vel, ego_acc, prev_pos, prev_vel, prev_acc, next_pos, next_vel, next_acc);
        
        % Update the vehicle dynamics
        velocities(i) = velocities(i) + control_input * dt;  % Apply control input
        positions(i) = positions(i) + velocities(i) * dt;
        accelerations(i) = control_input;  % Store acceleration
    end
    
    % Compute inter-vehicle distances
    for i = 1:num_vehicles-1
        inter_vehicle_distances(i, t_idx) = positions(i+1) - positions(i);
    end
    
    % Store the results for plotting
    distance_travelled(:, t_idx) = positions;
    velocities_history(:, t_idx) = velocities;
    accelerations_history(:, t_idx) = accelerations;
end

% Plot results
figure;

% Position plot
subplot(4,1,1);
plot(time, distance_travelled');
title('Vehicle Positions Over Time');
xlabel('Time (s)');
ylabel('Position (m)');
grid on;

% Velocity plot
subplot(4,1,2);
plot(time, velocities_history');
title('Vehicle Velocities Over Time');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
grid on;

% Acceleration plot
subplot(4,1,3);
plot(time, accelerations_history');
title('Vehicle Accelerations Over Time');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
grid on;

% Inter-vehicle Distance plot
subplot(4,1,4);
plot(time, inter_vehicle_distances');
title('Inter-Vehicle Distances Over Time');
xlabel('Time (s)');
ylabel('Distance (m)');
grid on;

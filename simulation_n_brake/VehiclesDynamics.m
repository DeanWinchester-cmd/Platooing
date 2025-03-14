classdef VehiclesDynamics < handle
    properties
        % Vehicle Parameters
        mass (1,1) double {mustBePositive} = 1500;  % Vehicle mass (kg)
        drag_coeff (1,1) double = 0.3;  % Aerodynamic drag coefficient
        rolling_resist (1,1) double = 0.01;  % Rolling resistance coefficient   
        front_area (1,1) double = 2.2; %Frontal area of vehicle (m^2)
        d_air (1,1) double = 1.225; %Air density (kg/m^3)
        
        % State Variables
        position (1,1) double = 0;  % Initial position (m)
        velocity (1,1) double = 0;  % Initial velocity (m/s)
        acceleration (1,1) double = 0;  % Initial acceleration (m/s^2)
        
        % Simulation Parameters
        dt (1,1) double = 0.01;  % Time step (s)
    end
    
    methods
        % Constructor
        function obj = VehiclesDynamics(mass, position, velocity, acceleration)
            if nargin > 0
                % Allow custom initialization
                if nargin >= 1, obj.mass = mass; end
                if nargin >= 2, obj.position = position; end
                if nargin >= 3, obj.velocity = velocity; end
                if nargin >= 4, obj.acceleration = acceleration; end
            end
        end
        
        % Compute External Forces
        function F_ext = compute_external_forces(obj)
            % Air resistance
            F_drag = 0.5*obj.d_air * obj.drag_coeff * obj.front_area * obj.velocity^2;  % Find out which Air Density
            
            % Rolling resistance
            F_rolling = obj.rolling_resist * obj.mass * 9.81;
            
            % Total external forces
            F_ext = F_drag + F_rolling;
        end
        
        % Vehicle Dynamics Update
        function update(obj, control_input)
            % Compute external forces
            F_ext = obj.compute_external_forces(obj.velocity);
            
            % Compute net acceleration (F = ma)
            net_acceleration = (control_input - F_ext) / obj.mass;
            
            % Update state variables
            obj.acceleration = net_acceleration;
            obj.velocity = obj.velocity + obj.acceleration * obj.dt;
            obj.position = obj.position + obj.velocity * obj.dt;
        end
        
        % Simulation Runner
        function [t, pos, vel, acc] = simulate(obj, total_time, control_profile)
            % Prepare simulation arrays
            steps = floor(total_time / obj.dt);
            t = zeros(1, steps);
            pos = zeros(1, steps);
            vel = zeros(1, steps);
            acc = zeros(1, steps);
            
            % Simulation loop
            for i = 1:steps
                % Time tracking
                t(i) = (i-1) * obj.dt;
                
                % Apply control input from profile (cycle if necessary)
                if nargin > 2
                    control_input = control_profile(mod(i-1, length(control_profile)) + 1);
                else
                    control_input = 0;  % Default to zero if no profile
                end
                
                % Update vehicle dynamics
                obj.update(control_input);
                
                % Store current state
                pos(i) = obj.position;
                vel(i) = obj.velocity;
                acc(i) = obj.acceleration;
            end
        end
        
        % Plotting Method
        % function plot_results(obj, t, pos, vel, acc)
        %     figure;
        % 
        %     % Position subplot
        %     subplot(3,1,1);
        %     plot(t, pos);
        %     title('Position vs Time');
        %     ylabel('Position (m)');
        % 
        %     % Velocity subplot
        %     subplot(3,1,2);
        %     plot(t, vel);
        %     title('Velocity vs Time');
        %     ylabel('Velocity (m/s)');
        % 
        %     % Acceleration subplot
        %     subplot(3,1,3);
        %     plot(t, acc);
        %     title('Acceleration vs Time');
        %     ylabel('Acceleration (m/s^2)');
        % 
        %     % Common x-axis
        %     xlabel('Time (s)');
        %     grid on;  % Optional: turn on the grid for better visualization
        % end
    end
end

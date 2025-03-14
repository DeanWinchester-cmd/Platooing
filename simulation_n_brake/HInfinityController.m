classdef HInfinityController < handle
    properties
        % Controller Parameters
        h (1,1) double = 1.0;          % Time headway (s)
        desired_distance (1,1) double = 10;  % Standstill distance (m)
        
        % H-infinity controller gains (precomputed)
        K_1 (1,1) double = 0.8;        % Position error gain
        K_2 (1,1) double = 1.2;        % Velocity error gain
        K_3 (1,1) double = 0.5;        % Acceleration feedforward gain
        
        % Disturbance attenuation level
        gamma (1,1) double = 0.5;      % H-infinity performance bound
    end
    
    methods
        % Constructor
        function obj = HInfinityController(h, desired_distance, K_1, K_2, K_3, gamma)
            if nargin > 0
                if nargin >= 1, obj.h = h; end
                if nargin >= 2, obj.desired_distance = desired_distance; end
                if nargin >= 3, obj.K_1 = K_1; end
                if nargin >= 4, obj.K_2 = K_2; end
                if nargin >= 5, obj.K_3 = K_3; end
                if nargin >= 6, obj.gamma = gamma; end
            end
        end
        
        % Calculate control input
        function u = calculate_control(obj, ego_pos, ego_vel, ego_acc, leader_pos, leader_vel, leader_acc)
            % Calculate desired spacing
            desired_spacing = obj.desired_distance + obj.h * ego_vel;
            
            % Calculate actual spacing
            actual_spacing = leader_pos - ego_pos;
            
            % Calculate spacing error
            spacing_error = actual_spacing - desired_spacing;
            
            % Calculate velocity error
            velocity_error = leader_vel - ego_vel;
            
            % H-infinity control law
            % Note: In practice, the H-infinity controller would be designed
            % by solving Riccati equations or using specialized toolboxes
            u = obj.K_1 * spacing_error + obj.K_2 * velocity_error + obj.K_3 * leader_acc;
        end
    end
end

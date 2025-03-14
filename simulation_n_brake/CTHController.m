classdef CTHController < handle
    properties
        % Controller Parameters
        h (1,1) double = 1.5;         % Time headway (s)
        k_p (1,1) double = 0.5;       % Proportional gain
        k_d (1,1) double = 0.2;       % Derivative gain
        desired_distance (1,1) double = 10;  % Standstill distance (m)
    end
    
    methods
        % Constructor
        function obj = CTHController(h, k_p, k_d, desired_distance)
            if nargin > 0
                if nargin >= 1, obj.h = h; end
                if nargin >= 2, obj.k_p = k_p; end
                if nargin >= 3, obj.k_d = k_d; end
                if nargin >= 4, obj.desired_distance = desired_distance; end
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

            %debugging
            % fprintf('h=%.2f, k_p=%.2f, k_d=%.2f, desired_distance=%.2f\n', ...
            % obj.h, obj.k_p, obj.k_d, obj.desired_distance);
            % 
            % CTH control law
            u = obj.k_p * spacing_error + obj.k_d * velocity_error + leader_acc;
            
        end
    end
end

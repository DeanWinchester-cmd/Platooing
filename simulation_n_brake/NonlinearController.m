classdef NonlinearController < handle
    properties
        % Controller Parameters
        h (1,1) double = 1.0;          % Time headway (s)
        desired_distance (1,1) double = 10;  % Standstill distance (m)
        
        % Nonlinear control parameters
        k1 (1,1) double = 0.5;         % Position error gain
        k2 (1,1) double = 0.8;         % Velocity error gain
        k3 (1,1) double = 0.3;         % Nonlinear damping term
        
        % Saturation limits
        max_accel (1,1) double = 2.0;  % Maximum acceleration (m/s^2)
        max_decel (1,1) double = 3.0;  % Maximum deceleration (m/s^2)
    end
    
    methods
        % Constructor
        function obj = NonlinearController(h, desired_distance, k1, k2, k3, max_accel, max_decel)
            if nargin > 0
                if nargin >= 1, obj.h = h; end
                if nargin >= 2, obj.desired_distance = desired_distance; end
                if nargin >= 3, obj.k1 = k1; end
                if nargin >= 4, obj.k2 = k2; end
                if nargin >= 5, obj.k3 = k3; end
                if nargin >= 6, obj.max_accel = max_accel; end
                if nargin >= 7, obj.max_decel = max_decel; end
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
            
            % Nonlinear control law
            % Sigmoid function for spacing error
            sigmoid_spacing = tanh(spacing_error);
            
            % Cubic function for velocity damping
            cubic_velocity = velocity_error^3;
            
            % Nonlinear control law
            u = obj.k1 * sigmoid_spacing + obj.k2 * velocity_error + obj.k3 * cubic_velocity + leader_acc;
            
            % Apply saturation
            if u > obj.max_accel
                u = obj.max_accel;
            elseif u < -obj.max_decel
                u = -obj.max_decel;
            end
        end
    end
end

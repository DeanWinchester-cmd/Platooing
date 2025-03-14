classdef PIDController < handle
    properties
        % Controller Parameters
        h (1,1) double = 1.2;          % Time headway (s)
        desired_distance (1,1) double = 10;  % Standstill distance (m)
        
        % PID gains
        k_p (1,1) double = 0.6;        % Proportional gain
        k_i (1,1) double = 0.1;        % Integral gain
        k_d (1,1) double = 0.3;        % Derivative gain
        
        % Internal variables
        integral_error (1,1) double = 0;  % Accumulated error
        prev_error (1,1) double = 0;      % Previous error
        dt (1,1) double = 0.1;            % Time step (s)
    end
    
    methods
        % Constructor
        function obj = PIDController(h, desired_distance, k_p, k_i, k_d, dt)
            if nargin > 0
                if nargin >= 1, obj.h = h; end
                if nargin >= 2, obj.desired_distance = desired_distance; end
                if nargin >= 3, obj.k_p = k_p; end
                if nargin >= 4, obj.k_i = k_i; end
                if nargin >= 5, obj.k_d = k_d; end
                if nargin >= 6, obj.dt = dt; end
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
            
            % Update integral term with anti-windup
            obj.integral_error = obj.integral_error + spacing_error * obj.dt;
            
            % Calculate derivative term
            derivative_error = (spacing_error - obj.prev_error) / obj.dt;
            obj.prev_error = spacing_error;
            
            % PID control law with feed-forward term
            u = obj.k_p * spacing_error + obj.k_i * obj.integral_error + obj.k_d * derivative_error + leader_acc;
        end
        
        % Reset controller
        function reset(obj)
            obj.integral_error = 0;
            obj.prev_error = 0;
        end
    end
end

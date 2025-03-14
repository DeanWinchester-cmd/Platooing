classdef SMCController < handle
    properties
        % Controller Parameters
        h (1,1) double = 1.0;          % Time headway (s)
        desired_distance (1,1) double = 10;  % Standstill distance (m)
        lambda (1,1) double = 2.0;     % Sliding surface parameter
        eta (1,1) double = 0.5;        % Reaching law parameter
        phi (1,1) double = 0.1;        % Boundary layer thickness
    end
    
    methods
        % Constructor
        function obj = SMCController(h, desired_distance, lambda, eta, phi)
            if nargin > 0
                if nargin >= 1, obj.h = h; end
                if nargin >= 2, obj.desired_distance = desired_distance; end
                if nargin >= 3, obj.lambda = lambda; end
                if nargin >= 4, obj.eta = eta; end
                if nargin >= 5, obj.phi = phi; end
            end
        end
        
        % Calculate control input
        function u = calculate_control(obj, ego_pos, ego_vel, ego_acc, leader_pos, leader_vel, leader_acc)
            % Calculate desired spacing
            desired_spacing = obj.desired_distance + obj.h * ego_vel;
            
            % Calculate actual spacing
            actual_spacing = leader_pos - ego_pos;
            
            % Calculate spacing error and its derivative
            spacing_error = actual_spacing - desired_spacing;
            spacing_error_dot = leader_vel - ego_vel - obj.h * ego_acc;
            
            % Define sliding surface
            s = spacing_error_dot + obj.lambda * spacing_error;
            
            % Saturation function to avoid chattering
            sat_s = min(max(s/obj.phi, -1), 1);
            
            % SMC control law with boundary layer
            u = leader_acc + obj.lambda * (leader_vel - ego_vel - obj.h * ego_acc) + obj.eta * sat_s;
        end
    end
end

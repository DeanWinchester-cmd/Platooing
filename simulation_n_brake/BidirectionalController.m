classdef BidirectionalController < handle
    properties
        % Controller Parameters
        h_prev (1,1) double = 1.0;     % Time headway for predecessor (s)
        h_next (1,1) double = 0.8;     % Time headway for follower (s)
        desired_distance (1,1) double = 10;  % Standstill distance (m)
        
        % Control gains
        k_prev (1,1) double = 0.6;     % Weight for predecessor errors
        k_next (1,1) double = 0.4;     % Weight for follower errors
        k_v (1,1) double = 0.5;        % Velocity error gain
    end
    
    methods
        % Constructor
        function obj = BidirectionalController(h_prev, h_next, desired_distance, k_prev, k_next, k_v)
            if nargin > 0
                if nargin >= 1, obj.h_prev = h_prev; end
                if nargin >= 2, obj.h_next = h_next; end
                if nargin >= 3, obj.desired_distance = desired_distance; end
                if nargin >= 4, obj.k_prev = k_prev; end
                if nargin >= 5, obj.k_next = k_next; end
                if nargin >= 6, obj.k_v = k_v; end
            end
        end
        
        % Calculate control input
        function u = calculate_control(obj, ego_pos, ego_vel, ego_acc, prev_pos, prev_vel, prev_acc, next_pos, next_vel, next_acc)
            % Calculate desired spacing with predecessor
            desired_spacing_prev = obj.desired_distance + obj.h_prev * ego_vel;
            
            % Calculate actual spacing with predecessor
            actual_spacing_prev = prev_pos - ego_pos;
            
            % Calculate spacing error with predecessor
            spacing_error_prev = actual_spacing_prev - desired_spacing_prev;
            
            % Calculate desired spacing with follower
            desired_spacing_next = obj.desired_distance + obj.h_next * next_vel;
            
            % Calculate actual spacing with follower
            actual_spacing_next = ego_pos - next_pos;
            
            % Calculate spacing error with follower
            spacing_error_next = actual_spacing_next - desired_spacing_next;
            
            % Calculate velocity errors
            vel_error_prev = prev_vel - ego_vel;
            vel_error_next = ego_vel - next_vel;
            
            % Bidirectional control law
            u_prev = obj.k_prev * spacing_error_prev;
            u_next = obj.k_next * spacing_error_next;
            u_vel = obj.k_v * (vel_error_prev - vel_error_next);
            
            % Complete control law
            u = u_prev + u_next + u_vel;
        end
    end
end

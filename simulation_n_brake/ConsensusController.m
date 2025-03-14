classdef ConsensusController < handle
    properties
        % Controller Parameters
        h (1,1) double = 1.0;          % Time headway (s)
        desired_distance (1,1) double = 10;  % Standstill distance (m)
        
        % Consensus gains
        a_prev (1,1) double = 0.5;     % Weight for predecessor vehicle
        a_next (1,1) double = 0.3;     % Weight for follower vehicle
        a_lead (1,1) double = 0.2;     % Weight for leader vehicle
        
        % Control gains
        k_p (1,1) double = 0.5;        % Proportional gain
        k_d (1,1) double = 0.3;        % Derivative gain
        
        % Communication topology
        is_leader (1,1) logical = false; % Is this the lead vehicle?
        has_leader_info (1,1) logical = true; % Does this vehicle have leader information?
    end
    
    methods
        % Constructor
        function obj = ConsensusController(h, desired_distance, a_prev, a_next, a_lead, k_p, k_d, is_leader, has_leader_info)
            if nargin > 0
                if nargin >= 1, obj.h = h; end
                if nargin >= 2, obj.desired_distance = desired_distance; end
                if nargin >= 3, obj.a_prev = a_prev; end
                if nargin >= 4, obj.a_next = a_next; end
                if nargin >= 5, obj.a_lead = a_lead; end
                if nargin >= 6, obj.k_p = k_p; end
                if nargin >= 7, obj.k_d = k_d; end
                if nargin >= 8, obj.is_leader = is_leader; end
                if nargin >= 9, obj.has_leader_info = has_leader_info; end
            end
        end
        
        % Calculate control input
        function u = calculate_control(obj, ego_pos, ego_vel, ego_acc, ...
                                      prev_pos, prev_vel, prev_acc, ...
                                      next_pos, next_vel, next_acc, ...
                                      lead_pos, lead_vel, lead_acc)
            
            % If this is the lead vehicle, follow a predefined trajectory
            if obj.is_leader
                u = 0;  % Or some other predefined input
                return;
            end
            
            % Calculate desired spacing with predecessor
            desired_spacing_prev = obj.desired_distance + obj.h * ego_vel;
            
            % Calculate actual spacing with predecessor
            actual_spacing_prev = prev_pos - ego_pos;
            
            % Calculate spacing error with predecessor
            spacing_error_prev = actual_spacing_prev - desired_spacing_prev;
            
            % Calculate velocity errors
            vel_error_prev = prev_vel - ego_vel;
            vel_error_next = ego_vel - next_vel;
            
            % Calculate consensus acceleration
            cons_acc = obj.a_prev * prev_acc;
            
            % Add follower information if available
            if nargin >= 12
                cons_acc = cons_acc + obj.a_next * next_acc;
            end
            
            % Add leader information if available
            if obj.has_leader_info && nargin >= 15
                % Calculate desired spacing with leader
                desired_spacing_lead = obj.desired_distance * 2 + obj.h * ego_vel;  % Approximate for 2 vehicles ahead
                
                % Calculate actual spacing with leader
                actual_spacing_lead = lead_pos - ego_pos;
                
                % Calculate spacing error with leader
                spacing_error_lead = actual_spacing_lead - desired_spacing_lead;
                
                % Calculate velocity error with leader
                vel_error_lead = lead_vel - ego_vel;
                
                % Add leader contribution to consensus
                cons_acc = cons_acc + obj.a_lead * lead_acc;
                
                % Add leader-based control term
                u_lead = obj.k_p * spacing_error_lead + obj.k_d * vel_error_lead;
            else
                u_lead = 0;
            end
            
            % Predecessor-based control term
            u_prev = obj.k_p * spacing_error_prev + obj.k_d * vel_error_prev;
            
            % Complete control law
            u = u_prev + u_lead + cons_acc;
        end
    end
end

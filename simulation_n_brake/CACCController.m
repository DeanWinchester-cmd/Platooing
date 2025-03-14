classdef CACCController < handle
    properties
        % Controller Parameters
        h (1,1) double = 0.7;          % Time headway (s) - can be smaller with V2V communication
        desired_distance (1,1) double = 5;  % Standstill distance (m)
        
        % CACC gains
        k_p (1,1) double = 0.45;       % Proportional gain
        k_d (1,1) double = 0.25;       % Derivative gain
        
        % Feedforward gain
        alpha (1,1) double = 0.8;      % Weight for leader acceleration
        beta (1,1) double = 0.2;       % Weight for predecessor acceleration
        
        % Communication delay
        comm_delay (1,1) double = 0.05; % V2V communication delay (s)
        
        % Internal buffers for delay handling
        acc_buffer = [];               % Buffer for delayed acceleration values
    end
    
    methods
        % Constructor
        function obj = CACCController(h, desired_distance, k_p, k_d, alpha, beta, comm_delay)
            if nargin > 0
                if nargin >= 1, obj.h = h; end
                if nargin >= 2, obj.desired_distance = desired_distance; end
                if nargin >= 3, obj.k_p = k_p; end
                if nargin >= 4, obj.k_d = k_d; end
                if nargin >= 5, obj.alpha = alpha; end
                if nargin >= 6, obj.beta = beta; end
                if nargin >= 7, obj.comm_delay = comm_delay; end
            end
        end
        
        % Calculate control input
        function u = calculate_control(obj, ego_pos, ego_vel, ego_acc, pred_pos, pred_vel, pred_acc, lead_pos, lead_vel, lead_acc, dt)
            % Store acceleration for delay handling
            obj.acc_buffer = [obj.acc_buffer; pred_acc lead_acc];
            
            % Apply communication delay (simplified)
            if size(obj.acc_buffer, 1) > round(obj.comm_delay/dt)
                delayed_acc = obj.acc_buffer(1, :);
                obj.acc_buffer = obj.acc_buffer(2:end, :);
            else
                delayed_acc = [0, 0];  % Default values before buffer is filled
            end
            
            delayed_pred_acc = delayed_acc(1);
            delayed_lead_acc = delayed_acc(2);
            
            % Calculate desired spacing
            desired_spacing = obj.desired_distance + obj.h * ego_vel;
            
            % Calculate actual spacing with predecessor
            actual_spacing = pred_pos - ego_pos;
            
            % Calculate spacing error
            spacing_error = actual_spacing - desired_spacing;
            
            % Calculate velocity error
            velocity_error = pred_vel - ego_vel;
            
            % CACC control law with V2V communication
            u_fb = obj.k_p * spacing_error + obj.k_d * velocity_error;                 % Feedback term
            u_ff = obj.alpha * delayed_lead_acc + obj.beta * delayed_pred_acc;         % Feedforward term
            
            u = u_fb + u_ff;
        end
        
        % Reset controller
        function reset(obj)
            obj.acc_buffer = [];
        end
    end
end

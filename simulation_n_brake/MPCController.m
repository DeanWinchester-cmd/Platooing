classdef MPCController < handle
    properties
        % Controller Parameters
        h (1,1) double = 1.0;          % Time headway (s)
        desired_distance (1,1) double = 10;  % Standstill distance (m)
        
        % MPC Parameters
        Np (1,1) double = 10;          % Prediction horizon
        Nc (1,1) double = 5;           % Control horizon
        Q (1,1) double = 10;           % Weight for spacing error
        R (1,1) double = 1;            % Weight for control input
        
        % Constraints
        u_min (1,1) double = -3;       % Minimum acceleration (m/s^2)
        u_max (1,1) double = 2;        % Maximum acceleration (m/s^2)
        
        % System model matrices
        A = [1 1 0; 0 1 1; 0 0 1];     % System matrix (position, velocity, acceleration)
        B = [0; 0; 1];                 % Input matrix
        C = [1 0 0; 0 1 0];            % Output matrix (position, velocity)
        
        % Sampling time
        dt (1,1) double = 0.1;         % Sampling time (s)
    end
    
    methods
        % Constructor
        function obj = MPCController(h, desired_distance, Np, Nc, Q, R, u_min, u_max, dt)
            if nargin > 0
                if nargin >= 1, obj.h = h; end
                if nargin >= 2, obj.desired_distance = desired_distance; end
                if nargin >= 3, obj.Np = Np; end
                if nargin >= 4, obj.Nc = Nc; end
                if nargin >= 5, obj.Q = Q; end
                if nargin >= 6, obj.R = R; end
                if nargin >= 7, obj.u_min = u_min; end
                if nargin >= 8, obj.u_max = u_max; end
                if nargin >= 9, obj.dt = dt; end
            end
            
            % Update system matrices based on sampling time
            obj.A = [1 obj.dt 0; 0 1 obj.dt; 0 0 1];
            obj.B = [0; 0; 1] * obj.dt;
        end
        
        % Calculate control input
        function u = calculate_control(obj, ego_pos, ego_vel, ego_acc, leader_pos, leader_vel, leader_acc)
            % Create state vectors
            x = [ego_pos; ego_vel; ego_acc];
            
            % Reference trajectory (leader predicted positions)
            ref = zeros(2, obj.Np);
            leader_state = [leader_pos; leader_vel; leader_acc];
            
            % Simple prediction of leader trajectory
            for i = 1:obj.Np
                leader_state = obj.A * leader_state + obj.B * 0;  % Assume constant leader acceleration
                desired_spacing = obj.desired_distance + obj.h * leader_state(2);
                ref(1, i) = leader_state(1) - desired_spacing;
                ref(2, i) = leader_state(2);
            end
            
            % This is a simplified MPC implementation - in practice, you would use an MPC solver
            % Here we'll implement a basic unconstrained MPC solution
            
            % Formulate the optimization problem
            % Simplified implementation: compute one-step optimal control
            
            % For a real implementation, we would use:
            % [u_opt, ~, ~] = quadprog(H, f, A_ineq, b_ineq, [], [], u_min*ones(Nc,1), u_max*ones(Nc,1));
            
            % Simplified version (one-step ahead, no constraints)
            error = (obj.C * (obj.A * x)) - ref(:, 1);
            K = -inv(obj.R + obj.B' * obj.C' * obj.Q * obj.C * obj.B) * obj.B' * obj.C' * obj.Q;
            u = K * error;
            
            % Apply constraints
            u = min(max(u, obj.u_min), obj.u_max);
        end
    end
end

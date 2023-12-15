classdef MpcControl_z < MpcControlBase
    properties
        A_bar, B_bar, C_bar % Augmented system for disturbance rejection
        L                   % Estimator gain for disturbance rejection
    end
    
    methods
        function mpc = MpcControl_z(sys, Ts, H)
            mpc = mpc@MpcControlBase(sys, Ts, H);
            
            [mpc.A_bar, mpc.B_bar, mpc.C_bar, mpc.L] = mpc.setup_estimator();
        end
        
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc, Ts, H)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   X(:,1)       - initial state (estimate)
            %   d_est        - disturbance estimate
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   U(:,1)       - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(H/Ts); % Horizon steps
            N = N_segs + 1;      % Last index in 1-based Matlab indexing
            
            [nx, nu] = size(mpc.B);
            
            % Targets (Ignore this before Todo 3.3)
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);
            
            % Disturbance estimate (Ignore this before Part 5)
            d_est = sdpvar(1);
            
            % Predicted state and input trajectories
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            obj = 0;
            con = [];
            
            % state constraints
            %none
    
            % input constraints
            Us = 56.6666665401736;  % Steady state input
            M = [1; -1];
            m = [80 - Us; -(50 - Us)];
            
            % matrices
            Q = 50*eye(2);

            R = 0.1;
            sys = LTISystem('A',mpc.A,'B',mpc.B);

            sys.x.max = [Inf;Inf];
            sys.x.min = [-Inf;-Inf];
            sys.u.min = [50];
            sys.u.max = [80];
            sys.x.penalty = QuadFunction(Q);
            sys.u.penalty = QuadFunction(R);

            Qf = sys.LQRPenalty.weight;
            Xf = sys.LQRSet;
            %[~, Qf, ~] = dlqr(mpc.A, mpc.B, Q, R, H);
            
            Ff = double(Xf.A);
            ff = double(Xf.b);

            obj = 0;
            con = [];

            for i = 1:N-1
                con = [con, (X(:,i+1)) == mpc.A*(X(:,i)) + mpc.B*(U(:,i))]; % System dynamics
                
                con = [con, M*U(:,i) <= m]; % Input constraints
                obj = obj + (X(:,i+1)-x_ref)'*Q*(X(:,i+1)-x_ref) + (U(:,i)-u_ref)'*R*(U(:,i)-u_ref); % Cost function
            end
            con = [con, Ff*(X(:,N)-x_ref) <= ff]; % Terminal constraint
            obj = obj + (X(:,N)-x_ref)'*Qf*(X(:,N)-x_ref); % Terminal weight
            
            %plot(Xf)

            %title('Projection of terminal set on 1st and 2nd dimensions')
            %Xf.projection(1:2).plot();
            
            %title('Projection of terminal set on 2nd and 3rd dimensions')
            %Xf.projection(2:3).plot();
            
            %title('Projection of terminal set on 3rd and 4th dimensions')
            %Xf.projection(3:4).plot();
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {X(:,1), x_ref, u_ref, d_est}, {U(:,1), X, U});
        end
        
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opti = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            %   d_est  - disturbance estimate
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            nx = size(mpc.A, 1);
            
            % Steady-state targets
            xs = sdpvar(nx, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.3)
            ref = sdpvar;
            
            % Disturbance estimate (Ignore this before Part 5)
            d_est = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            obj = 0;
            con = [xs == 0, us == 0];

            Sigma = [eye(nx)-mpc.A, -mpc.B; mpc.C, zeros(size(mpc.C,1), size(mpc.B,2))];

           

            Q_sigma = 0.01*eye(2);

            R_sigma = 1;

            B_Sigma = [zeros(nx,1); ref];
            
            obj = us'*R_sigma*us;
            
             % input constraints
            Us = 56.6666665401736;  % Steady state input
            M = [1; -1];
            m = [80 - Us; -(50 - Us)];
            
            con = [Sigma*[xs;us]==B_Sigma,
                           
                           M*us<= m];
            diagnostics = solvesdp(con,obj,sdpsettings('verbose',0));
            double(xs)
            
            if diagnostics.problem ~= 0
                % no solution exists: compute reachable set point that is
                % closest to ref
                obj = (mpc.C*xs - ref)'*Q_sigma*(mpc.C*xs - ref);
                con = [xs == mpc.A*xs + mpc.B*us,
                          
                           M*us<= m];
                solvesdp(con,obj,sdpsettings('verbose',0));
            end
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), {ref, d_est}, {xs, us});
        end
        
        
        % Compute augmented system and estimator gain for input disturbance rejection
        function [A_bar, B_bar, C_bar, L] = setup_estimator(mpc)
            
            %%% Design the matrices A_bar, B_bar, L, and C_bar
            %%% so that the estimate x_bar_next [ x_hat; disturbance_hat ]
            %%% converges to the correct state and constant input disturbance
            %%%   x_bar_next = A_bar * x_bar + B_bar * u + L * (C_bar * x_bar - y);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            
            A_bar = [];
            B_bar = [];
            C_bar = [];
            L = [];
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        
    end
end
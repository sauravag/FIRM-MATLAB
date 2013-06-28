classdef MotionModel_interface
    properties (Abstract, Constant) % note that all properties must be constant, because we have a lot of copies of this object and it can take a lot of memory otherwise.
        stDim; % state dimension
        ctDim;  % control vector dimension
        wDim;   % Process noise (W) dimension
        dt; % delta_t for time discretization
        sigma_b_u; % A constant bias intensity (covariance) of the control noise
        eta_u; % A coefficient, which makes the control noise intensity proportional to the control signal
        P_Wg; % covariance of state-additive-noise
    end
    
    methods (Abstract, Static)
        x_next = f_discrete(x,u,w) % discrete motion model equation
        A = df_dx_func(x,u,w) % state Jacobian
        B = df_du_func(x,u,w) % control Jacobian
        G = df_dw_func(x,u,w) % nosie Jacobian
        Q_process_noise = process_noise_cov(x,u) % compute the covariance of process noise based on the current poistion and controls
        w = generate_process_noise(x,u) % simulate (generate) process noise based on the current poistion and controls
        
        nominal_traj = generate_open_loop_point2point_traj(X_initial,X_final) % generates open-loop trajectories between two start and goal states
%         nominal_traj = generate_VALID_open_loop_point2point_traj(X_initial,X_final) % generates open-loop trajectories between two start and goal states
        YesNo = is_constraints_violated(open_loop_traj)
        traj_plot_handle = draw_nominal_traj(nominal_traj, varargin)
    end
end
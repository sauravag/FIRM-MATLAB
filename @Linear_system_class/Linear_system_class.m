classdef Linear_system_class < handle
    %LINEAR_SYSTEM_CLASS encapsulates a linear system class.
    %   Detailed explanation goes here
    
    properties
        lnr_pts  % lnr_pts or "linear points" is an structure that has four fields: x,u,w,v that determines the linearization point.
        A = [];
        B = [];
        G = [];
        Q = [];
        H = [];
        M = [];
        R = [];
        Kalman_gain = []; % Kalman Gain for this linear system.
    end
    
    methods (Static = true)
        function [A,B,G,Q,H,M,R] = all_matrices(lnr_pts)
            x = lnr_pts.x;
            u = lnr_pts.u;
            if ~isfield(lnr_pts,'w')
                w = zeros(MotionModel_class.wDim,1);
            else
                w = lnr_pts.w;
            end
            if ~isfield(lnr_pts,'v')
                v = zeros(ObservationModel_class.obsNoiseDim,1);
            else
                v = lnr_pts.v;
            end
            A = MotionModel_class.df_dx_func(x,u,w);
            B = MotionModel_class.df_du_func(x,u,w);
            G = MotionModel_class.df_dw_func(x,u,w);
            Q = MotionModel_class.process_noise_cov(x,u);
            H = ObservationModel_class.dh_dx_func(x,v);
            M = ObservationModel_class.dh_dv_func(x,v);
            R = ObservationModel_class.noise_covariance(x);
        end
    end
    
    methods
        function obj = Linear_system_class(lnr_pts_inp)
            obj.lnr_pts = lnr_pts_inp;
            if ~isfield(obj.lnr_pts,'w')
                obj.lnr_pts.w = zeros(MotionModel_class.wDim,1);
            end
            if ~isfield(obj.lnr_pts,'v')
                obj.lnr_pts.v = zeros(ObservationModel_class.obsNoiseDim,1);
            end
        end
        function A_val = get.A(obj)
            if isempty(obj.A) % The first time this property is needed, it is gonna be computed here. But after that this property will not be recomputed again.
                x = obj.lnr_pts.x;
                u = obj.lnr_pts.u;
                w = obj.lnr_pts.w;
                A_val = MotionModel_class.df_dx_func(x,u,w);
                obj.A = A_val; % Since this is a handle class, obj is changed in this line without listing it as an output argument.
            else
                A_val = obj.A;
            end
        end
        function B_val = get.B(obj)
            if isempty(obj.B) % The first time this property is needed, it is gonna be computed here. But after that this property will not be recomputed again.
                x = obj.lnr_pts.x;
                u = obj.lnr_pts.u;
                w = obj.lnr_pts.w;
                B_val = MotionModel_class.df_du_func(x,u,w);
                obj.B = B_val; % Since this is a handle class, obj is changed in this line without listing it as an output argument.
            else
                B_val = obj.B;
            end
        end
        function G_val = get.G(obj)
            if isempty(obj.G) % The first time this property is needed, it is gonna be computed here. But after that this property will not be recomputed again.
                x = obj.lnr_pts.x;
                u = obj.lnr_pts.u;
                w = obj.lnr_pts.w;
                G_val = MotionModel_class.df_dw_func(x,u,w);
                obj.G = G_val; % Since this is a handle class, obj is changed in this line without listing it as an output argument.
            else
                G_val = obj.G;
            end
        end
        function Q_val = get.Q(obj)
            if isempty(obj.Q) % The first time this property is needed, it is gonna be computed here. But after that this property will not be recomputed again.
                x = obj.lnr_pts.x;
                u = obj.lnr_pts.u;
                Q_val = MotionModel_class.process_noise_cov(x,u);
                obj.Q = Q_val; % Since this is a handle class, obj is changed in this line without listing it as an output argument.
            else
                Q_val = obj.Q;
            end
        end
        function H_val = get.H(obj)
            if isempty(obj.H) % The first time this property is needed, it is gonna be computed here. But after that this property will not be recomputed again.
                x = obj.lnr_pts.x;
                v = obj.lnr_pts.v;
                H_val = ObservationModel_class.dh_dx_func(x,v);
                obj.H = H_val; % Since this is a handle class, obj is changed in this line without listing it as an output argument.
            else
                H_val = obj.H;
            end
        end
        function M_val = get.M(obj)
            if isempty(obj.M) % The first time this property is needed, it is gonna be computed here. But after that this property will not be recomputed again.
                x = obj.lnr_pts.x;
                v = obj.lnr_pts.v;
                M_val = ObservationModel_class.dh_dv_func(x,v);
                obj.M = M_val; % Since this is a handle class, obj is changed in this line without listing it as an output argument.
            else
                M_val = obj.M;
            end
        end
        function R_val = get.R(obj)
            if isempty(obj.R) % The first time this property is needed, it is gonna be computed here. But after that this property will not be recomputed again.
                x = obj.lnr_pts.x;
                R_val = ObservationModel_class.noise_covariance(x);
                obj.R = R_val; % Since this is a handle class, obj is changed in this line without listing it as an output argument.
            else
                R_val = obj.R;
            end
        end
%         function KalGain_val = get.Kalman_gain(obj)
%             if isempty(obj.Kalman_gain)
                
    end
end

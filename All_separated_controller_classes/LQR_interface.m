classdef LQR_interface < separated_controller_interface
    %LQR_class encapsulates the LQR controller.
    
    properties
        Feedback_gains; % feedback gain (either a single feedback gain/ periodic sequence/ or a finite number of gains  depending on the derived class)
        lnr_pts; % linearization points (either a single point/ periodic sequence/ or a finite number of pointa  depending on the derived class)
    end
    
    properties (Constant = true)
        state_cost = user_data_class.par.state_cost;
        control_cost = user_data_class.par.control_cost;
        Final_state_cost = user_data_class.par.Final_state_cost;
        valid_lnr_domain = user_data_class.par.valid_linearization_domain;
    end
    
    methods (Abstract)
        obj = generate_feedback_control(obj, varargin)
    end
    
end

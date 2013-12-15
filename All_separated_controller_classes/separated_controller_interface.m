classdef separated_controller_interface
    % This class encapsulates different variants of Kalman Filter.
    methods (Abstract)
        obj = generate_feedback_control(obj, varargin)
    end
end
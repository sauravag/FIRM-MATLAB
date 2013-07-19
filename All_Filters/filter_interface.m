classdef filter_interface
    % This class encapsulates different variants of Kalman Filter.
    methods (Abstract)
        obj = predict(obj, varargin)
        obj = update(obj, varargin)
        obj = filter(obj, varargin)
    end
end
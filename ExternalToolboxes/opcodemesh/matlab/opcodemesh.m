classdef opcodemesh < handle
    properties (SetAccess = private, Hidden = true)
        objectHandle; % Handle to the underlying C++ class instance
    end
    methods
        %% Constructor - Create a new C++ class instance 
        function this = opcodemesh(varargin)
            this.objectHandle = opcodemeshmex('create', varargin{:});
        end
        
        %% Destructor - Destroy the C++ class instance
        function delete(this)
            opcodemeshmex('delete', this.objectHandle);
        end

        function varargout = intersect(this, varargin)
            [varargout{1:nargout}] = opcodemeshmex('intersect', this.objectHandle, varargin{:});
        end
        
        function update(this, varargin)
            opcodemeshmex('update', this.objectHandle, varargin{:});
        end
    end
end

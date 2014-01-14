classdef Particle_belief
    % This class encapsulates the Paricle-based belief concept.
    properties
        num_particles;
        particles; 
        weights; 
        plot_handle;
    end

    methods
        function obj = draw(obj, varargin)
%                  try % Avoid errors if the graphic object has already been deleted
%                 delete(obj.ellipse_handle);
%             end
%             obj.ellipse_handle = [];
%         
        end
    end
    
    methods
        function obj = Particle_belief(num_particles_inp, particles_inp,weights_inp)
            obj.num_particles = num_particles_inp;
            obj.particles = particles_inp;
            obj.weights = wights_inp;
        end
        function obj = delete_plot(obj,varargin)
%             try % Avoid errors if the graphic object has already been deleted
%                 delete(obj.ellipse_handle);
%             end
%             obj.ellipse_handle = [];
%             obj.est_mean = obj.est_mean.delete_plot(varargin{:});
        end
        function obj = apply_differentiable_constraints(obj)
            % normally this function is empty. If the state has any
            % differentiable constraints (e.g., quaternion norm is one),
            % this function needs to be written specifically.
        end
    end

end
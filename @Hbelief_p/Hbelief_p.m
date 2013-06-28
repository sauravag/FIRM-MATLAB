classdef Hbelief_p % particle-based H-belief
    properties
        num_p;  % number of particles
        Hparticles;
        stopped_particles; % You cannot change this initial value.
        stopping_times;
        collided_particles; % You cannot change this initial value.
        unsuccessful_particles; % You cannot change this initial value.
        % not used properties
        stat_Hmean;
        stat_Hcov;
    end
    
    methods
        function obj = Hbelief_p(Hstate_particles, num_particles)
            if nargin == 2
                obj.num_p = num_particles;
            else % defualt value for number of particles
                obj.num_p = user_data_class.par.par_n;  % number of particles
            end
            obj.stopped_particles = zeros(obj.num_p,1); % You cannot change this initial value.
            obj.stopping_times = ones(obj.num_p,1)*(-1);
            obj.collided_particles = zeros(obj.num_p,1); % You cannot change this initial value.
            obj.unsuccessful_particles = zeros(obj.num_p,1); % You cannot change this initial value.
            if nargin >= 1
                % The input of structor is a collection of Hstate_particles in
                % a vector
                obj.Hparticles = Hstate_particles;
            end
        end
        function obj = draw(obj,varargin)
            if isempty(varargin)
                % defaule properties for drawing "Hbelief_p".
                varargin = {'robotshape','triangle','XgtriaColor','g','XestTriaColor','r','XgColor','g','XestColor','r'};
            end
            tmp = get(gca,'NextPlot'); hold on
            obj.Hparticles(1) = obj.Hparticles(1).draw(varargin{:});
            for q = 2:obj.num_p
                obj.Hparticles(q) = obj.Hparticles(q).draw(varargin{:});
            end
            set(gca,'NextPlot',tmp);
        end
        function obj = delete_plot(obj)
            for i = 1:obj.num_p
                obj.Hparticles(i) = obj.Hparticles(i).delete_plot();
            end
        end
        function obj = draw_CovOnNominal(obj, nominal_state, varargin)
            if isempty(varargin)
                % defaule properties for drawing "Hbelief_p".
                varargin = {'robotshape','triangle','XgtriaColor','g','XestTriaColor','r','XgColor','g','XestColor','r'};
            end
            tmp = get(gca,'NextPlot'); hold on
            if obj.collided_particles(1) ~= 1  % Note that if a particle is collided with an obstacle, we do not want to propagate its covariance along the nominal, while its mean has stopped already!!
                obj.Hparticles(1) = obj.Hparticles(1).draw_CovOnNominal(nominal_state,varargin{:});
            else % this means if the particle has already stopped or collided we do not drag its covariance center by the nominal state value.
                obj.Hparticles(1) = obj.Hparticles(1).draw(varargin{:});
            end
            for q = 2:obj.num_p
                if obj.collided_particles(q) ~= 1; % Note that if a particle is collided with an obstacle, we do not want to propagate its covariance along the nominal, while its mean has stopped already!!
                    obj.Hparticles(q) = obj.Hparticles(q).draw_CovOnNominal(nominal_state,varargin{:});
                else % this means if the particle has already stopped or collided we do not drag its covariance center by the nominal state value.
                    obj.Hparticles(q) = obj.Hparticles(q).draw(varargin{:});
                end
            end
            set(gca,'NextPlot',tmp);
        end
        function obj = collision_check(obj)
            for q = 1 : obj.num_p
                if obj.collided_particles(q) ~= 1; % we only check collision for the particles that have not been collided yet.
                    obj.collided_particles(q) = obj.Hparticles(q).Xg.is_constraint_violated();
                end
            end
        end
    end
end
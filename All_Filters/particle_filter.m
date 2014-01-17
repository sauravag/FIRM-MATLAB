classdef particle_filter < filter_interface
    % This class encapsulates different variants of Kalman Filter.
    methods 
        function b_next = estimate(obj, b, u, z)
            b_prd = obj.predict(b,u);
            b_next = obj.update(b_prd,z);
        end
        function b_prd = predict(obj, b, u)
            for m = 1:b.num_particles
                w = b.particles(m).generate_noise();
                b.particles(m) = b.particles(m).propagate(u,w);
            end
            b_prd = b;
        end
        function b = update(obj, b_prd, z)
            b = b_prd;
            for m = 1:b_prd.num_particles
                likelihood = b_prd.particles(m).ObservationLikelihood(z);
                b.weights(m) = b_prd.weights(m)*likelihood;
            end
            weightSum = sum(b.weights);
            for m = 1:b_prd.num_particles
                b.weights(m) = b.weights(m)/weightSum;
            end
        end
    end
end
classdef Stabilizer_interface
    %Stabilizer_interface is the base class for the stabilizers used in FIRM framework
    
    properties
        reachable_FIRM_nodes; % reachable FIRM nodes under the stabilizer
        stabilizer_number; % the stabilizer number among all FIRM stabilizers; % This is not initilized by the constructor, as it is not needed for the class really. It is just for "display" purposes. So, if it is needed it has to be explicitly set from the callling function.
        par; % parameters of the stabilizer
    end
    
    methods
        function obj = Stabilizer_interface() % constructor may or may not have "obj" as an input.
            % Constructor; just initializes the |reachable_FIRM_nodes|with empty objects.
            obj.reachable_FIRM_nodes = FIRM_node_class.empty;
            obj.stabilizer_number = 'Not provided yet';
        end
    end
    
    methods (Abstract)
        % constructs reachable nodes under this stabilizer
        obj = construct_reachable_FIRM_nodes(obj)        
        
        % for a given initial H-belief, returns a sequence of H-beliefs until reaching a FIRM node 
        [seq_of_PHb, seq_of_GHb, target_reaching_probabilities] = construct_seq_of_Hb(obj,initial_PHb, initial_GHb)        
        
        % returns the next Hstate, given the current Hstate. Also, returns
        % the landed node or the failure, if the run is unsuccessful. If it
        % is selected by the user, it considers replanning in case of large
        % deviations too.
        [next_Hstate, YesNo_unsuccessful, landed_node_ind] = execute(obj,current_Hstate,convergence_time)   
    end
    
end


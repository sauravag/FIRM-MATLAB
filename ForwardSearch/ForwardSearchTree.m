classdef ForwardSearchTree

    methods
        function BestPath = ConstructAndSearch(Depth, bs_init, bn_init)
            % inputs are the search depth, initial stabilizable belief
            % bs_init, and initial non-stabilizable belief bn_init
            
            % initial node initiazliation
            init_node.bs = bs_init;
            init_node.bn = bn_init;
            init_node.costToReach = 0;
            init_node.constraintToReach.stabilizable = 0;
            init_node.constraintToReach.Nonstabilizable = 0;
            init_node.path = [];
            % initializing teh OPEN set
            OpenNodes = init_node;
            for k = 1:Depth
                
                firstNode = OpenNodes(1)
            end
        end
    end
    
end
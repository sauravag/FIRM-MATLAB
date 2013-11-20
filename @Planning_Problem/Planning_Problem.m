classdef Planning_Problem
    %PLANNING_PROBLEM is a base class, from which one can instantiate a planning problem with a user inputed environment (obstacles and information sources)
    
    properties
        sim;
        PRM;
        FIRM_graph;
        par;
    end
    
    methods
        function obj = Planning_Problem(sim)
            % in constructor we retrive the paraemters of the planning
            % problem entered by the user.
            obj.par = user_data_class.par.planning_problem_param;
            obj.sim = sim;
            obj.PRM = PRM_class;
        end
        function obj = solve(obj)
            [loading_folder_path, ~, ~] = fileparts(user_data_class.par.LoadFileName); % This line returns the path of the folder from which we want to load the parameters.
            Constructed_FIRM_file = [loading_folder_path,filesep,'Constructed_FIRM.mat'];
            [saving_folder_path, ~, ~] = fileparts(user_data_class.par.SaveFileName); % This line returns the path of the folder into which we want to save the parameters.
            % --------------------------------------- Offline construction code
            if obj.par.Offline_construction_phase == 1  % Offline construction code
                
                if strcmpi(obj.par.solver,'Periodic LQG-based FIRM')
                    obj.FIRM_graph = PLQG_based_FIRM_graph_class(obj.PRM); % the input PRM is an object of PNPRM class
                else 
                    obj.FIRM_graph = FIRM_graph_class(obj.PRM); % the input PRM is an object of PNPRM class
                end                
                obj.FIRM_graph = obj.FIRM_graph.construct_all_stabilizers_and_FIRM_nodes(); % Here, we construct the set of stabilizers used in FIRM and then we construct the reachable nodes under those stabilizers.
                obj.FIRM_graph = obj.FIRM_graph.draw_all_nodes(); drawnow; % Draw the FIRM nodes
                obj.FIRM_graph = obj.FIRM_graph.construct_all_FIRM_edges(); % Construct all the FIRM edges and associated transition costs and probabilities.
                save([saving_folder_path, filesep,'Constructed_FIRM.mat'] , 'obj')  % Actually we are saving an object of "planning_problem" class, NOT only FIRM_graph. So, teh name of file, i.e., "Constructed_FIRM" can be a little misleading.
                
                % --------------------------------------- Online execution code
            elseif obj.par.Online_phase == 1  % Online execution code
                %                     load Data\FIRM17June
                %                     load Data\FIRM24June_3particles
                %                     load Data\FIRM_6July2011_OneParticle
                %                     load Data\FIRM_7July2011_OneParticle
                load(Constructed_FIRM_file); % Actually we are loading an object of "planning_problem" class, NOT only FIRM_graph. So, teh name of file, i.e., "Constructed_FIRM" can be a little misleading.
                if exist(Constructed_FIRM_file,'file')
                    copyfile(Constructed_FIRM_file,saving_folder_path)
                end
                obj.FIRM_graph = obj.FIRM_graph.draw_all_nodes(); drawnow
                
%                 myaa_Ali('FIRM_nodes_figure') % for producing the FIRM nodes figure for a paper
                
                start_node_ind = 6;
                goal_node_ind = 7;
             
		text_height = 0.5;
                text(obj.FIRM_graph.PRM.nodes(start_node_ind).val(1),obj.FIRM_graph.PRM.nodes(start_node_ind).val(2),obj.FIRM_graph.PRM.nodes(start_node_ind).val(3)+text_height,'start','color','r','fontsize',14); % we write "start" next to the start node
                text(obj.FIRM_graph.PRM.nodes(goal_node_ind).val(1),obj.FIRM_graph.PRM.nodes(goal_node_ind).val(2),obj.FIRM_graph.PRM.nodes(goal_node_ind).val(3)+text_height,'goal','color','r','fontsize',14); % we write "goal" next to the goal node
                
                obj.FIRM_graph = obj.FIRM_graph.DP_compute_cost_to_go_values(goal_node_ind);
%                 obj.FIRM_graph.feedback_pi(1)=2;
%                 obj.FIRM_graph.feedback_pi(5)=12;
%                 obj.FIRM_graph.feedback_pi(8)=17;
%                 obj.FIRM_graph.feedback_pi(1)=4;
%                 
                ensemble_size = 1;  % The execution phase only works for a single robot. If you need multiple realization, you have to re-run it multiple times.
                tmp_pHb = obj.FIRM_graph.Nodes(start_node_ind).sample(ensemble_size); % the "sample" function returns a particle-Hb, with a single particle (since "ensemble_size" is 1).
                initial_Hstate = tmp_pHb.Hparticles(1); % initialization % we retrive the single Hstate (or Hparticle) from the "tmp_pHb"
                
                obj.FIRM_graph = obj.FIRM_graph.Execute(initial_Hstate,start_node_ind,goal_node_ind);
                
            end
            if user_data_class.par.sim.video == 1;  close(vidObj);  end
        end
    end
    
    
    
end

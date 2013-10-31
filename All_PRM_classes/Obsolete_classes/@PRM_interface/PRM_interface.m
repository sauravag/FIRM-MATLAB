classdef PRM_interface
    %PRM_INTERFACE is a base class, from which different variants of PRM are derived.
    
    properties
        num_nodes = 0; % number of PRM nodes % initialization to zero is needed.
        num_stabilizers;
        num_orbits; % Added by Saurav.
        nodes; % PRM nodes
        edges;
        orbits;%added here by saurav
        edges_list; % list of PRM edges. It is an n-by-2 matrix, i-th row of which stores the starting and ending nodes of i-th edge.
        edges_matrix; % edges matrix. The (i,j)-th element is one if there exists an edge between nodes i and j. It is zero, otherwise.
        outgoing_edges;
        par; % PRM parameters entered by the user
    end
    
    % note that in Matlab, the private properties are not inherited to the subclasses. So, there is no point in writing these properties here but to remind you to have these properties in the subclasses in a consistent way.
    %     properties (Access = private) 
    %         edges_plot_handle;
    %         nodes2D_plot_handle;
    %         nodes_plot_handle;
    %         text_handle;
    %     end

    
    methods
        function obj = PRM_interface(obj) %#ok<INUSD>
            % the constructor; If an existing PRM is requested, it loads,
            % draws, and re-saves the PRM in the new output folder. And if
            % a new PRM is requested, it asks user to select the nodes and
            % then draws the PRM and saves it.
            if user_data_class.par.sim.interactive_PRM == 0
                obj = obj.load();
                obj = obj.overwrite_nodes();
                obj.par = user_data_class.par.PRM_parameters;  % IMPORTANT: note that you cannot put this command before the "load" command. Because, in that case the parameters of the loaded PRM will be considered, not the new parameters entered by the user.
                obj = obj.draw();
                obj = obj.save(); % In this case, the function "save" actually copies the previous PRM into the new output folder.
            else
                obj.par = user_data_class.par.PRM_parameters;
                obj = obj.request_nodes();
                obj = obj.overwrite_nodes();
                obj = obj.save();
            end
        end
    end
    
    methods %(Access = private)  % These have to be private! But the private methods are not inherited. So, they are set to be public.
        function old_prop = set_figure(obj) % This function sets the figure (size and other properties) to values that are needed for landmark selection or drawing.
            figure(gcf);
            old_prop{1}=get(gca,'NextPlot');hold on; % save the old "NextPlot" property and set it to "hold on" % Note that this procedure cannot be moved into the "set_figure" function.
            old_prop{2}=get(gca,'XGrid'); % save the old "XGrid" property.
            old_prop{3}=get(gca,'YGrid'); % save the old "YGrid" property.
            grid on; % set the XGrid and YGrid to "on".
            if ~isempty(user_data_class.par.sim.figure_position)
                set(gcf,'Position',user_data_class.par.sim.figure_position)
            end
            axis(user_data_class.par.sim.env_limits);
            set(gca,'DataAspectRatio',[1 1 1]); % makes the scaling of different axes the same. So, a circle is shown as a circle not ellipse.
        end
        function reset_figure(obj,old_prop) % This function resets the figure properties (size and other properties), to what they were before setting them in this class.
            set(gca,'NextPlot',old_prop{1}); % reset the "NextPlot" property to what it was.
            set(gca,'XGrid',old_prop{2}); % reset  the "XGrid" property to what it was.
            set(gca,'YGrid',old_prop{3}); % reset  the "YGrid" property to what it was.
        end
        function obj = add_set_of_nodes(obj, new_nodes)
            for i = 1:length(new_nodes)
                obj = obj.add_node(new_nodes(i));
            end
        end
    end
    methods (Abstract)
        obj = load(obj) % loads an existing PRM
        obj = draw(obj) % draws the PRM graph, "varargin" specifies drawing's "prop/val"s.
        obj = delete_plot(obj) % deletes the PRM drawings
        obj = save(obj) % saves the PRM
        % obj = request_construct_and_draw(obj) % request 2D PRM nodes (or
        % orbit centers) from the user and draws it as the same. We do not
        % have it any more
        obj = add_node(obj,new_node) % adds a new node to the graph, updates it, and re-draw it.
    end
end


function getEnvironmentFromUser()
% user_data = user_data_class; % The object user_data will never be used. This line only cause the "Constant" properties of the "user_data_class" class to be initialized.
if user_data_class.par.sim.intractive_obst == 1
    Obstacles = obstacles_class; %#ok<NASGU> % The object "Obstacles" is never used. This line only cause the "Constant" properties of the "obstacles_class" class to be initialized.
else
    Obstacles = obstacles_class; %#ok<NASGU> % The object "Obstacles" is never used. This line only cause the "Constant" properties of the "obstacles_class" class to be initialized.
    figure(gcf);
    axis(user_data_class.par.sim.env_limits)
    title({'Please mark the vertices of polygonal obstacles'},'fontsize',14)
    ib=0;
    tmp_prop.boundary_handle = [];
    tmp_prop.fill_handle = [];
    while true
        axis(user_data_class.par.sim.env_limits)
        h_obs=impoly;
        tmp_prop.boundary_handle = [tmp_prop.boundary_handle, h_obs];
        if isempty(h_obs) && ib==0
            tmp_prop.obst=[];
            break
        elseif isempty(h_obs) && ib~=0
            tmp_prop.obst = inputed_obstacles;
            break
        else
            ib=ib+1;
            inputed_obstacles{ib} = h_obs.getPosition; %#ok<AGROW>
            %         [inputed_obstacles{ib}(:,1),inputed_obstacles{ib}(:,2)] = poly2cw(inputed_obstacles{ib}(:,1),inputed_obstacles{ib}(:,2));  % ordering the polygon vertices in a clockwise order, if they are not already. % This is gonna be important in drawing 3D version of obstacles, or in projecting light on scene.
            fill_color_handle = fill(inputed_obstacles{ib}(:,1),inputed_obstacles{ib}(:,2),obstacles_class.obst_2D_color);
            tmp_prop.fill_handle = [tmp_prop.fill_handle, fill_color_handle];
        end
    end
    name = './test_delete_env8.obj';
    fid = fopen(name,'w');
    numOfVertices = 0;
    for idx = 1:numel(inputed_obstacles)
        %     inputed_obstacles{idx}
        
        numCurrentVertices = size(inputed_obstacles{idx},1);
        C = [(1:(numCurrentVertices-1))' (2:numCurrentVertices)'; numCurrentVertices 1];
        
        dt = DelaunayTri(inputed_obstacles{idx}(:,1), inputed_obstacles{idx}(:,2), C);
        io = dt.inOutStatus();
        FV.vertices=[dt.X,zeros(size(dt.X,1),1)];
        FV.faces=dt.Triangulation(io,:);
        FV.faces = FV.faces + numOfVertices;
        vertface2obj(FV.vertices,FV.faces,idx,fid)
        numOfVertices = numCurrentVertices + numOfVertices;
        
    end
    fclose(fid);
end
end
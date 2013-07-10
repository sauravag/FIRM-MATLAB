%% This is the main script for doing FIRM using the simulated laser data

%% Commands to get the data of the Laser Scanner
close all;
clear all;clc
% FIRM in MATLAB
addpath(genpath('C:\Users\Amirhossein\Documents\GitHub\FIRM-MATLAB'));
% Feature extraction ECMR, Ali's code
addpath(genpath('C:\Users\Amirhossein\Desktop\MyWork_TAMU\FIRM_with_simulated_laser\ECMR paper all things\ECMR_paper_codes\completed_after_paper_trends\for ECMR\'))
% SLAM summer school 2006 code for data association
addpath(genpath('C:\Users\Amirhossein\Desktop\MyWork_TAMU\FIRM_with_simulated_laser\SSS06.Prac1.DataAssociation'))

LoadFileName = 'laser_env_map.obj';
objOutput=read_wobj(LoadFileName);
verboseFlag = 1

% objOutput=read_wobj(LoadFileName);
ObstVertices = objOutput.vertices;
tmp_prop.obst = ObstVertices;

figure(gcf);
% axis(user_data_class.par.sim.env_limits)
title({'Please mark the vertices of polygonal obstacles'},'fontsize',14)
ib=0;
tmp_prop.boundary_handle = [];
tmp_prop.fill_handle = [];
%             if user_data_class.par_new.sim.verboseFlag

cprintf('Blue','Reading objects from %s \n',LoadFileName)
%             end
for idx_obj =1:numel(objOutput.objects)
    if (strcmp(objOutput.objects(idx_obj).type,'g'))
        if verboseFlag
            cprintf('Red','object name : %s \n',objOutput.objects(idx_obj).data)
        end
    elseif strcmp(objOutput.objects(idx_obj).type,'f')
        currentObjectVertIndices =unique(objOutput.objects(idx_obj).data.vertices(:));
        % reading the vertice positions
        vertPosX = [ObstVertices(currentObjectVertIndices,1);ObstVertices(currentObjectVertIndices(1),1)];
        vertPosY = [ObstVertices(currentObjectVertIndices,2);ObstVertices(currentObjectVertIndices(1),2)];
        vertPosZ = [ObstVertices(currentObjectVertIndices,3);ObstVertices(currentObjectVertIndices(1),3)]; 
        % Assuming thatwe walls are vertical, in order to find the corners
        % we can just considers the vertices with the same z coordinate
        % value. in our case, corners are comming from vertices with max(Z)
        maxZ = max(vertPosZ);
        vertPosX = vertPosX(vertPosZ==maxZ);
        vertPosY = vertPosY(vertPosZ==maxZ);
        vertPosZ = vertPosZ(vertPosZ==maxZ);
        
%         axis(user_data_class.par.sim.env_limits)
        h_obs=impoly(gca,[vertPosX,vertPosY]);
        tmp_prop.boundary_handle = [tmp_prop.boundary_handle, h_obs];
        if isempty(h_obs) && ib==0
            tmp_prop.obst=[];
            break
            % % %                     elseif isempty(h_obs) && ib~=0
            % % %                         tmp_prop.obst = inputed_obstacles;
            % % %                         break
        else
            ib=ib+1;
            inputed_obstacles{ib} = h_obs.getPosition; %#ok<AGROW>
            %         [inputed_obstacles{ib}(:,1),inputed_obstacles{ib}(:,2)] = poly2cw(inputed_obstacles{ib}(:,1),inputed_obstacles{ib}(:,2));  % ordering the polygon vertices in a clockwise order, if they are not already. % This is gonna be important in drawing 3D version of obstacles, or in projecting light on scene.
            fill_color_handle = fill(inputed_obstacles{ib}(:,1),inputed_obstacles{ib}(:,2),'r');
            tmp_prop.fill_handle = [tmp_prop.fill_handle, fill_color_handle];
        end
    end
end
tmp_prop.obst = inputed_obstacles;


















robot_init = [0 0 0];
sim = vrep_interface();
sim = sim.simInitialize();
sim = sim.SetRobot(robot_init);
thresholds=default_thresholds_func();

for i = 1:100
    
    sim = sim.getSensorData;
    laserData = squeeze(sim.sensor.laserData);
%     removing outlier point
    idx = find(abs(laserData(1,:))<10 & abs(laserData(2,:))<10);
    if i>3
        scan.x = -laserData(2,idx).*100;
        scan.y = laserData(1,idx).*100;
        new_features_set=hierarchical_feature_extracting(scan,thresholds,'new');
    end
end

sim = sim.delete();
 

 
 
 
 

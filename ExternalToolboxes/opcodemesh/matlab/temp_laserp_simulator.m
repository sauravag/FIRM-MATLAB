function temp_laserp_simulator()
close all
% Very simple shape
% f = [1 2 3]'; v = [-1 0 0 ; 1 1 0 ; 1 -1 0]';
% [hit,d,trix,bary] = opcodemeshmex('intersect',t,1,1,[0 0 0.5]',[0 0 -1]');

% More complex shape
objOutput = read_obj('obstacle_map_temp.obj');

f = [];




for idx_obj =1:numel(objOutput.objects)
    if (strcmp(objOutput.objects(idx_obj).type,'g'))
        if user_data_class.par.sim.verboseFlag
            cprintf('Red','object name : %s \n',objOutput.objects(idx_obj).data)
        end
    elseif strcmp(objOutput.objects(idx_obj).type,'f')
        currentObjectFaces =(objOutput.objects(idx_obj).data.vertices);
        f= [f;currentObjectFaces];
    end
end










v = objOutput.vertices;
% f = obj.objects(4).data.vertices;

patch_display(struct('vertices',v,'faces',f))
hold on
% minb = min(v(:,[3 2]));
% maxb = max(v(:,[3 2]));
%t = opcodemeshmex('create',v',f');
t = opcodemesh(v',f');
intitialTheta = 0*pi/180; % radian. The angle of the first ray of the laser
endTheta = 180*pi/180; % radian. The angle of the last ray of the laser
raysPerDegree = 4; % resolution in terms of dnumber of rays per degree
resolution = (1/raysPerDegree)*pi/180; % resolution in radians






user_data = user_data_class; % The object user_data will never be used. This line only cause the "Constant" properties of the "user_data_class" class to be initialized.

robot_init = state([0 -1 pi/4]);
robot_final = state([1.50 1 0]);

% sim = Simulator();
% sim = sim.initialize();
% sim = sim.setRobot(robot_init);
MM = MotionModel_class;
nominal_traj = MM.generate_open_loop_point2point_traj(robot_init,robot_final); % generates open-loop trajectories between two start and goal states
theta  = intitialTheta:resolution:endTheta;
rangeLaser = 4; % meter. the range of the laser scanner



for i=1:size(nominal_traj.x,2)
    
    from_laser = [nominal_traj.x(1,i),nominal_traj.x(2,i),0.05];
    
    to_rays = [[rangeLaser.*cos(theta+nominal_traj.x(3,i))]',[rangeLaser.*sin(theta+nominal_traj.x(3,i))]',repmat(0,length(theta),1) ];
    % from = [-Z(:) Y(:) X(:)]';
    % to = [Z(:) Y(:) X(:)]';
    
    %[hit,d,trix,bary] = opcodemeshmex('intersect',t,from,to-from);
    tic
    [hit,d,trix,bary] = t.intersect(repmat(from_laser',1,length(theta)),to_rays');
    a = toc;
    disp(['-------------  ',num2str(a),' seconds -----'])
    x=from_laser(1)+rangeLaser.*d'.*cos(theta+nominal_traj.x(3,i));
    y= from_laser(2)+rangeLaser.*d'.*sin(theta+nominal_traj.x(3,i));
    z = nan(1,length(theta));
    z(~isnan(d))=from_laser(3);
    % figure
    if exist('h2')
        set(h2,'Visible','off')
        set(h3,'Visible','off')
        
    end
    h2 = plot3(x,y,repmat(from_laser(3),1,length(x)),'.r','LineWidth',6);
    h3= plot3(nominal_traj.x(1,i),nominal_traj.x(1,i),0.05,'+r','MarkerSize',10);
    pause(0.1)
end
% x1=from_laser(1)+rangeLaser'.*cos(theta);
% y1=from_laser(2)+rangeLaser'.*sin(theta);
% plot3(x1,y1,repmat(from_laser(3),1,length(x1)),'.b','MarkerSize',0.1)
% opcodemeshmex('delete',t);


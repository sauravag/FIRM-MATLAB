%RRT3D Class for rapidly-exploring random tree navigation
%
% A concrete subclass of the Navigation class that implements the rapidly
% exploring random tree (RRT3D) algorithm.  This is a kinodynamic planner
% that takes into account the motion constraints of the vehicle.
%
% Methods::
%
% plan         Compute the tree
% path         Compute a path
% plot         Display the tree
% display      Display the parameters in human readable form
% char         Convert to string
%
% References::
% - Randomized kinodynamic planning,
%   S. LaValle and J. Kuffner,
%   International Journal of Robotics Research vol. 20, pp. 378-400, May 2001.
% - Probabilistic roadmaps for path planning in high dimensional configuration spaces,
%   L. Kavraki, P. Svestka, J. Latombe, and M. Overmars,
%   IEEE Transactions on Robotics and Automation, vol. 12, pp. 566-580, Aug 1996.
% - Robotics, Vision & Control, Section 5.2.5,
%   P. Corke, Springer 2011.
%
% See also Navigation, PRM, DXform, Dstar, PGraph.

%TODO
%   more info to the display method
%   distance metric choice or weightings
%   pass time and model options to the simulation

classdef RRT3D < Navigation
    
    properties
        npoints         % number of points to find
        graph           % graph Object representing random nodes
        
        sim_time        % path simulation time
        kine_model      % simulink model for kinematics
        
        xrange
        yrange
        zrange
        
        plant
        
        speed
        steermax
        vehicle
        start
        %goal
    end
    
    methods
        
        function RRT3D = RRT3D(map, vehicle, varargin)
            %RRT3D.RRT3D Create a RRT3D navigation object
            %
            % R = RRT3D.RRT3D(MAP, VEH, OPTIONS) is a rapidly exploring tree navigation
            % object for a region with obstacles defined by the map object MAP.
            %
            % R = RRT3D.RRT3D() as above but internally creates a Vehicle class object
            % and does not support any MAP or OPTIONS.  For compatibility with
            % RVC book.
            %
            % Options::
            % 'npoints',N    Number of nodes in the tree
            % 'time',T       Period to simulate dynamic model toward random point
            % 'range',R      Specify rectangular bounds
            %                - R scalar; X: -R to +R, Y: -R to +R
            %                - R (1x2); X: -R(1) to +R(1), Y: -R(2) to +R(2)
            %                - R (1x4); X: R(1) to R(2), Y: R(3) to R(4)
            % 'goal',P       Goal position (1x2) or pose (1x3) in workspace
            % 'speed',S      Speed of vehicle [m/s] (default 1)
            % 'steermax',S   Maximum steer angle of vehicle [rad] (default 1.2)
            %
            % Notes::
            % - Does not (yet) support obstacles, ie. MAP is ignored but must be given.
            % - 'steermax' selects the range of steering angles that the vehicle will
            %   be asked to track.  If not given the steering angle range of the vehicle
            %   will be used.
            % - There is no check that the steering range or speed is within the limits
            %   of the vehicle object.
            %
            % Reference::
            % - Robotics, Vision & Control
            %   Peter Corke, Springer 2011.  p102.
            %
            % See also Vehicle.
            
            % invoke the superclass constructor
            RRT3D = RRT3D@Navigation(varargin{:});
            
            RRT3D.graph = PGraph(7, 'distance', '6DOFQuaternion');  % graph of points in S0(3)
            if nargin == 0
                RRT3D.vehicle = Aircraft_Kinematic();
            else
                % RVC book compatability mode
                RRT3D.vehicle = vehicle;
            end
            
            opt.npoints = 1500;
            opt.time = 0.5;
            opt.range = 100;
            opt.start = [0, 0, 0,1,0,0,0]'; % State is a column vector
            opt.steermax = [];
            opt.speed = 1;
            
            [opt,args] = tb_optparse(opt, varargin);
            RRT3D.npoints = opt.npoints;
            RRT3D.sim_time = opt.time;
            
            switch length(opt.range)
                case 1
                    RRT3D.xrange = [-opt.range opt.range];
                    RRT3D.yrange = [-opt.range opt.range];
                    RRT3D.zrange = [-opt.range opt.range];
                case 2
                    RRT3D.xrange = [-opt.range(1) opt.range(1)];
                    RRT3D.yrange = [-opt.range(2) opt.range(2)];
                    RRT3D.zrange = [-opt.range(3) opt.range(3)];
                case 4
                    RRT3D.xrange = [opt.range(1) opt.range(2)];
                    RRT3D.yrange = [opt.range(3) opt.range(4)];
                    RRT3D.zrange = [opt.range(5) opt.range(6)];
                otherwise
                    error('bad range specified');
            end
            %             if ~isempty(opt.steermax)
            %                 RRT3D.steermax = opt.steermax;
            %             else
            %                 RRT3D.steermax = RRT3D.vehicle.alphalim;
            %             end
            RRT3D.speed = opt.speed;
            RRT3D.start = opt.start;
            
        end
        
        function plan(RRT3D, varargin)
            %RRT3D.plan Create a rapidly exploring tree
            %
            % R.plan(OPTIONS) creates the tree roadmap by driving the vehicle
            % model toward random goal points.  The resulting graph is kept
            % within the object.
            %
            % Options::
            % 'goal',P        Goal pose (1x3)
            % 'noprogress'    Don't show the progress bar
            % 'samples'       Show samples
            %                 - '.' for each random point x_rand
            %                 - 'o' for the nearest point which is added to the tree
            %                 - red line for the best path
            
            opt.progress = true;
            opt.samples = false;
            opt.start = [];
            opt.goal  = [];
            
            opt = tb_optparse(opt, varargin);
            
            if ~isempty(opt.goal)
                RRT3D.goal = opt.goal;
            end
            
            % build a graph over the free space
            RRT3D.message('create the graph');
            RRT3D.graph.clear();
            
            if RRT3D.verbose
                clf
                %idisp(1-RRT3D.occgrid, 'ynormal', 'nogui');
                hold on
            end
            
            % check goal sanity
            %             if isempty(RRT3D.goal)
            %                 error('no goal specified');
            %             end
            %             switch length(RRT3D.goal)
            %             case 3
            %                 RRT3D.goal = [RRT3D.goal(:);1;0;0;0];
            %             case 3
            %             otherwise
            %                 error('goal must be 3-vector');
            %             end
            
            % add the goal point as the first node
            RRT3D.graph.add_node(RRT3D.start);
            
            % graphics setup
            if opt.progress
                h = waitbar(0, 'RRT3D planning...');
            end
            if opt.samples
                clf
                hold on
                xlabel('x'); ylabel('y');
            end
            
            [nPointsOnSegment, pointsOnSegment] = getPointsOnGuidanceVector(RRT3D);
            RRT3D.npoints = nPointsOnSegment+1
            for j=1:RRT3D.npoints      % build the tree
                
                if opt.progress
                    waitbar(j / RRT3D.npoints);
                end
                
                % Step 3
                % find random state x,y,z,q0,q1,q2,q3
                % pick a point not in obstacle
                
                %                 while true
                %                     xyz = RRT3D.randxyz();  % get random coordinate (x,y,z)
                %
                %                     % test if lies in the obstacle map (if it exists)
                %                     if isempty(RRT3D.occgrid)
                %                         break;
                %                     end
                %                     try
                %                         if RRT3D.occgrid(ixy(2),ixy(1)) == 0
                %                             break;
                %                         end
                %                     catch
                %                         % index error, point must be off the map
                %                         continue;
                %                     end
                %                 end
                %                 yaw = RRT3D.rand*2*pi;
                %                 pitch =  RRT3D.rand*2*pi;
                %                 roll =  RRT3D.rand*2*pi;
                %                 quat = angle2quat(yaw,pitch,roll);
                %                 xrand = [xyz, quat]';
                %
                %                 if opt.samples
                %                     plot3(xyz(1), xyz(2),xyz(3), '.')
                %                 end
                
                %STEP 3 NEW
                xrand = pointsOnSegment(:,j);
                xyz = xrand(1:3);
                
                % Step 4
                % find the existing node closest in state space
                
                vnear = RRT3D.graph.closest(xrand);   % nearest vertex
                xnear = RRT3D.graph.coord(vnear);     % coord of nearest vertex
                
                RRT3D.message('xrand (%g, %g) node %d', xyz, vnear);
                
                % Step 5
                % figure how to drive the robot from xnear to xrand
                
                ntrials = 75;% Try 75
                
                best = RRT3D.bestpath(xnear, xrand, ntrials);
                
                xnew = best.path(:,best.k);
                if opt.samples
                    plot3(xnew(1), xnew(2),xnew(3), 'o');
                    plot3(best.path', 'r');
                    drawnow
                end
                
                %                 % ensure that the path is collision free
                %                 if ~RRT3D.clearpath(y(:,1:2))
                %                     disp('path collision');
                %                     continue;
                %                 end
                
                % Step 7,8
                % add xnew to the graph, with an edge from xnear
                v = RRT3D.graph.add_node(xnew);
                RRT3D.graph.add_edge(vnear, v);
                
                RRT3D.graph.setdata(v, best);
            end
            
            if opt.progress
                close(h)
            end
            RRT3D.message('graph create done');
        end
        
        function p_ = path(RRT3D, xstart, xgoal)
            %PRM.path Find a path between two points
            %
            % X = R.path(START, GOAL) finds a path (Nx3) from state START (1x3)
            % to the GOAL (1x3).
            %
            % P.path(START, GOAL) as above but plots the path in 3D.  The nodes
            % are shown as circles and the line segments are blue for forward motion
            % and red for backward motion.
            %
            % Notes::
            % - The path starts at the vertex closest to the START state, and ends
            %   at the vertex closest to the GOAL state.  If the tree is sparse this
            %   might be a poor approximation to the desired start and end.
            
            g = RRT3D.graph;
            vstart = g.closest(xstart);
            vgoal = g.closest(xgoal);
            
            % find path through the graph using A* search
            path = g.Astar(vstart, vgoal);
            
            % concatenate the vehicle motion segments
            cpath = [];
            for i = 1:length(path)
                p = path(i);
                data = g.data(p);
                if ~isempty(data)
                    if i >= length(path) || g.edgedir(p, path(i+1)) > 0
                        cpath = [cpath data.path];
                    else
                        cpath = [cpath data.path(:,end:-1:1)];
                        
                    end
                end
            end
            
            if nargout == 0
                % plot the path
                clf; hold on
                coords  = g.coord(path)'
                plot3(coords(:,1),coords(:,2),coords(:,3), 'o');     % plot the node coordinates
                
                for i = 1:length(path)
                    p = path(i)
                    b = g.data(p);            % get path data for segment
                    
                    % draw segment with direction dependent color
                    if ~isempty(b)
                        % if the vertex has a path leading to it
                        
                        if i >= length(path) || g.edgedir(p, path(i+1)) > 0
                            % positive edge
                            %  draw from prev vertex to end of path
                            seg = [g.coord(path(i-1)) b.path]';
                        else
                            % negative edge
                            %  draw reverse path to next next vertex
                            seg = [  b.path(:,end:-1:1)  g.coord(path(i+1))]';
                        end
                        
                        if b.vel > 0
                            plot2(seg, 'b');
                        else
                            plot2(seg, 'r');
                        end
                    end
                end
                
                xlabel('x'); ylabel('y'); zlabel('z');
                grid
            else
                p_ = cpath';
            end
            p_ = path;
        end
        
        function plot(RRT3D, varargin)
            %RRT3D.plot Visualize navigation environment
            %
            % R.plot() displays the navigation tree in 3D.
            
            clf
            RRT3D.graph.plot('noedges', 'NodeSize', 6, 'NodeFaceColor', 'g', 'NodeEdgeColor', 'g', 'edges');
            
            hold on
            for i=2:RRT3D.graph.n
                b = RRT3D.graph.data(i);
                plot3(b.path(:,1:b.k)')
            end
            xlabel('x'); ylabel('y'); zlabel('Z');
            grid; hold off
        end
        
        % required by abstract superclass
        function next(RRT3D)
        end
        
        function s = char(RRT3D)
            %RRT3D.char  Convert to string
            %
            % R.char() is a string representing the state of the RRT3D
            % object in human-readable form.
            %
            % invoke the superclass char() method
            s = char@Navigation(RRT3D);
            
            % add RRT3D specific stuff information
            s = char(s, sprintf('  region: X %f : %f; Y %f : %f', RRT3D.xrange, RRT3D.yrange));
            s = char(s, sprintf('  path time: %f', RRT3D.sim_time));
            s = char(s, sprintf('  graph size: %d nodes', RRT3D.npoints));
            s = char(s, char(RRT3D.graph) );
            %             if ~isempty(RRT3D.vehicle)
            %                 s = char(s, char(RRT3D.vehicle) );
            %             end
        end
        
        function test(RRT3D)
            xy = RRT3D.randxy()
        end
        
        
    end % methods
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%    P R I V A T E    M E T H O D S
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods (Access='protected')
        
        function best = bestpath(RRT3D, x0, xg, N)
            
            % initial and final state as column vectors
            x0 = x0(:); xg = xg(:);
            
            best.d = Inf;
            for i=1:N   % for multiple trials
                
                %choose random direction of motion and random steer angle
                %                 if rand > 0.5
                %                     vel = RRT3D.speed;
                %                 else
                %                     vel = -RRT3D.speed;
                %                 end
                vel = RRT3D.vehicle.Min_Velocity + abs((2*RRT3D.rand - 1)*(RRT3D.vehicle.Max_Velocity - RRT3D.vehicle.Min_Velocity));
                roll_rate = (2*RRT3D.rand - 1) * RRT3D.vehicle.Max_Roll_Rate;
                pitch_rate = (2*RRT3D.rand - 1) * RRT3D.vehicle.Max_Pitch_Rate;
                yaw_rate = (2*RRT3D.rand - 1) * RRT3D.vehicle.Max_Yaw_Rate;
                
                u = [vel,roll_rate, pitch_rate, yaw_rate];
                w = [0,0,0,0];
                % simulate motion of vehicle for this speed and steer angle which
                % results in a path
                x = RRT3D.vehicle.f_discrete(x0, u, w);
                
                %% find point on the path closest to xg
                % distance of all path points from goal
                d = colnorm( [bsxfun(@minus, x(1:7,:), xg)] );
                % the closest one
                [dmin,k] = min(d);
                
                % is it the best so far?
                if dmin < best.d
                    % yes it is!  save it and the inputs that led to it
                    best.d = dmin;
                    best.path = x;
                    best.steer = u;
                    best.vel = vel;
                    best.k = k;
                end
            end
        end
        
        % generate a random coordinate within the working region
        function xyz = randxyz(RRT3D)
            xstart = RRT3D.start;
            xgoal = RRT3D.goal;
            midpoint = (xstart(1:3)+xgoal(1:3))/2;
            
            d = norm(xstart(1:3)-xgoal(1:3));
            range = d;
            xrange = [-range range];
            yrange = [-range range];
            zrange = [-range range];
            xyz = RRT3D.rand(1,3) .* [xrange(2)-xrange(1) yrange(2)-yrange(1) zrange(2)-zrange(1)] + ...
                [midpoint(1)+RRT3D.xrange(1) midpoint(2)+RRT3D.yrange(1) midpoint(3)+RRT3D.zrange(1)];
        end
        
        function xy = randxy(RRT3D)
            xy = RRT3D.rand(1,2) .* [RRT3D.xrange(2)-RRT3D.xrange(1) RRT3D.yrange(2)-RRT3D.yrange(1)] + ...
                [RRT3D.xrange(1) RRT3D.yrange(1)];
        end
        
        % test if a path is obstacle free
        function c = clearpath(RRT3D, xy)
            if isempty(RRT3D.occgrid)
                c = true;
                return;
            end
            
            xy = round(xy);
            try
                % test that all points along the path do lie within an obstacle
                for pp=xy'
                    if RRT3D.occgrid(pp(2), pp(1)) > 0
                        c = false;
                        return;
                    end
                end
                c = true;
            catch
                % come here if we index out of bounds
                c = false;
                return;
            end
        end
        
        function [nPointsOnSegment, Points] = getPointsOnGuidanceVector(RRT3D)
            
            vel = RRT3D.vehicle.Max_Velocity;
            nPointsOnSegment = ceil(( norm(RRT3D.goal(1:3)-RRT3D.start(1:3)) )/(vel*RRT3D.sim_time));
            
            Points = zeros(7, nPointsOnSegment+1);
            
            %NEED TO FIND THE PITCH AND YAW ANGLE FOR THE AIRCRAFT ON THIS
            %SEGMENT
            guidanceVector = RRT3D.goal(1:3) - RRT3D.start(1:3);
            yaw = atan2(guidanceVector(2),guidanceVector(1));
            pitch = atan2(abs(guidanceVector(3)),sqrt((guidanceVector(1))^2 + (guidanceVector(2))^2));
            
            if guidanceVector(3) > 0
                pitch = -pitch;
            end
            roll =  0;
            quat = angle2quat(yaw,pitch,roll);
            
            for i=1:nPointsOnSegment+1
                Points(1:3,i) = RRT3D.start(1:3) + (i/(nPointsOnSegment+1))*guidanceVector(1:3);
                Points(4:7,i) = quat;
            end
            quat = angle2quat(yaw,0,roll);
            Points(4:7,end)=quat;
        end
        
        
    end % private methods
end % class

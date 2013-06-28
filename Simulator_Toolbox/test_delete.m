% % % AA = 'C:\Users\Amirhossein\Dropbox\FIRM_toolbox_ver_current (1)\'
% % % m2html('mfiles',AA, 'htmldir','doc', 'recursive','on', 'global','on','template','frame', 'index','menu', 'graph','on');
% % % 
% % % 
% % % 
  OBJ=read_wobj('examples\example10.obj');
  FV.vertices=OBJ.vertices;
  FV.faces=OBJ.objects(3).data.vertices;
  figure, patch(FV,'facecolor',[1 0 0]); camlight

% % %   
% % %   
% % %     % Load MRI scan
% % %   load('mri','D'); D=smooth3(squeeze(D));
% % %   
% % %   
% % %   
% % %   
% % %     OBJ=read_wobj('examples\example10.obj');
% % %   FV.vertices=OBJ.vertices;
% % %   FV.faces=OBJ.objects(3).data.vertices;
% % %   % Make iso-surface (Mesh) of skin
% % % %   FV=isosurface(D,1);
% % %   % Calculate Iso-Normals of the surface
% % %   N=isonormals(D,FV.vertices);
% % %   L=sqrt(N(:,1).^2+N(:,2).^2+N(:,3).^2)+eps;
% % %   N(:,1)=N(:,1)./L; N(:,2)=N(:,2)./L; N(:,3)=N(:,3)./L;
% % %   % Display the iso-surface
% % %   figure, patch(FV,'facecolor',[1 0 0],'edgecolor','none'); view(3);camlight
% % %   % Invert Face rotation
% % %   FV.faces=[FV.faces(:,3) FV.faces(:,2) FV.faces(:,1)];
% % % 
% % %   % Make a material structure
% % %   material(1).type='newmtl';
% % %   material(1).data='skin';
% % %   material(2).type='Ka';
% % %   material(2).data=[0.8 0.4 0.4];
% % %   material(3).type='Kd';
% % %   material(3).data=[0.8 0.4 0.4];
% % %   material(4).type='Ks';
% % %   material(4).data=[1 1 1];
% % %   material(5).type='illum';
% % %   material(5).data=2;
% % %   material(6).type='Ns';
% % %   material(6).data=27;
% % % 
% % %   % Make OBJ structure
% % %   clear OBJ
% % %   OBJ.vertices = FV.vertices;
% % %   OBJ.vertices_normal = N;
% % %   OBJ.material = material;
% % %   OBJ.objects(1).type='g';
% % %   OBJ.objects(1).data='skin';
% % %   OBJ.objects(2).type='usemtl';
% % %   OBJ.objects(2).data='skin';
% % %   OBJ.objects(3).type='f';
% % %   OBJ.objects(3).data.vertices=FV.faces;
% % %   OBJ.objects(3).data.normal=FV.faces;
% % %   write_wobj(OBJ,'skinMRI.obj');
% % % %



clf
close all
clc

% load usapolygon
% Define an edge constraint between two successive
% points that make up the polygonal boundary.
v_out
nump = size(v_out,2);
C = [(1:(nump-1))' (2:nump)'; nump 1];





dt = DelaunayTri(v_out(1,:)', v_out(2,:)', C);
io = dt.inOutStatus();
patch('faces',dt(io,:), 'vertices', dt.X, 'FaceColor','r');
axis equal;
% axis([-130 -60 20 55]);
xlabel('Constrained Delaunay Triangulation of usapolygon', 'fontweight','b');
figure
plot(v_out(1,:),v_out(2,:))






v_AliOffice
nump1 = size(v_AliOffice,2);
C1 = [(1:(nump1-1))' (2:nump1)'; nump1 1];

dt1 = DelaunayTri(v_AliOffice(1,:)', v_AliOffice(2,:)', C1);
io1 = dt1.inOutStatus();
patch('faces',dt1(io1,:), 'vertices', dt1.X, 'FaceColor','r');
axis equal;
% axis([-130 -60 20 55]);
xlabel('Constrained Delaunay Triangulation of usapolygon', 'fontweight','b');
figure
plot(v_AliOffice(1,:),v_AliOffice(2,:))







  FV.vertices=[dt.X,zeros(size(dt.X,1),1);...
               dt1.X,zeros(size(dt1.X,1),1)];
  FV.faces=[dt.Triangulation(io,:);...
            dt1.Triangulation(io1,:)+size(dt.X,1)];
  
  vertface2obj(FV.vertices,FV.faces,'environment_forth_floor2.obj')

load usapolygon
% Define an edge constraint between two successive
% points that make up the polygonal boundary.
nump = numel(uslon);
C = [(1:(nump-1))' (2:nump)'; nump 1];
dt = DelaunayTri(uslon, uslat, C);
io = dt.inOutStatus();
patch('faces',dt(io,:), 'vertices', dt.X, 'FaceColor','r');
axis equal;
axis([-130 -60 20 55]);
xlabel('Constrained Delaunay Triangulation of usapolygon', 'fontweight','b');
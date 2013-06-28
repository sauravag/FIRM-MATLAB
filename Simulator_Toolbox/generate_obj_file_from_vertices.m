function generate_obj_file_from_vertices(vertices,fileName)


name = fileName;
fid = fopen(name,'w');
numOfVertices = 0;
for idx = 1:numel(vertices)
    %     vertices{idx}
    
    numCurrentVertices = size(vertices{idx},1);
    C = [(1:(numCurrentVertices-1))' (2:numCurrentVertices)'; numCurrentVertices 1];
    
    dt = DelaunayTri(vertices{idx}(:,1), vertices{idx}(:,2), C);
    
    numCurrentVertices = size(dt.X,1);
    
    io = dt.inOutStatus();
    FV.vertices=[dt.X,zeros(size(dt.X,1),1)];
    FV.faces=dt.Triangulation(io,:);
    FV.faces = FV.faces + numOfVertices;
    vertface2obj(FV.vertices,FV.faces,idx,fid)
    numOfVertices = numCurrentVertices + numOfVertices;
    
end
fclose(fid);



% % %
% % %
% % % name = fileName;%'./v_AliOffice.obj';
% % % fid = fopen(name,'w');
% % % numOfVertices = 0;
% % %
% % % idx =1;
% % % vertices{idx} = vertices;
% % %
% % % numCurrentVertices = size(vertices{idx},1);
% % % C = [(1:(numCurrentVertices-1))' (2:numCurrentVertices)'; numCurrentVertices 1];
% % %
% % % dt = DelaunayTri(vertices{idx}(:,1), vertices{idx}(:,2), C);
% % % io = dt.inOutStatus();
% % % FV.vertices=[dt.X,zeros(size(dt.X,1),1)];
% % % FV.faces=dt.Triangulation(io,:);
% % % FV.faces = FV.faces + numOfVertices;
% % % vertface2obj(FV.vertices,FV.faces,idx,fid)
% % % % numOfVertices = numCurrentVertices + numOfVertices;
% % % fclose(fid);
% % %


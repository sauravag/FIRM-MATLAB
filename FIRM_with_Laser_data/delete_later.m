    % Intersect a cube by a plane
    plane = createPlane([0 0 0.2], [0 0 1]);
    % draw the primitives
    figure; hold on; set(gcf, 'renderer', 'opengl');
%     axis([-7 7 -7 7 -2 2]); 
view(3);
    drawMesh(objOutput.vertices, faces_); drawPlane3d(plane);
    % compute intersection polygon
    polys = intersectPlaneMesh(plane, objOutput.vertices, faces_);
    figure
    drawPolygon3d(polys, 'LineWidth', 2);

    faces_=[];
    
    for idx_obj =1:numel(objOutput.objects)
    if (strcmp(objOutput.objects(idx_obj).type,'g'))
        if verboseFlag
            cprintf('Red','object name : %s \n',objOutput.objects(idx_obj).data)
        end
    elseif strcmp(objOutput.objects(idx_obj).type,'f')
        currentObjectVertIndices =unique(objOutput.objects(idx_obj).data.vertices(:));
        % reading the v
faces_ = [faces_;objOutput.objects(idx_obj).data.vertices]

    end
    end
    
            meshFromObj = makeMesh(objOutput.vertices,faces_)
            
            
            
            
faces_ = objOutput.objects(idx_obj).data.vertices ;
vertices_ = objOutput.vertices(1:32,:);
            meshFromObj = makeMesh(vertices_,faces_)


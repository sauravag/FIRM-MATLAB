% errors = [];
% start_k = 12;
% s = obj.planned_lnr_pts_seq(start_k).x;
% for k = start_k:length(obj.planned_lnr_pts_seq)-1
%     s = MotionModel_class.f_discrete(s,obj.planned_lnr_pts_seq(k+1).u,zeros( MotionModel_class.wDim,1))
%     %sstate = state(s);
%     %sstate.draw('robotshape','triangle','triacolor','g')
%     errors = [errors , s - obj.planned_lnr_pts_seq(k+1).x]
% end

clc
obj = obj.edge_controller;
u_openLoop = [obj.planned_lnr_pts_seq.u]; % planned u
x_openLoop = [obj.planned_lnr_pts_seq.x]; % planned x
w_openLoop = [obj.planned_lnr_pts_seq.w]; % planned w

errors_accumulative = [];
errors_just = [];
start_k = 1;
s = obj.planned_lnr_pts_seq(start_k).x;
for k = 1:length(x_openLoop)-1
    s = MotionModel_class.f_discrete(s, u_openLoop(:,k), w_openLoop(:,k));
    %sstate = state(s);
    %sstate.draw('robotshape','triangle','triacolor','g')
    errors_accumulative = [errors_accumulative , s - x_openLoop(:,k+1)];
    
    just_s = MotionModel_class.f_discrete(x_openLoop(:,k), u_openLoop(:,k), w_openLoop(:,k));
    errors_just = [errors_just , just_s - x_openLoop(:,k+1)];
end
errors_accumulative
errors_just

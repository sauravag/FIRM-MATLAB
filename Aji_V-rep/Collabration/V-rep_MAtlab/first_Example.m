% clear classes;clear variables;close all;clc;
% seed =502; rand('state',seed);randn('state',seed); %#ok<RAND>
% addpath(genpath('General_functions')); % In this line we add everything inside the "General_functions" to the current paths of the matlab.
% startup
% load myfilebrkpnts;dbstop(s)
% 
% % Parameters
% user_data = user_data_class; % The object user_data will never be used. This line only cause the "Constant" properties of the "user_data_class" class to be initialized.


% aa = state.sample_a_valid_state;
% bb = state.sample_a_valid_state;


aa = state;
bb = state;

aa.val = [0;0;0]
aa.draw

bb.val = [5;8;pi/6]
bb.draw

aa.val
bb.val
mm = Unicycle_robot;
action = mm.generate_VALID_open_loop_point2point_traj(aa,bb)


aa.draw
for i=1:size(action.u,2)
    w=[0;0;0;0;0];
    u=action.u(:,i);
    x_next = mm.f_discrete(aa.val,u,w);
    aa.val = x_next;
    aa.draw
end
bb.val -  aa.val

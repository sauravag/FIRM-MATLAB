clear classes;clear variables;%close all;
clc
rand('state',0);randn('state',11); %#ok<RAND>
cd ..
addpath(genpath('General_functions')); % In this line we add everything inside the "General_functions" to the current paths of the matlab.
addpath(genpath('All_FIRM_graph_classes')); % In this line we add everything inside the "All_FIRM_graph_classes" to the current paths of the matlab.
addpath(genpath('All_PRM_classes')); % In this line we add everything inside the "All_PRM_classes" to the current paths of the matlab.
addpath(genpath('All_stabilizer_classes')); % In this line we add everything inside the "All_FIRM_Node_classes" to the current paths of the matlab.
startup
load myfilebrkpnts;dbstop(s)
% Parameters
user_data = user_data_class; % The object user_data will never be used. This line only cause the "Constant" properties of the "user_data_class" class to be initialized.
if user_data_class.par.Cancel_Run ~= 1
    % video making
    if user_data_class.par.sim.video == 1;
        global vidObj; %#ok<TLEV>
        vidObj = VideoWriter([user_data_class.par.output_directory,'\OnlinePhase_video.avi']);
        vidObj.Quality = 100;
        open(vidObj);
    end
    
    % main program
    prob_inst = Planning_Problem;
    prob_inst = prob_inst.solve();
    if user_data_class.par.sim.video == 1;  close(vidObj);  end
end
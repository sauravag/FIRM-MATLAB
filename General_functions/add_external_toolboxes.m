function add_external_toolboxes()
% For the 6D aircraft Model, quadrotor, and if you want generate rrt tree
% we use Peter Corke's Robotics Toolbox.
% ==================== IT IS IMPORTANT TO NOTE THAT YOU SHOULD NOT RUN
% startup_rvc IN THE MIDDLE OF YOUR PROGRAM, AS IT MAY DELETE SOME OF THE
% VARIABLES IN THE WORKSPACE. SUCH AN INSTANCE HAS BEEN EXPERIENCED WITH A "GLOBAL"
% VARIABLE.
    addpath('./ExternalToolboxes/rvctools/'); 
    startup_rvc; 
end
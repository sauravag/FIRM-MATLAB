classdef user_data_class < handle
    properties (Constant = true)
        par = call_GUI_helper_function();
    end
end

function par_new = call_GUI_helper_function()
ToolboxAddress = fullfile(fileparts(which('info.xml')));
addpath(genpath(ToolboxAddress));
%List_of_folders = dir (fullfile(MainFolderAddress,'output')); % provides the list of folders in the output directory.
%last_folder_name = List_of_folders(end).name;  % retrieves the name of last saved folder, which contains the parameteres of last run of program.
global New_LoadFileName; % This is defined as a global variable and the "user_GUI" will have access to that.
if exist(fullfile(ToolboxAddress,'output','current_parameters.mat'))
    New_LoadFileName = fullfile(ToolboxAddress,'output','current_parameters.mat'); % The name of file, from which we want to load the parameters.
else
    New_LoadFileName = fullfile(ToolboxAddress,'output','default_parameters.mat'); % The name of file, from which we want to load the parameters.
end
load(New_LoadFileName,'par')
old_par = par;

[gui_handle, OkCancel, par_new_from_GUI] = user_GUI(); %#ok<ASGLU> % This function uses the global variable "LoadFileName" internally
par_new_from_GUI.Cancel_Run = strcmpi(OkCancel,'Cancel'); % we need this in the main file

if ~strcmpi(OkCancel,'Cancel') % if the user press Ok button
%     new_output_directory = fullfile(MainFolderAddress,'output',datestr(now, 'yyyy-mm-dd-HH-MM-SS'));
    par_new = Input_XML_reader(old_par, par_new_from_GUI);
    %===========  Save and Load file names
    % we make a directory based on current date and time to save the parameters and results in it.
    par_new.LoadFileName = New_LoadFileName;  % Since after loading the "par", the "LoadFileName" changes to whaterever exists in "par", we need to update it here with "New_LoadFileName".
    par_new.Cancel_Run = par_new_from_GUI.Cancel_Run;
%     par_new.output_directory = new_output_directory;
%     par_new.environmentFile = fullfile(fileparts(which('Main.m')),'Environment_Construction','environment_forth_floor.obj');
    par_new.environmentFile = fullfile(ToolboxAddress,'output','obstacle_map.obj');
    %par_new.SaveFileName = fullfile(par_new.output_directory,'parameters.mat');
    par_new.SaveFileName = New_LoadFileName;
    
    %===========  saving the parameters in a file
    
%     mkdir(par_new.output_directory)
%     old_parameters_file = par_new.LoadFileName;
%     [saving_folder_path, ~, ~] = fileparts(par_new.SaveFileName); % This line returns the path of the folder into which we want to save the parameters.
%     if exist(old_parameters_file,'file')
%         copyfile(old_parameters_file,saving_folder_path)
%     end
    par = par_new; %#ok<NASGU>
    save(par_new.SaveFileName ,'par','-append')
else
    par_new.Cancel_Run = par_new_from_GUI.Cancel_Run;
end

end
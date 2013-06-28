function Help_generator()
clc;fclose('all');%clear variables;close all;clear classes

addpath(genpath('General_functions')); % In this line we add everything inside the "General_functions" to the current paths of the matlab.
user_data = user_data_class; % The object user_data will never be used. This line only cause the "Constant" properties of the "user_data_class" class to be initialized.

addpath(genpath(pwd)); % add the current directory and all its subdirectories to the Matlab path
Help_folder_adress = [pwd,'\Help_html']; % the folder in which we save all html help files.
class_listing_file_address = [Help_folder_adress,'\class_listing.m']; % the address of file, to which we write the list of functions. The published version (html) of this file will be seen if the "classes and functions list" link on the "first page of help" is clicked.
fid = fopen(class_listing_file_address , 'w');
beginning_text = [sprintf('%%%% Classes by Category\n%% FIRM Motion Planning Toolbox\n%% Version 12.4.17.00'), date, sprintf('\n%% Requires Control System Toolbox(TM).\n%%\n') ];

[local_file_text] = generate_help_for_a_folder(pwd , Help_folder_adress);

final_text = sprintf('%%%% Source\n%% Ali-akbar Agha-mohammadi\n%% Copyright 2011-2012 The ?, Inc.');

entire_text = [beginning_text,local_file_text,final_text];

fwrite(fid, entire_text);
fclose(fid);

publish_options.outputDir = Help_folder_adress;
publish(class_listing_file_address , publish_options);
helpbrowser
% open([class_listing_file_address(1:end-1),'html']);
rmpath(genpath(pwd)); % add the current directory and all its subdirectories to the Matlab path
end


%====================================================================================
function local_file_text = generate_help_for_a_folder(current_folder_adress , Help_folder_adress)

folder_contents = dir; % the content of current directory
for i = 1:length(folder_contents)
    is_valid_folder(i) = folder_contents(i).isdir && folder_contents(i).name(1)~='.'; % check if it is folder or not and also check if it starts with  "All", ie, if it is a repository for some other classes or not.
    is_class_file(i) = (size(folder_contents(i).name, 2)>2) && exist(folder_contents(i).name(1:end-2), 'class'); % if some file is a class file it must have the ".m" extension, which is checked by the first condition. Then, the second condition removes the ".m", because the function "exist" only works without the extension ".m".
    is_class_folder(i) = folder_contents(i).isdir && folder_contents(i).name(1) == '@'; % check if the folder is the "class folder" or not.
end

[~, current_folder_name, ~] = fileparts(current_folder_adress); % returns the name of last folder in the given address (i.e., the name of "current_folder")
disp(current_folder_name)

if strcmpi(current_folder_name,'output') || strcmpi(current_folder_name,'Code_for_testing_classes') || ...
        strcmpi(current_folder_name,'General_functions') || strcmpi(current_folder_name,'Help_html') || ...
        strcmpi(current_folder_name,'StandAlone_code_To_produce_plots_in_paper_and_ppt') || ...
        strcmpi(current_folder_name,'Top_runs') % Exclude the folders that you do not want to have "html" help folder.
    local_file_text = [];
    return
end

current_name_len = size(current_folder_name,2);
if strcmpi(current_folder_name(1:min(3,current_name_len)) ,  'All')
    local_file_text = [sprintf('%%%% '), current_folder_name, sprintf(' Classes\n')];
else
    local_file_text = [];
end


for i = find(is_class_file | is_class_folder) % in this for loop, we only generate help for the "classes" in the "current_folder"
    [~,class_help]=helpwin_modified(folder_contents(i).name); % we generate help for the i-th class in the "current_folder"
    if is_class_file(i)
        local_help_file_name = [folder_contents(i).name(1:end-2),'_help.html'];
    elseif is_class_folder(i)
        local_help_file_name = [folder_contents(i).name(2:end),'_help.html'];
    end
    local_help_html_adress = [Help_folder_adress,'\',local_help_file_name];
    fid_local = fopen(local_help_html_adress , 'w'); % open the html file
    fwrite(fid_local, class_help); % write on the html file
    fclose(fid_local); % close the html file
    % Now, generate the necessary "html_code" for the "class_listing_file",
    % which inculdes the address and name of the file you have just
    % created.
    strange_html_adress = ['file:///',Help_folder_adress,'\',local_help_file_name];
    %         strange_html_adress = ['matlab:helpwin(',sprintf('\'''),Help_folder_adress,'\',local_help_file_name,sprintf('\'''),')'];
% some addresses may contain "space". So, we have to replace those with
    % "%20" so that the html address is still valid.
    space_locations = strfind(strange_html_adress,' ');
    for i_space = space_locations
        strange_html_adress(i_space)=[];

strange_html_adress=[strange_html_adress(1:i_space-1),'%20',strange_html_adress(i_space:end)];
    end
    local_file_text = [local_file_text , [sprintf('%% * <'), strange_html_adress, sprintf(' |'), local_help_file_name(1:end-5), sprintf('|> - class explanation\n')]  ]; %#ok<AGROW>
end

insider_file_text = [];
for i = find(is_valid_folder & ~(is_class_file | is_class_folder) )
    % Following "cd" is not really needed, i.e., the "what" works even
    % without following "cd". However, the "helpwin" does not work, without
    % entering into the folder, so I have to use following "cd", for which I do
    % "cd .." at the end.
    subfolder = folder_contents(i).name;
    old_folder = cd(subfolder );  %#ok<NASGU> %  I do not use the "old_folder" now, but it is good to remember that the "cd" command returns the "old_folder".
    insider_file_text = generate_help_for_a_folder(subfolder , Help_folder_adress);
    cd ..
    local_file_text = [local_file_text , insider_file_text]; %#ok<AGROW>
end % end of for loop
data_text = [];%sprintf('%%%% Data\n%% * somedatafile.mat - its
% description\n'); % uncomment this line if you have some data file that
% you want to appear in the listing.
local_file_text = [local_file_text , data_text];

end % end of function

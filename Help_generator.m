function Help_generator()
%clear variables;close all;clear classes; Note: fclose('all') close all open figures and windows
clc;fclose('all');

%genpath(FolderName) will generate the directory for the FolerName
%addpath(Directory) will add the Directory and any subdirectory inside it to the current path of the MATLAB
addpath(genpath('General_functions'));

% The object user_data will never be used. This line only cause the "Constant" properties of the "user_data_class" class in user_data_class.m to be initialized.
% This line is added since, we need some of the variables to be initialized by getting the user input data to the par
% This line should be replaced by some XML file so that we do not need to run the program to generate the help files
user_data = user_data_class;

% pwd has the current directory.
% Thus, this line adds the current directory and all its subdirectories to the Matlab path
addpath(genpath(pwd));

produce_info_XML();

% We concatenate the matlabroot and the folder name of help files and store the directory of the all html help files in Help_folder_adres
%Help_folder_adress = fullfile(matlabroot,'help','toolbox','FIRM');
Help_folder_adress = fullfile(pwd,'Help_html');

% WE add the path of the Help folder to the current path of MATLAB
addpath(Help_folder_adress);

% We will write the list of functions in class_listing.m file that in inside the help_html folder
% The published version (html) of class_listing.m will be seen if the "classes and functions list" link on the "first page of help" is clicked.
% class_listing_file_address has the directory of the class_listing.m
class_listing_file_address = fullfile(Help_folder_adress,'class_listing.m'); 

% fopen(FileDirectory,'w') opens the File in the Directory with write-ability permission and makes it an open file object
% fid will be the open class_listing.m file object which can be modified and all the previous written lines in it will be discarded
% Note: if fopen could not open the file it returns the error message in the msg e.g. not enough permissions. Note that the directory of help
% folders should have full permissions for the user that wants to run this help-generator
[fid,msg] = fopen(class_listing_file_address , 'w'); %#ok<NASGU>

% beginning_text is the first few comment lines that we write in the class_listing.m including commnets about the name, author, etc.
beginning_text = [sprintf('%%%% Classes by Category\n%% FIRM Motion Planning Toolbox\n%% Version 12.4.17.00'), date, sprintf('\n%% Requires Control System Toolbox(TM).\n%%\n') ];

% local_file_text includes the body of the class_listing.m file that is obtained by calling the generate_help_for_a_folder()function in below
% Note: we send the current directory and the beginning directory and the address of the help_html folder as the destination for the generated html files
[local_file_text] = generate_help_for_a_folder(pwd , Help_folder_adress);

% final_text will include the end comments that we will write on the class_listing.m
final_text = sprintf('%%%% Source\n%% Ali-akbar Agha-mohammadi\n%% Copyright 2011-2012.');

% We concatenate all the text lines in one text object
entire_text = [beginning_text,local_file_text,final_text];

% Here we write the entire text to the class_listing.m and then close it
fwrite(fid, entire_text);
fclose(fid);

% First we construct a struct "publish_options" and give it a member
% "outputDir"; then we put the directory of the help files (help_html) folder in this member
publish_options.outputDir = Help_folder_adress;

% publish() will read through the file in the class_listing_file_address directory which is class_listing.m publish the html files based on the
% lines written inside it. Then it will put the files in the address given by publish_options, which we have stored the directory of th e help_html
% folder inside it
publish(class_listing_file_address , publish_options);

% helpbrowser opens a MATLAB help window. In future this line should be replaced by doc
helpbrowser
% open([class_listing_file_address(1:end-1),'html']);

% remove the current directory and all its subdirectories to the Matlab path
rmpath(genpath(pwd)); 
end


%====================================================================================
function local_file_text = generate_help_for_a_folder(current_folder_adress , Help_folder_adress)

% dir lists all the content of current directory (like "ls" in linux)
folder_contents = dir;

% Now, we search for all the files inside the current path and classify them as follows:

% is_valid_folder(i) has the indexes of the files that their name does not start with '.'
% (including three directories: .[current dir] ..[upper dir] .dropbox [hidden])
% Note in MATLAB the indexes start with 1, thus, only 3 directories start with '.'
% folder_contents(i).isdir checks to see if it is directory(folder) i.e. not a file

% is_class_file(i) has the indexes of the files that thier name has size of more that 2 (if it is .m file it has at min 2 chars) and from the
% beginning of the name till before .m it has 'class' in its name
% Note: if some file is a class file it must have the ".m" extension also the function "exist" only works without the extension ".m"

% is_class_folder(i) returns indexes of the files that are folder and their name begins with '@'

% Summary:
% is_valid_folder(i) contains indexes that are folder regardless of thier names except for . .. .dropbox
% is_class_file(i) contains indexes of files that are like *class.m
% is_class_folder(i) contains indexes of folders with names like @*class

for i = 1:length(folder_contents)
    is_valid_folder(i) = folder_contents(i).isdir && folder_contents(i).name(1)~='.'; %#ok<AGROW>
    is_class_file(i) = (size(folder_contents(i).name, 2)>2) && exist(folder_contents(i).name(1:end-2), 'class'); %#ok<AGROW>
    is_class_folder(i) = folder_contents(i).isdir && folder_contents(i).name(1) == '@'; %#ok<AGROW>
end

% This command returns the name of last folder in the given address (i.e., the name of "current_folder")
[~, current_folder_name, ~] = fileparts(current_folder_adress);
% This command displays the above name in command window
disp(current_folder_name)

% strcmpi is string compare
% we check that the current_folder_name (which is the current folder) is not one of the following folders
% (that are not of interest and we don't want to generate help for them because they don't contain classes)
% if we are in one of the following folders we return empty string as the output of the method and give up the rest of method by 'return'
if strcmpi(current_folder_name,'output') || strcmpi(current_folder_name,'Code_for_testing_classes') || ...
        strcmpi(current_folder_name,'General_functions') || strcmpi(current_folder_name,'Help_htm') || ...
        strcmpi(current_folder_name,'StandAlone_code_To_produce_plots_in_paper_and_ppt') || ...
        strcmpi(current_folder_name,'Top_runs')
    local_file_text = [];
    return
end

% The follwoing returns the size of the name of current folder
current_name_len = size(current_folder_name,2);
% Then we check that the name of the folder contains 'All', if yes, for those folders we write a line in the output like %%%% FolderName Classes\n
% Note: the min(3, ) is at least 3 so that if the folder name is less than 3 chars, we could compare the name with 'All'
if strcmpi(current_folder_name(1:min(3,current_name_len)) ,  'All')
    local_file_text = [sprintf('%%%% '), current_folder_name, sprintf(' Classes\n')];
else
    local_file_text = [];
end

% In the following for loop we generate help for the "classes" [that may be class files or class folders] in the "current_folder"
% helpwin_modified(FileName) generates html help as a whole string for the inside file
for i = find(is_class_file | is_class_folder)
    % we generate help for the i-th class in the "current_folder" and put it in class_help string
    [~,class_help]=helpwin_modified(folder_contents(i).name);
    % if it is class file its name is like *class.m thus, we cut '.m'(end-2), 
    % and save the name of the desired html help file in local_help_file_name
    if is_class_file(i)
        local_help_file_name = [folder_contents(i).name(1:end-2),'_help.htm'];
    % if it is class_folder its name begins with '@' that we cut it
    elseif is_class_folder(i)
        local_help_file_name = [folder_contents(i).name(2:end),'_help.htm'];
    end
    
    % Note: We store the directory of html help file that we produced above in the local_help_html_adress
    % Help_folder_adress is always the path for Help_html folder inside the Firm toolbox directory
    % fullfile() generates the file directory that runs correctly on the platform on which it is executed 
    % (without the concern for filesep, etc on the current running machine)
    % to create a path to MATLAB and toolbox folders that does not depend on a specific platform or matlab version 'matlabroot' must be used;
    % Otherwise, the path would depend on the machine in which the program runs on
    
    % Summary: the generated html file will be stored on
    local_help_html_adress = fullfile(Help_folder_adress,local_help_file_name);
    
    % Now, open that html file and write the class_help string of html codes in it, then close the file
    [fid_local, msg] = fopen(local_help_html_adress , 'w');
    if fid_local == -1
        disp(msg)
    end
    fwrite(fid_local, class_help);
    fclose(fid_local);
    
    % Now, generate the necessary "html_code" for the "class_listing_file",
    % which inculdes the address and name of the file you have just
    % created.
    

%    strange_html_adress = ['file:///',Help_folder_adress,'\',local_help_file_name];
%            %         strange_html_adress = ['matlab:helpwin(',sprintf('\'''),Help_folder_adress,'\',local_help_file_name,sprintf('\'''),')'];
%            % some addresses may contain "space". So, we have to replace those with
%            % "%20" so that the html address is still valid.
%    space_locations = strfind(strange_html_adress,' ');
%    for i_space = space_locations
%        strange_html_adress(i_space)=[];
%        strange_html_adress=[strange_html_adress(1:i_space-1),'%20',strange_html_adress(i_space:end)];
%        space_locations = strfind(strange_html_adress,' ');
%    end
%    local_file_text = [local_file_text , [sprintf('%% * <'), strange_html_adress, sprintf(' |'), local_help_file_name(1:end-5), sprintf('|> - class explanation\n')]  ]; %#ok<AGROW>
    strange_html_adress=local_help_html_adress;
    space_locations = strfind(strange_html_adress,' ');
    for i_space = space_locations
        strange_html_adress(i_space)=[];
        strange_html_adress=[strange_html_adress(1:i_space-1),'%20',strange_html_adress(i_space:end)];
    end
    % The following line will be the part of the lines that will be written on the output of this method.
    % The starnge formattig comes from the special chars needed to produce the desired output for the final html help file
    local_file_text = [local_file_text , [sprintf('%% * <file:///'), strange_html_adress, sprintf(' |'), local_help_file_name(1:end-5), sprintf('|> - class explanation\n')]  ];  %#ok<AGROW>
end

% The following 'for loop' will run recursively the generate_help_for_a_folder() for the other folders that were not
% class-folders and generate help for the classes inside them

% insider_file_text = [];
for i = find(is_valid_folder & ~(is_class_file | is_class_folder) )
    % Following "cd" is not really needed, i.e., the "what" works even
    % without following "cd".

    % In the following line we obtain the current folder's name and cd() change directory to inside it
    % The reason for this cd is that the "helpwin" does not work without entering into the folder; then, after generating hep for that
    % subfolder we come out of that folder by cd .. and then do the same procedure for other fodlers in the loop
    subfolder = folder_contents(i).name;
    cd(subfolder);
    insider_file_text = generate_help_for_a_folder(subfolder , Help_folder_adress);
    cd ..
    % the new lines for our class_listing.m file generated during this loop will be added to the local_file_text 
    local_file_text = [local_file_text , insider_file_text]; %#ok<AGROW>
end % end of for loop

% Uncomment the following three lines if you have some data file that you want to appear in the listing.
% sprintf('%%%% Data\n%% * somedatafile.mat - its description\n');
% data_text = [];
% local_file_text = [local_file_text , data_text];

end % end of function

function produce_info_XML()
begining_text=sprintf(['<productinfo xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="optional">\n',...
   ' <?xml-stylesheet type="text/xsl"href="optional"?>\n',...
    '<!-- info.xml file for the Upslope toolbox -->\n',...
    '<!-- Author: Ali-akbar Agha-mohammadi -->\n',...
    '<!-- Copyright 2011-2012 The ???, Inc. -->\n',...
   '<!-- Release element is not used, but provides documentation -->\n',...
    '<matlabrelease>13</matlabrelease>\n',...
	'<name>FIRM Motion Planning</name>\n',...
	'<type>toolbox</type>\n',...
	'<icon>$toolbox/matlab/icons/matlabicon.gif</icon>\n',...
	'<help_location>']);
help_files_root=fullfile(pwd,'Help_html');
info_xml_root=fullfile(pwd,'info.xml');
ending_text=sprintf(['</help_location>\n',...
    '<!-- <help_location>$docroot/toolbox/FIRM</help_location>-->\n',...
    '<!-- <help_location>$docroot/toolbox/FIRM</help_location>-->\n',...
    '<!-- (Required) The name element appears in the Contents pane -->\n',...
   '<!-- (Required) The type elementidentifies your package; pick one: -->\n',...
    '<!-- matlab, toolbox, simulink, blockset, links_targets  -->\n',...
    '<!-- The icon element is used in the Start button -->\n',...
    '<!-- (Required) The help_location is relative path to help (HTML) file folder -->\n',...
'<list>	\n',...
'<listitem>\n',...
'<label>Help</label>\n',...
'<callback>doc FIRM</callback>\n',...
'<icon>$toolbox/matlab/icons/book_mat.gif</icon>\n',...
'</listitem>\n',...
'</list>\n',...
'</productinfo>'...
    ]);
the_whole_text=[begining_text,help_files_root,ending_text];
[fid2,msg] = fopen(info_xml_root, 'w'); %#ok<NASGU>
fwrite(fid2,the_whole_text);
fclose(fid2);
end

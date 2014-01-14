function typeDef( old_type , new_type )

old_file_address = which([old_type,'.m']);
[pathstr, ~, ~] = fileparts(old_file_address) ;
new_file_address = [pathstr,filesep,new_type,'.m'];
text_content = fileread(old_file_address);

modified_text_content = strrep(text_content,['classdef ',old_type], ['classdef ',new_type]);   % To update the class name in the first line
modified_text_content = strrep(modified_text_content,[old_type,'.'], [new_type,'.']); % To update all the places that 'classname.something' occurs.
%modified_text_content = strrep(modified_text_content, ['function obj = ', old_type, '(varargin)'], ['function obj = ', new_type, '(varargin)']); % To update the constructor function
modified_text_content = strrep(modified_text_content, [' ', old_type, '('], [' ', new_type, '('] ); % To update the constructor function OR to update the recursive calls to the class itself.
modified_text_content = strrep(modified_text_content, ['=', old_type, '('], ['=', new_type, '('] ); % To update the constructor function OR to update the recursive calls to the class itself.
modified_text_content = strrep(modified_text_content, ['''', old_type, ''''], ['''', new_type, '''']); % To update the class name if it is inputed to some function (for example to "isa" function) as an string.

%TextEndOfEachLine = sprintf([' %% This is a dummy class. It is ''''typedefed'''' from the oringinal class ''''', old_type, '''''. If you want to make any changes you need to change the original class.\n']);
%TextEndOfEachLine = sprintf([' %% =====']);
%modified_text_content = strrep(modified_text_content, sprintf('\n  '), TextEndOfEachLine); % To put the above text at the end of each line in this file.

% TextBeginningOfFile = sprintf([' %% This is a dummy class. It is ''''typedefed'''' from the oringinal class ''''', old_type, '''''.\n%% If you want to make any changes you need to change the original class.\n']);
% TextEndOfFile = sprintf([' %% This is a dummy class. It is ''''typedefed'''' from the oringinal class ''''', old_type, '''''.\n%% If you want to make any changes you need to change the original class.\n']);
% modified_text_content = [TextBeginningOfFile, modified_text_content, TextEndOfFile];

new_file_id = fopen(new_file_address,'w');
fwrite(new_file_id, modified_text_content);
fclose(new_file_id);

end
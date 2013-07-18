function varargout = user_GUI(varargin)
% USER_GUI MATLAB code for user_GUI.fig
%      USER_GUI, by itself, creates a new USER_GUI or raises the existing
%      singleton*.
%
%      H = USER_GUI returns the handle to a new USER_GUI or the handle to
%      the existing singleton*.
%
%      USER_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in USER_GUI.M with the given input arguments.
%
%      USER_GUI('Property','Value',...) creates a new USER_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before user_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to user_GUI_OpeningFcn via vararginan.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help user_GUI

% Last Modified by GUIDE v2.5 18-Jul-2013 16:20:03

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @user_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @user_GUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes during object creation, after setting all properties.
function SFMP_GUI_fig_tag_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SFMP_GUI_fig_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
global New_LoadFileName
handles.LoadFileName = New_LoadFileName; % The name of file, from which we want to load the parameters.
load(handles.LoadFileName,'par')

handles.old_par = par;

% Update handles structure
guidata(hObject, handles);


% --- Executes just before user_GUI is made visible.
function user_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to user_GUI (see VARARGIN)

% Choose default command line output for user_GUI
handles.output_GUI_figure_handle = hObject;

Update_GUI_fields(handles, handles.old_par); % We initialize the GUI fields using this function, based on the latest user parameters.

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes user_GUI wait for user response (see UIRESUME)
uiwait(handles.SFMP_GUI_fig_tag);


% --- Outputs from this function are returned to the command line.
function varargout = user_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output_GUI_figure_handle; % This is the handle of main GUI figure. This figure is gonna be deleted in next few lines, but this output is generated to keep the convention in Matlab that the first output in plotting something is the handle of figure.
varargout{2} = handles.output_Ok_Cancel; % This output indicates that if the user is pressed Ok or Cancel.
varargout{3} = handles.output_parameters; % This output is the main output and returns all the parameters needed in program execution.
delete(handles.SFMP_GUI_fig_tag);


% --- Executes on button press in OKbutton.
function OKbutton_Callback(hObject, eventdata, handles)
% hObject    handle to OKbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Callback called when the OK button is pressed
handles.output_Ok_Cancel ='Ok';
par = gather_parameteres_from_GUI( handles ); % This function gathers all parameters, provided by user in GUI.
handles.output_parameters = par; % If the user presses the Cancel button, we do not return any parameters.

% resume the execution of the GUI
uiresume;
guidata(hObject,handles)
% delete(handles.SFMP_GUI_fig_tag); % Although we should close the GUI
% window here, we do not do it here, because then the information stored in
% handle is get cleared. Thus, we close the window in the
% "user_GUI_OutputFcn" function, which is executed after this function,
% because we have "uiresume" here.


% --- Executes on button press in Cancelbutton.
function Cancelbutton_Callback(hObject, eventdata, handles)
% hObject    handle to Cancelbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 
% Callback called when the Cancel button is pressed
handles.output_Ok_Cancel ='Cancel';
handles.output_parameters = []; % If the user presses the Cancel button, we do not return any parameters.
uiresume;
guidata(hObject,handles)
% delete(handles.SFMP_GUI_fig_tag); % Although we should close the GUI
% window here, we do not do it here, because then the information stored in
% handle is get cleared. Thus, we close the window in the
% "user_GUI_OutputFcn" function, which is executed after this function,
% because we have "uiresume" here.


% --- Executes when user attempts to close SFMP_GUI_fig_tag.
function SFMP_GUI_fig_tag_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to SFMP_GUI_fig_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Callback called when the Close button is pressed
% Since the code for pressing the "close button" is exactly the same as the
% code for pressing the cancel button, we just call the
% "Cancelbutton_Callback" function.
Cancelbutton_Callback(handles.Cancelbutton, eventdata, handles)

% --- Executes on selection change in popupmenu_motion_model_selector.
function popupmenu_motion_model_selector_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_motion_model_selector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_motion_model_selector contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_motion_model_selector
string_list = cellstr(get(hObject,'String'));
val = get(hObject,'Value');
handles = Draw_motion_model_panel(handles,string_list{val});
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function popupmenu_motion_model_selector_CreateFcn(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
% hObject    handle to popupmenu_motion_model_selector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% This function draws the motion model panel in the GUI figure, based on
% the selected model in the "popupmenu_motion_model_selector".
function handles = Draw_motion_model_panel(handles , selected_motion_model, motion_model_par)
% following code is for initilizing the state of "pop-up" menu, based on
% the selected motion model.
string_list = get(handles.popupmenu_motion_model_selector, 'String'); % get list of possible motion models from the pop-up menu
val = find(strcmpi(string_list , selected_motion_model)); % see which motion model is chosen
set(handles.popupmenu_motion_model_selector, 'Value', val) % set the pop-up menu to the selected motion model.

% This function draws the observation model panel in the GUI figure, based on
% the selected model in the "popupmenu_observation_model_selector".
function handles = Draw_observation_model_panel(handles , selected_observation_model, observation_model_par)
% following code is for initilizing the state of "pop-up" menu, based on
% the selected motion model.
string_list = get(handles.popupmenu_observation_model_selector, 'String'); % get list of possible motion models from the pop-up menu
val = find(strcmpi(string_list , selected_observation_model)); % see which motion model is chosen
set(handles.popupmenu_observation_model_selector, 'Value', val) % set the pop-up menu to the selected motion model.

% This function draws the FIRM method panel (so far only a popup menu - not really a panel) in the GUI figure, based on
% the selected method in the "popupmenu_FIRM_method_selector".
function handles = Draw_FIRM_method_panel(handles,planning_prob_param, FIRM_method_par)
% set up the buttons for offline or online phase selection
set(handles.online_planning_phase_button,'Value',planning_prob_param.Online_phase)
% following code is for initilizing the state of "pop-up" menu, based on
% the selected FIRM method.
selected_FIRM_method = planning_prob_param.solver;
string_list = get(handles.popupmenu_FIRM_method_selector, 'String'); % get list of possible FIRM methods from the pop-up menu
val = find(strcmpi(string_list , selected_FIRM_method)); % see which FIRM method is chosen
set(handles.popupmenu_FIRM_method_selector, 'Value', val) % set the pop-up menu to the selected FIRM method.

% following code is useful if you want to give the user the freedom to set
% some parameters of the FIRM method. You have to have a "panel" for the
% FIRM method in your GUI. Then, design a edit box for each parameter and
% let the user set its value. A "commented" example has been already done
% for the "motion model panel".

% all_objects_in_FIRM_method_panel = get(handles.panel_FIRM_method,'Children');
% set(all_objects_in_FIRM_method_panel,'visible','off'); % Here, we make all the objects in the motion model invisible.
% For the rest of this code follow the convention in the design of the
% "panel" for the motion model.

function edit_unicycle_base_length_Callback(hObject, eventdata, handles)
% hObject    handle to edit_unicycle_base_length (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_unicycle_base_length as text
%        str2double(get(hObject,'String')) returns contents of edit_unicycle_base_length as a double


% --- Executes during object creation, after setting all properties.
function edit_unicycle_base_length_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_unicycle_base_length (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on key release with focus on SFMP_GUI_fig_tag or any of its controls.
function SFMP_GUI_fig_tag_WindowKeyReleaseFcn(hObject, eventdata, handles)
% hObject    handle to SFMP_GUI_fig_tag (see GCBO)
% eventdata  structure with the following fields (see FIGURE)
%	Key: name of the key that was released, in lower case
%	Character: character interpretation of the key(s) that was released
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) released
% handles    structure with handles and user data (see GUIDATA)

% For some reason, which is not known to me at this point, without this
% function when we press a keyboard key on the GUI figure it returns an
% error. So, I have to have this function here, although it is empty! I
% guess it has to do something with the "uiwait" function.


function edit_parameter_file_address_Callback(hObject, eventdata, handles)
% hObject    handle to edit_parameter_file_address (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_parameter_file_address as text
%        str2double(get(hObject,'String')) returns contents of edit_parameter_file_address as a double


% --- Executes during object creation, after setting all properties.
function edit_parameter_file_address_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_parameter_file_address (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_browse.
function pushbutton_browse_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_browse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Callback called when the icon file selection button is pressed
filespec = {'*.mat', 'MATLAB MAT files (*.mat)'};
[filename, pathname] = uigetfile(filespec, 'Pick a parameter file', 'output');

if ~isequal(filename,0)
    LoadFileName =fullfile(pathname, filename);
    set(handles.edit_parameter_file_address,'String',LoadFileName);
    load(LoadFileName, 'par')
    
    handles.LoadFileName = LoadFileName;
    handles.old_par = par;
    
    Update_GUI_fields(handles,par);
    
    % Update handles structure
    guidata(hObject, handles);
end


function Update_GUI_fields(handles,old_par)

selected_motion_model = old_par.selected_motion_model;
handles = Draw_motion_model_panel(handles , selected_motion_model); % updates the "motion_model" panel (draws the panel based on initially selected motion model.)

selected_observation_model = old_par.selected_observation_model;
handles = Draw_observation_model_panel(handles , selected_observation_model); % updates the "motion_model" panel (draws the panel based on initially selected motion model.)

planning_problem_parameters = old_par.planning_problem_param;
handles = Draw_FIRM_method_panel(handles , planning_problem_parameters); % Right now, we only have the "popup_menu"; so, there is no panel really.

guidata(gcf, handles);


% --- Executes on button press in checkbox_obstacles.
function checkbox_obstacles_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_obstacles (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_obstacles


% --- Executes on selection change in popupmenu_observation_model_selector.
function popupmenu_observation_model_selector_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_observation_model_selector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_observation_model_selector contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_observation_model_selector


% --- Executes during object creation, after setting all properties.
function popupmenu_observation_model_selector_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_observation_model_selector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox_landmarks.
function checkbox_landmarks_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_landmarks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_landmarks


function edit_eta_u_unicycle_Callback(hObject, eventdata, handles)
% hObject    handle to edit_eta_u_unicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_eta_u_unicycle as text
%        str2double(get(hObject,'String')) returns contents of edit_eta_u_unicycle as a double


% --- Executes during object creation, after setting all properties.
function edit_eta_u_unicycle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_eta_u_unicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function text_eta_u_unicycle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text_eta_u_unicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



function edit_eta_u_omni_Callback(hObject, eventdata, handles)
% hObject    handle to edit_eta_u_omni (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_eta_u_omni as text
%        str2double(get(hObject,'String')) returns contents of edit_eta_u_omni as a double


% --- Executes during object creation, after setting all properties.
function edit_eta_u_omni_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_eta_u_omni (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on selection change in popupmenu_FIRM_method_selector.
function popupmenu_FIRM_method_selector_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_FIRM_method_selector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_FIRM_method_selector contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_FIRM_method_selector


% --- Executes during object creation, after setting all properties.
function popupmenu_FIRM_method_selector_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_FIRM_method_selector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% This function is for gathering the parameters from the user through GUI.
function par_new = gather_parameteres_from_GUI(handles)


%=========== Motion Model Parameters
contents = cellstr(get(handles.popupmenu_motion_model_selector,'String')); % returns popupmenu_motion_model_selector contents as cell array
par_new.selected_motion_model = contents{get(handles.popupmenu_motion_model_selector,'Value')}; % returns selected item from popupmenu_motion_model_selector

%=========== Observation Model Parameters
contents = cellstr(get(handles.popupmenu_observation_model_selector,'String')); % returns popupmenu_motion_model_selector contents as cell array
par_new.selected_observation_model = contents{get(handles.popupmenu_observation_model_selector,'Value')}; % returns selected item from popupmenu_motion_model_selector
par_new.observation_model_parameters.interactive_OM = get(handles.checkbox_landmarks,'Value');

%=========== Planning Problem (Solver) Parameters
contents = cellstr(get(handles.popupmenu_FIRM_method_selector,'String')); % returns popupmenu_FIRM_Node_selector contents as cell array
par_new.planning_problem_param.solver = contents{get(handles.popupmenu_FIRM_method_selector,'Value')}; % returns selected item from popupmenu_FIRM_Node_selector

par_new.planning_problem_param.Offline_construction_phase = get(handles.offline_construction_phase_button,'Value');
par_new.planning_problem_param.Online_phase = get(handles.online_planning_phase_button,'Value');

%========== Simulator Parameters
contents = cellstr(get(handles.popupmenu_simulator,'String')); % returns popupmenu_motion_model_selector contents as cell array
par_new.selected_simulator = contents{get(handles.popupmenu_simulator,'Value')}; % returns selected item from popupmenu_motion_model_selector

par_new.sim.intractive_obst = get(handles.checkbox_obstacles,'Value');  % returns toggle state of "checkbox_obstacles"
par_new.sim.interactive_PRM = get(handles.checkbox_manual_PRM_2D_nodes,'Value');  % returns toggle state of "checkbox_manual_PRM_2D_nodes"




% --- Executes on button press in checkbox_manual_PRM_2D_nodes.
function checkbox_manual_PRM_2D_nodes_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_manual_PRM_2D_nodes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_manual_PRM_2D_nodes


% --- Executes on selection change in popupmenu_simulator.
function popupmenu_simulator_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_simulator (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_simulator contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_simulator


% --- Executes during object creation, after setting all properties.
function popupmenu_simulator_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_simulator (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

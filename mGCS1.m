function varargout = mGCS1(varargin)
% MGCS1 MATLAB code for mGCS1.fig
%      MGCS1, by itself, creates a new MGCS1 or raises the existing
%      singleton*.
%
%      H = MGCS1 returns the handle to a new MGCS1 or the handle to
%      the existing singleton*.
%
%      MGCS1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MGCS1.M with the given input arguments.
%
%      MGCS1('Property','Value',...) creates a new MGCS1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before mGCS1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to mGCS1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help mGCS1

% Last Modified by GUIDE v2.5 30-Jul-2017 09:03:21

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @mGCS1_OpeningFcn, ...
                   'gui_OutputFcn',  @mGCS1_OutputFcn, ...
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


% --- Executes just before mGCS1 is made visible.
function mGCS1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to mGCS1 (see VARARGIN)

% Choose default command line output for mGCS1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes mGCS1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);

fontSize = 10; % Whatever you want.
axes(handles.grphPos);
xlabel('North [m]', 'FontSize', fontSize);
ylabel('East [m]', 'FontSize', fontSize);
handles.xVal = 0;
handles.yVal = 0;
guidata(hObject, handles);



% --- Outputs from this function are returned to the command line.
function varargout = mGCS1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

global mTimer;
mTimer = timer('executionMode','fixedRate', 'period', 0.03, ...
    'TimerFcn', @(hObject, eventdata)timerCallback(hObject, eventdata, handles));

function timerCallback(hObject, eventdata, handles)
%     handles = guidata(hObject);
    status = get_param(bdroot,'simulationstatus');
    if strcmp(status,'running')    
        value1 = get_param([bdroot '/Out1'],'UserData');
        set(handles.tbxPitch, 'String', value1);
        value2 = get_param([bdroot '/Out2'],'UserData');
        set(handles.tbxYaw, 'String', value2);
        if size(handles.xVal,2) > 50000
            handles.xVal(1)=[];
        end 
        if size(handles.yVal,2) > 50000
            handles.yVal(1)=[];
        end 
        handles.xVal(end + 1) = value1;
        handles.yVal(end + 1) = value2;
%         guidata(hObject, handles);
%         stem(handles.grphPos,[value1, value2] ,'Marker','.','LineStyle','-');
    end
    axes(handles.grphPos);
    plot(handles.grphPos, handles.xVal,handles.yVal);
%     guidata(hObject, handles);
    
function tbxRoll_Callback(hObject, eventdata, handles)
% hObject    handle to tbxRoll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tbxRoll as text
%        str2double(get(hObject,'String')) returns contents of tbxRoll as a double
gain = get(hObject,'String');
handles.gain = gain;
set_param([bdroot '/Gain'],'Gain',gain)
status = get_param(bdroot,'simulationstatus');
if strcmp(status,'running')    
    set_param(bdroot, 'SimulationCommand', 'Update')
end

%guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function tbxRoll_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbxRoll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tbxPitch_Callback(hObject, eventdata, handles)
% hObject    handle to tbxPitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tbxPitch as text
%        str2double(get(hObject,'String')) returns contents of tbxPitch as a double


% --- Executes during object creation, after setting all properties.
function tbxPitch_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbxPitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tbxYaw_Callback(hObject, eventdata, handles)
% hObject    handle to tbxYaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tbxYaw as text
%        str2double(get(hObject,'String')) returns contents of tbxYaw as a double


% --- Executes during object creation, after setting all properties.
function tbxYaw_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbxYaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btnAttCmd.
function btnAttCmd_Callback(hObject, eventdata, handles)
% hObject    handle to btnAttCmd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in btnSetHvr.
function btnSetHvr_Callback(hObject, eventdata, handles)
% hObject    handle to btnSetHvr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Update the model's gain value
status = get_param(bdroot,'simulationstatus');
if strcmp(status,'stopped')
    set_param(bdroot,'simulationcommand','start')
end
assignin('base','gs_handles',handles)
assignin('base','setHvr_hObject',handles.btnSetHvr)
global mTimer;
start(mTimer);

function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btnSendPosCmd.
function btnSendPosCmd_Callback(hObject, eventdata, handles)
% hObject    handle to btnSendPosCmd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in btnSetHome.
function btnSetHome_Callback(hObject, eventdata, handles)
% hObject    handle to btnSetHome (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%f = figure();

%grid(ax, 'on');

function figure1_DeleteFcn(hObject, eventdata, handles, varargin)
% --- Executes during object deletion, before destroying properties.
global mTimer;
stop(mTimer);
delete(timer);
clear timer;
status = get_param(bdroot,'simulationstatus');
if strcmp(status,'running') 
    set_param(bdroot, 'SimulationCommand', 'Stop')
end
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

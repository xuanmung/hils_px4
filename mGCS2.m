function varargout = mGCS2(varargin)
% MGCS2 MATLAB code for mGCS2.fig
%      MGCS2, by itself, creates a new MGCS2 or raises the existing
%      singleton*.
%
%      H = MGCS2 returns the handle to a new MGCS2 or the handle to
%      the existing singleton*.
%
%      MGCS2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MGCS2.M with the given input arguments.
%
%      MGCS2('Property','Value',...) creates a new MGCS2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before mGCS2_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to mGCS2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help mGCS2

% Last Modified by GUIDE v2.5 31-Jul-2017 13:20:16

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @mGCS2_OpeningFcn, ...
                   'gui_OutputFcn',  @mGCS2_OutputFcn, ...
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


% --- Executes just before mGCS2 is made visible.
function mGCS2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to mGCS2 (see VARARGIN)

% Choose default command line output for mGCS2
handles.output = hObject;

set(handles.btnSync,'visible','off');

handles.serialStatus = 'none';

handles.rollSetpoint = 0;
handles.pitchSetpoint = 0;
handles.yawSetpoint = 0;

handles.northSetpoint = 0;
handles.eastSetpoint = 0;

handles.rollFb = 0;
handles.pitchFb = 0;
handles.yawFb = 0;
handles.pFb = 0;
handles.qFb = 0;
handles.rFb = 0;

handles.northFb = 0;
handles.eastFb = 0;

global mSerial;
global mTimer;
% mTimer = timer('ExecutionMode', 'fixedRate',...
%     'Period', 0.001, 'TimerFcn',{@update_feedback, handles});

mTimer = timer('executionMode','fixedRate', 'period', 0.1, ...
    'TimerFcn', @(hObject, eventdata)timerCallback(hObject, eventdata, handles));

handles.timerStatus = 'stopped';

% Update handles structure
guidata(hObject, handles);

% sim('HILS_Px4');

% UIWAIT makes mGCS2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function timerCallback(hObject, eventdata, handles)
    handles = guidata(handles.figure1);
    handles.rollFb
    
    set_param([bdroot '/Gain'],'Gain', num2str(handles.rollFb))
    status = get_param(bdroot,'simulationstatus');
    if strcmp(status,'running')    
%         handles.northFb = get_param([bdroot '/xOut'],'UserData');
%         handles.eastFb = get_param([bdroot '/yOut'],'UserData');
%         
        
        set(handles.tbxNorthFb, 'string', '123');
        set(handles.tbxEastFb, 'string', num2str(handles.eastFb))
%         set_param(bdroot, 'SimulationCommand', 'Update')
    end
    guidata(handles.figure1, handles);
    
    


% --- Outputs from this function are returned to the command line.
function varargout = mGCS2_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function tbxRoll_Callback(hObject, eventdata, handles)
% hObject    handle to tbxRoll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tbxRoll as text
%        str2double(get(hObject,'String')) returns contents of tbxRoll as a double
rollSetpoint = str2double(get(hObject, 'String'));
if ~isnumeric (rollSetpoint)
    warndlg('Input must be numerical');
else
    handles.rollSetpoint = rollSetpoint;
end
guidata(hObject, handles); 

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
handles.pitchSetpoint = str2double(get(hObject, 'String'));
guidata(hObject, handles);

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
handles.yawSetpoint = str2double(get(hObject, 'String'));
guidata(hObject, handles);

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


% --- Executes on button press in btnSendAttitude.
function btnSendAttitude_Callback(hObject, eventdata, handles)
% hObject    handle to btnSendAttitude (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~strcmp(handles.serialStatus,'open')
    warndlg('No Connection to vehicle');
else
%     handles.rollSetpoint = str2double(get(hObject.tbxRoll, 'String'));
    buffer = strcat(num2str(handles.rollSetpoint),'p789\n');
    disp(num2str(handles.rollSetpoint));
    global mSerial;
    fprintf(mSerial,'%s', buffer);
end

% --- Executes on button press in btnSetHover.
function btnSetHover_Callback(hObject, eventdata, handles)
% hObject    handle to btnSetHover (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.tbxRoll, 'string', 0); 
set(handles.tbxPitch, 'string', 0);
set(handles.tbxYaw, 'string', 0);
handles.rollSetpoint = 0;
handles.pitchSetpoint = 0;
handles.yawSetpoint = 0;
guidata(hObject, handles);


function tbxEastFb_Callback(hObject, eventdata, handles)
% hObject    handle to tbxEastFb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tbxEastFb as text
%        str2double(get(hObject,'String')) returns contents of tbxEastFb as a double


% --- Executes during object creation, after setting all properties.
function tbxEastFb_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbxEastFb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tbxNorthFb_Callback(hObject, eventdata, handles)
% hObject    handle to tbxNorthFb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tbxNorthFb as text
%        str2double(get(hObject,'String')) returns contents of tbxNorthFb as a double


% --- Executes during object creation, after setting all properties.
function tbxNorthFb_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbxNorthFb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tbxNorthSet_Callback(hObject, eventdata, handles)
% hObject    handle to tbxNorthSet (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tbxNorthSet as text
%        str2double(get(hObject,'String')) returns contents of tbxNorthSet as a double
handles.northSetpoint = str2double(get(hObject, 'String'));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function tbxNorthSet_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbxNorthSet (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tbxEastSet_Callback(hObject, eventdata, handles)
% hObject    handle to tbxEastSet (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tbxEastSet as text
%        str2double(get(hObject,'String')) returns contents of tbxEastSet as a double
handles.eastSetpoint = str2double(get(hObject, 'String'));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function tbxEastSet_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbxEastSet (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btnSendPos.
function btnSendPos_Callback(hObject, eventdata, handles)
% hObject    handle to btnSendPos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.tbxNorthFb, 'string', -890);
if ~strcmp(handles.serialStatus,'open')
    msbx = warndlg('No Connection to vehicle');
else
    
end


% --- Executes on button press in btnSetHome.
function btnSetHome_Callback(hObject, eventdata, handles)
% hObject    handle to btnSetHome (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.tbxNorthSet, 'string', 0);
set(handles.tbxEastSet, 'string', 0);
handles.northSetpoint = 0;
handles.eastSetpoint = 0;
guidata(hObject, handles);


    


function tbxComPort_Callback(hObject, eventdata, handles)
% hObject    handle to tbxComPort (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tbxComPort as text
%        str2double(get(hObject,'String')) returns contents of tbxComPort as a double

handles.comPort = get(hObject,'String');
% handles.comPort = comPort;
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function tbxComPort_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbxComPort (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btnConnect.
function btnConnect_Callback(hObject, eventdata, handles)
% hObject    handle to btnConnect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
        set(handles.btnConnect,'visible','off');
        set(handles.btnDisconnect,'visible','on');
        global mSerial; 
        mSerial = serial('COM25');
        set(mSerial, 'BaudRate', 57600);
        mSerial.BytesAvailableFcn = @(handles, eventdata)update_feedback(hObject, ...
            eventdata);
        mSerial.BytesAvailableFcnCount = 42;
        mSerial.BytesAvailableFcnMode = 'byte';
        guidata(hObject, handles);
        
        status = get_param(bdroot,'simulationstatus');
        if strcmp(status,'stopped')
            set_param(bdroot,'simulationcommand','start')
        end
%         assignin('base','gs_handles',handles)
       
    try
        fopen(mSerial);
        handles.serialStatus = 'open';
        guidata(hObject, handles);
        msbx = msgbox('Vehicle connected');
        set(handles.btnSync,'visible','on');
    catch
        %delete(handles.mSerial);
        msbx = warndlg('Connection failed');
        handles.serialStatus = 'closed';
        set(handles.btnConnect,'visible','on');
        set(handles.btnDisconnect,'visible','off');
        
        status = get_param(bdroot,'simulationstatus');
        if strcmp(status,'running') 
            set_param(bdroot, 'SimulationCommand', 'Stop')
        end

    end
    guidata(hObject, handles);
    
function update_feedback(hObject,eventdata)
    handles = guidata(hObject);
    global mSerial;
    
%     heloo= receivedBuffer(1)
    flag = fscanf(mSerial, '%s',1);
    while (~strcmp(flag, 'a'))
        flag = fscanf(mSerial, '%s',1);
    end
     
    receivedBuffer = fscanf(mSerial, '%s',41);
    handles.receivedBuffer = receivedBuffer;

    if (strcmp(receivedBuffer(7), 'b') && strcmp(receivedBuffer(14), 'c') ...
            && strcmp(receivedBuffer(21), 'p') && strcmp(receivedBuffer(28), 'q') ...
            && strcmp(receivedBuffer(35), 'r'))
   
        handles.rollFb = str2double(receivedBuffer(1:6))/100.0;
        handles.pitchFb = str2double(receivedBuffer(8:13))/100.0;
        handles.yawFb = str2double(receivedBuffer(15:20))/100.0;
        handles.pFb = str2double(receivedBuffer(22:27))/100.0;
        handles.qFb = str2double(receivedBuffer(29:34))/100.0;
        handles.rFb = str2double(receivedBuffer(36:41))/100.0;
        guidata(handles.figure1, handles);
    else
        %do nothing
    end
    
    %guidata(hObject, handles);
     
    set(handles.tbxRollFb, 'string', handles.rollFb);
    set(handles.tbxPitchFb, 'string', handles.pitchFb);
    set(handles.tbxYawFb, 'string', handles.yawFb);
    guidata(hObject, handles);

    
% --- Executes on button press in btnDisconnect.
function btnDisconnect_Callback(hObject, eventdata, handles)
% hObject    handle to btnDisconnect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%     if (handles.mSerial.Status == 'open')
    global mSerial;
    if strcmp(handles.serialStatus, 'open')
        fclose(mSerial); 
    end
    handles.serialStatus = 'closed';
%     end
    delete(mSerial); 
    handles.serialStatus = 'none';
    
    global mTimer;
    stop(mTimer);
    handles.timerStatus = 'stopped';
    
    status = get_param(bdroot,'simulationstatus');
    if strcmp(status,'running') 
        set_param(bdroot, 'SimulationCommand', 'Stop')
    end

    set(handles.btnConnect,'visible','on');
    set(handles.btnDisconnect,'visible','off');
    set(handles.btnSync,'visible','off');
    guidata(hObject, handles);
    



function tbxRollFb_Callback(hObject, eventdata, handles)
% hObject    handle to tbxRollFb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tbxRollFb as text
%        str2double(get(hObject,'String')) returns contents of tbxRollFb as a double
%     set_param([bdroot '/Gain'],'Gain',get(hObject,'String'))
%     status = get_param(bdroot,'simulationstatus');
%     if strcmp(status,'running')    
%         set_param(bdroot, 'SimulationCommand', 'Update')
%     end

% --- Executes during object creation, after setting all properties.
function tbxRollFb_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbxRollFb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function tbxPitchFb_Callback(hObject, eventdata, handles)
% hObject    handle to tbxPitchFb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tbxPitchFb as text
%        str2double(get(hObject,'String')) returns contents of tbxPitchFb as a double


% --- Executes during object creation, after setting all properties.
function tbxPitchFb_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbxPitchFb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function tbxYawFb_Callback(hObject, eventdata, handles)
% hObject    handle to tbxYawFb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tbxYawFb as text
%        str2double(get(hObject,'String')) returns contents of tbxYawFb as a double


% --- Executes during object creation, after setting all properties.
function tbxYawFb_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbxYawFb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object deletion, before destroying properties.
function figure1_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global mSerial;
if strcmp(handles.serialStatus, 'open')
    warndlg('The serial port will be closed automatically.');
    fclose(mSerial); 
end
delete(mSerial); 
clear mSerial;

global mTimer;
stop(mTimer);
delete(mTimer);
clear mTimer;
handles.timerStatus = 'none';

status = get_param(bdroot,'simulationstatus');
if strcmp(status,'running') 
    set_param(bdroot, 'SimulationCommand', 'Stop')
end


% --- Executes on button press in btnSync.
function btnSync_Callback(hObject, eventdata, handles)
% hObject    handle to btnSync (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if strcmp(handles.timerStatus, 'stopped')
    global mTimer;
    start(mTimer);
    handles.timerStatus = 'running';
    guidata(hObject, handles);
end

function varargout = readFT(varargin)
% READFT MATLAB code for readFT.fig
%      READFT, by itself, creates a new READFT or raises the existing
%      singleton*.
%
%      H = READFT returns the handle to a new READFT or the handle to
%      the existing singleton*.
%
%      READFT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in READFT.M with the given input arguments.
%
%      READFT('Property','Value',...) creates a new READFT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before readFT_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to readFT_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help readFT

% Last Modified by GUIDE v2.5 02-Aug-2017 11:29:10

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @readFT_OpeningFcn, ...
                   'gui_OutputFcn',  @readFT_OutputFcn, ...
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


% --- Executes just before readFT is made visible.
function readFT_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to readFT (see VARARGIN)

% Choose default command line output for readFT
handles.output = hObject;

handles.serialStatus = 'none';
handles.ftBuffer = '';
handles.ftData = zeros(6, 1);
handles.ftStatus = 'none';
handles.ftTimerStatus = 'none';

global ftTimer;
ftTimer = timer('executionMode','fixedRate', 'period', 0.02, ...
    'TimerFcn', @(hObject, eventdata)ftTimerCallback(hObject,...
    eventdata, handles));

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes readFT wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function ftTimerCallback(hObject, eventdata, handles)
    mhObject = handles.figure1;
    handles = guidata(mhObject);
    global mSerial1;
    if strcmp(handles.serialStatus, 'open')
        if strcmp(handles.ftStatus, 'none') ...
                && strcmp(handles.ftTimerStatus, 'running')
            fprintf(mSerial1, '%s',char(20));
%             fprintf(mSerial1, '%s', char(13));
            handles.ftStatus = 'ready';
        end
        if mSerial1.BytesAvailable() ...
                && strcmp(handles.ftStatus, 'ready')
            scanBuffer = fscanf(mSerial1, '%s');
            handles.ftStatus = 'none';
            scanBufferLen = length(scanBuffer);
            tmpBuff = '';
            comma = 0;
            if scanBufferLen > 0
                for i=1:1:scanBufferLen
                    flag = scanBuffer(i);
                    if (strcmp(flag, char(1))|| strcmp(flag, char(6))||...
                            strcmp(flag, char(96))||strcmp(flag, char(32))...
                            ||strcmp(flag, char(10))||strcmp(flag, 'Q')...
                            ||strcmp(flag, 'R')||strcmp(flag, '>'))
                        % >> ignore
                    else
                        if strcmp(flag, ',')
                            comma = comma + 1;
                        end
                        
                        %save flag into tmpBuff
                        tmpBuff = strcat(tmpBuff, scanBuffer(i)); 
                    end
                end
            end
            if ~isempty(tmpBuff)
                ftBuffer = tmpBuff;
                
%                 dec2hex(ftBuffer);
                if strcmp(ftBuffer(1),'0') && comma == 6
                    ftBuffer
                    commaPtr = [];
                    ftBuffLen = length(ftBuffer);
                    for i=1:1:ftBuffLen
                        if strcmp(ftBuffer(i), ',')
                            commaPtr(end+1) = i;
                        end
                    end
                    for i=1:1:comma-1
                        handles.ftData(i) = str2double(ftBuffer(...
                            commaPtr(i)+1 : commaPtr(i+1)-1));
                    end
                    handles.ftData(comma) = str2double(ftBuffer(...
                        commaPtr(comma)+1 : ftBuffLen));
%                     guidata(hObject, handles);
                    handles.ftData(1) = handles.ftData(1) / 20.0;
                    handles.ftData(2) = -handles.ftData(2) / 20.0;
                    handles.ftData(3) = -handles.ftData(3) / 1.6;
                    handles.ftData(4) = handles.ftData(4) / 400.0;
                    handles.ftData(5) = -handles.ftData(5) / 400.0;
                    handles.ftData(6) = -handles.ftData(6) / 400.0;
                    handles.ftData
                end  
            end
        end
        
    end
    guidata(mhObject, handles);


% --- Outputs from this function are returned to the command line.
function varargout = readFT_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in btnConn.
function btnConn_Callback(hObject, eventdata, handles)
% hObject    handle to btnConn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global mSerial1;
    mSerial1 = serial('COM7');
    set(mSerial1, 'BaudRate', 57600);
%     mSerial1.BytesAvailableFcn = @(handles, eventdata)update_ft(hObject, ...
%         eventdata);
%     mSerial1.BytesAvailableFcnCount = 1;
%     mSerial1.BytesAvailableFcnMode = 'byte';
    set(mSerial1, 'FlowControl', 'software');
%     set(mSerial1, 'Terminator', 'CR');

try 
    if ~strcmp(handles.serialStatus, 'open')
        fopen(mSerial1);
    end
    handles.serialStatus = 'open';
    guidata(hObject, handles);
    msgbox('FT connected');
catch
    warndlg('Connection to FT sensor system failed');
    handles.serialStatus = 'closed';
    guidata(hObject, handles);
end


%--- Update FT sensor data    
function update_ft(hObject, eventdata)
    handles = guidata(hObject);
    global mSerial1;

%     if strcmp(handles.ftStatus, 'ready')
%         flag = fscanf(mSerial1, '%c', 1); 
%     end
%     while (strcmp(flag, char(1))|| strcmp(flag, char(6))||...
%                     strcmp(flag, char(96))||strcmp(flag, char(32))...
%                     ||strcmp(flag, char(10))||strcmp(flag, 'Q')...
%                     ||strcmp(flag, 'R'))
%         flag = fscanf(mSerial1, '%c', 1); 
%     end
                
%     ftBuffer = '';
%     if strcmp(handles.ftStatus, 'ready')
%         ftBuffer = fscanf(mSerial1, '%c');
%         handles.ftStatus = 'none';
%         guidata(hObject, handles);
%     end
%     ftBufferLen = length(ftBuffer);
%     if ftBufferLen >= 50
%         ftBuffer
%     end
    
%     if ftBufferLen > 0
%         for i = 1:1:ftBufferLen
%             if (strcmp(flag, char(1))|| strcmp(flag, char(6))||...
%                     strcmp(flag, char(96))||strcmp(flag, char(32))...
%                     ||strcmp(flag, char(10))||strcmp(flag, 'Q')...
%                     ||strcmp(flag, 'R'))
%                 ftBuffer(i) = [];            
%             end
%         end
%         if strcmp(flag, '0')
%             tingting = 1
%             comma = zeros(6);
%             commaCount = 0;
%             for i = 1:1:ftBufferLen
%                 if strcmp(ftBuffer(i), ',')
%                     commaCount = commaCount + 1;
%                     comma(commaCount) = i;
%                 end
%             end
% %                 commaCount
%             if commaCount == 6
%                 for i=1:1:commaCount-1
%                     handles.ftData(i) = str2double(...
%                         ftBuffer(comma(i)+1:comma(i+1)-1));
%                 end
%                 handles.ftData(commaCount) = str2double(...
%                         ftBuffer(comma(commaCount)+1:ftBufferLen));
% %                     guidata(hObject, handles); 
% %                     fx = handles.ftData(1)/20.0;
% %                     fy = handles.ftData(2)/20.0;
%                 fz = handles.ftData(3)/10.0;
%     % 
%                 set(handles.tbxFz, 'String', num2str(fz));
%     %             tx = str2double(ftBuffer(30:37))/400.0;
%     %             ty = str2double(ftBuffer(39:46))/400.0;
%     %             tz = str2double(ftBuffer(48:55))/400.0;
%             end
%         end
%     end
         
    
%%    
%     flag = fscanf(mSerial1, '%c', 1);
%     
%     if strcmp(flag, char(13))
%         ftBufferLen = length(handles.ftBuffer);
%         if ftBufferLen > 0
%             handles.ftBuffer
%             if strcmp(handles.ftBuffer(1), '0')
%                 comma = zeros(6);
%                 commaCount = 0;
%                 for i = 1:1:ftBufferLen
%                     if strcmp(handles.ftBuffer(i), ',')
%                         commaCount = commaCount + 1;
%                         comma(commaCount) = i;
%                     end
%                 end
% %                 commaCount
%                 if commaCount == 6
%                     for i=1:1:commaCount-1
%                         handles.ftData(i) = str2double(...
%                             handles.ftBuffer(comma(i)+1:comma(i+1)-1));
%                     end
%                     handles.ftData(commaCount) = str2double(...
%                             handles.ftBuffer(comma(commaCount)+1:ftBufferLen));
% %                     guidata(hObject, handles); 
%                     fx = handles.ftData(1)/20.0;
%                     fy = handles.ftData(2)/20.0;
%                     fz = handles.ftData(3)/10.0;
%         % 
%                     set(handles.tbxFz, 'String', num2str(fz));
%         %             tx = str2double(ftBuffer(30:37))/400.0;
%         %             ty = str2double(ftBuffer(39:46))/400.0;
%         %             tz = str2double(ftBuffer(48:55))/400.0;
%                 end
%             end
%         end
%         handles.ftBuffer = '';
% %         guidata(hObject, handles);
%     else
%         if (strcmp(flag, char(1))||strcmp(flag, char(96))...
%                 ||strcmp(flag, char(32))||strcmp(flag, char(10))...
%                 ||strcmp(flag, 'Q')||strcmp(flag, 'S'))
%             %do nothing
%             %do not add useless characters into the ftBuffer
%         else
% %             flag
%             ftBuffer = handles.ftBuffer;
%             handles.ftBuffer = strcat(ftBuffer, flag);
%         end
%     end
%%
%     guidata(hObject, handles);

    

function tbxFz_Callback(hObject, eventdata, handles)
% hObject    handle to tbxFz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tbxFz as text
%        str2double(get(hObject,'String')) returns contents of tbxFz as a double


% --- Executes during object creation, after setting all properties.
function tbxFz_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbxFz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btnDisconn.
function btnDisconn_Callback(hObject, eventdata, handles)
% hObject    handle to btnDisconn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    global ftTimer;
    if strcmp(handles.ftTimerStatus, 'running')
        stop(ftTimer);
    end
    handles.ftTimerStatus = 'stopped';
    
    global mSerial1;
    if strcmp(handles.serialStatus, 'open')...
            && ~strcmp(handles.ftTimerStatus, 'running')
        fprintf(mSerial1, '%s', char(13));
        fclose(mSerial1); 
    end
    handles.serialStatus = 'closed';
%     end
    delete(mSerial1); 
    handles.serialStatus = 'none';
    handles.ftStatus = 'none';    
    guidata(hObject, handles);


% --- Executes during object deletion, before destroying properties.
function figure1_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    global ftTimer;
    if strcmp(handles.ftTimerStatus, 'running')
        stop(ftTimer);
    end
    delete(ftTimer);
    handles.ftTimerStatus = 'stopped';
    
    global mSerial1;
    if strcmp(handles.serialStatus, 'open')
        fclose(mSerial1); 
    end
    handles.serialStatus = 'closed';
    %     end
    delete(mSerial1); 
    handles.serialStatus = 'none';
        
    guidata(hObject, handles);


% --- Executes on button press in btnSync.
function btnSync_Callback(hObject, eventdata, handles)
% hObject    handle to btnSync (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% global mSerial1;
% if strcmp(handles.serialStatus, 'open')
% %     fprintf(mSerial1, '%s','CB 57600');
%     fprintf(mSerial1, '%s','QR');
%     fprintf(mSerial1, '%s', char(13));
%     handles.ftStatus = 'ready';
%     guidata(hObject, handles);
% else
%     warndlg('No connection to FT');
% end

global ftTimer;

if ~strcmp(handles.ftTimerStatus, 'running')
    start(ftTimer);
end
handles.ftTimerStatus = 'running';
guidata(hObject, handles);





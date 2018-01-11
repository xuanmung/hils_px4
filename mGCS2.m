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

% Last Modified by GUIDE v2.5 18-Dec-2017 16:19:16

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

fontSize = 8.5; % Whatever you want.
axes(handles.grphPos);
xlabel('East [m]', 'FontSize', fontSize);
ylabel('North [m]', 'FontSize', fontSize);
% set(handles.grphPos, 'XLim',[-100,100]);
% set(handles.grphPos, 'YLim',[-100, 100]);
set(handles.lbly,'visible','off');
% set(handles.lblx,'visible','off');

set(handles.btnSync,'visible','off');

handles.serialStatus = 'none';

handles.rollSetpoint    = 0;
handles.pitchSetpoint   = 0;
handles.yawSetpoint     = 0;

handles.northSetpoint   = 0;
handles.eastSetpoint    = 0;
handles.heightSetpoint  = 0;

handles.rollFb  = 0;
handles.pitchFb = 0;
handles.yawFb   = 0;
handles.pFb     = 0;
handles.qFb     = 0;
handles.rFb     = 0;

handles.northFb = 0;
handles.eastFb  = 0;
handles.downFb  = 0;

handles.k1Pos   = 0;
handles.k2Pos   = 0;

handles.taskMng = 0;
handles.plotMng = 0;
handles.serialMng = 0;
handles.ftMng   = 0;

handles.btnDownSendAtt = 0;
handles.btnDownSendPos = 0;
handles.btnDownResetPosControl = 1;

global mSerial;
global mTimer;
% mTimer = timer('ExecutionMode', 'fixedRate',...
%     'Period', 0.001, 'TimerFcn',{@update_feedback, handles});

mTimer = timer('executionMode','fixedRate', 'period', 0.01, ...
    'TimerFcn', @(hObject, eventdata)timerCallback(hObject, eventdata, handles));

handles.timerStatus = 'stopped';

% FT controller
global ftSerial;
handles.ftSerialStatus = 'none';
handles.ftBuffer = '';
handles.ftData = zeros(6, 1);
handles.fbFz = 0;
handles.ftStatus = 'none';
handles.ftTimerStatus = 'none';
handles.prevTic = 0;
handles.fbFz_filtered = 0;
% fileID = fopen('fbFz.txt','w');
% fclose(fileID);  

handles.fbStatus = 'none';

global ftTimer;
ftTimer = timer('executionMode','fixedRate', 'period', 0.005, ...
    'TimerFcn', @(hObject, eventdata)ftTimerCallback(hObject,...
    eventdata, handles));
%

% Update handles structure
guidata(hObject, handles);

% sim('HILS_Px4');

% UIWAIT makes mGCS2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);

%
function ftTimerCallback(hObject, eventdata, handles)
    mhObject = handles.figure1;
    handles = guidata(mhObject);
    
    handles.taskMng = handles.taskMng + 1;
    
    global ftSerial;
    global mSerial;
    
    if (handles.taskMng == 1)
        
        %%
    handles.ftMng = handles.ftMng + 1;
    
    if (handles.ftMng == 1)
        if strcmp(handles.ftSerialStatus, 'open')
            if strcmp(handles.ftStatus, 'none') ...
                    && strcmp(handles.ftTimerStatus, 'running')
                fprintf(ftSerial, '%s',char(20));
    %             fprintf(ftSerial, '%s', char(13));
                handles.ftStatus = 'ready';
            end
            if ftSerial.BytesAvailable() ...
                    && strcmp(handles.ftStatus, 'ready')
                scanBuffer = fscanf(ftSerial, '%s');
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
                    if strcmp(ftBuffer(1),'0') && comma == 6
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
                        handles.ftData(1) = handles.ftData(1) / 160.0;
                        handles.ftData(2) = handles.ftData(2) / 160.0;
                        handles.ftData(3) = - handles.ftData(3) / 160.0;
                        handles.ftData(4) = handles.ftData(4) / 3200.0;
                        handles.ftData(5) = -handles.ftData(5) / 3200.0;
                        handles.ftData(6) = -handles.ftData(6) / 3200.0;
%                         handles.ftData(1) = handles.ftData(1)- 9.7;
%                         handles.ftData(2) = handles.ftData(2)- 6.0;
                        fbFz = handles.ftData(3)- 14;
%                         fckingData1 = handles.ftData(1);
%                         fckingData2 = handles.ftData(2);
                        
%                         fileID = fopen('fbFz.txt','a');
                                          
%                         if handles.prevTic == 1
%                             toc
%                         end
%                         tic
%                         handles.prevTic = 1;
                        
                        alpha = 0; % no filter % cut-off freq = 1.5Hz, sapling time = 0.1s;
                        fbFz_filtered = handles.fbFz_filtered;
                        fbFz_filtered = alpha * fbFz_filtered + ...
                                                    (1-alpha) * fbFz;
                          handles.fbFz_filtered = fbFz_filtered;                      
%                         fprintf(fileID, strcat(num2str(fbFz), '\t', ...
%                                 num2str(handles.fbFz_filtered), '\n'));
%                         fclose(fileID);
%                         assignin('base','fbFz',handles.fbFz);
%                         fckingData
                    end  
                end
            end        
        end

%             buffer = strcat('fz', num2str_norm(handles.fbFz_filtered * 100, 5)); %, ...
% %                             'k', '1', num2str_norm(handles.ftData(1)*100, 5), ... 
% %                             'k','2', num2str_norm(handles.ftData(2)*100, 5));
%             fprintf(mSerial,'%s', buffer);
        
        handles.ftMng = 0;
    end
    elseif(handles.taskMng == 2)
%     if(handles.taskMng == 2)
    %%
    %%
        fprintf(mSerial,'%s', 's');        
        handles.fbStatus = 'ready';
        
        if handles.btnDownResetPosControl == 1
            buffer = 'r';
            fprintf(mSerial,'%s', buffer);
            handles.btnDownResetPosControl = 0;
        end
        
        if handles.btnDownSendAtt == 1
            buffer = strcat('a', num2str_norm(handles.rollSetpoint * 100 ,5), ...
                'b', num2str_norm(handles.pitchSetpoint * 100, 5),...
                'c', num2str_norm(handles.yawSetpoint * 100, 5));
            fprintf(mSerial,'%s', buffer);
            handles.btnDownSendAtt = 0;
        end
        
        if handles.btnDownSendPos == 1
            buffer = strcat('l', num2str_norm(handles.northSetpoint, 5), ...
                'i', num2str_norm(handles.eastSetpoint, 5), ...
                'd', num2str_norm(handles.heightSetpoint, 5), ...
                'k', '1', num2str_norm(handles.k1Pos*100, 5), ... 
                'k','2', num2str_norm(handles.k2Pos*100, 5));
%             buffer = strcat('l', num2str_norm(handles.northSetpoint, 5), ...
%                 'i', num2str_norm(handles.eastSetpoint, 5), ...
%                 'd', num2str_norm(handles.heightSetpoint, 5));
            fprintf(mSerial,'%s', buffer);
            handles.btnDownSendPos = 0;
        end
    %%
    elseif (handles.taskMng == 3)
        if strcmp(handles.serialStatus,'open') ...
            && strcmp(handles.ftTimerStatus, 'running')
        
            buffer = strcat('fz', num2str_norm(handles.fbFz_filtered * 100, 5)); %, ...
%                             'k', '1', num2str_norm(handles.ftData(1)*100, 5), ... 
%                             'k','2', num2str_norm(handles.ftData(2)*100, 5));
            fprintf(mSerial,'%s', buffer);
        end     
        
%         handles.hFb = hFb;
        
        if size(handles.northFb, 2) > 5000
            handles.northFb(1) = [];
        end
        if size(handles.eastFb, 2) > 5000
            handles.eastFb(1) = [];
        end
%         handles.northFb(end+1) = northFb;
%         handles.eastFb(end+1) = eastFb; 
        handles.northFb(end+1) = handles.pFb;
        handles.eastFb(end+1) = handles.qFb; 
        handles.downFb(end+1) = handles.rFb; 
    elseif (handles.taskMng == 4)
        
            %% %%
    if mSerial.BytesAvailable()>= 42 ...
            && strcmp(handles.fbStatus, 'ready')
        
        receivedBuffer = fscanf(mSerial, '%s', 42);
        handles.fbStatus = 'none';
        if length(receivedBuffer) >= 42
            i = 1;
            while i<2
                flag = receivedBuffer(i);
                if strcmp(flag, 'a')
                    rollFb = str2double(receivedBuffer(i+1:i+6))/100.0;
                    if strcmp(receivedBuffer(i+7), 'b')...
                            && strcmp(receivedBuffer(i+14), 'c')...
                            && strcmp(receivedBuffer(i+21), 'p')...
                            && strcmp(receivedBuffer(i+28), 'q')...
                            && strcmp(receivedBuffer(i+35), 'r')

                        pitchFb = str2double(receivedBuffer(i+8:i+13))/100.0;
                        yawFb = str2double(receivedBuffer(i+15:i+20))/100.0;
                        pFb = str2double(receivedBuffer(i+22:i+27))/100.0;
                        qFb = str2double(receivedBuffer(i+29:i+34))/100.0;
                        rFb = str2double(receivedBuffer(i+36:i+41))/100.0;

                        if ~isnan(rollFb)
                            handles.rollFb = rollFb;
                        end
                        if ~isnan(pitchFb)
                            handles.pitchFb = pitchFb;
                        end
                        if ~isnan(yawFb)
                            handles.yawFb = yawFb;
                        end
                        if ~isnan(pFb)
                            handles.pFb = pFb;
                        end
                        if ~isnan(qFb)
                            handles.qFb = qFb;
                        end
                        if ~isnan(rFb)
                            handles.rFb = rFb;
                        end
                    end
                    i = i + 42;
                elseif  strcmp(flag, 'b')
                        if strcmp(receivedBuffer(i+7), 'c')...
                                && strcmp(receivedBuffer(i+14), 'p')...
                                && strcmp(receivedBuffer(i+21), 'q')...
                                && strcmp(receivedBuffer(i+28), 'r')...
                                && strcmp(receivedBuffer(i+35), 'a')
                            pitchFb = str2double(receivedBuffer(i+1:i+6))/100.0;
                            yawFb = str2double(receivedBuffer(i+8:i+13))/100.0;
                            pFb = str2double(receivedBuffer(i+15:i+20))/100.0;
                            qFb = str2double(receivedBuffer(i+22:i+27))/100.0;
                            rFb = str2double(receivedBuffer(i+29:i+34))/100.0;
                            rollFb = str2double(receivedBuffer(i+36:i+41))/100.0;
                            if ~isnan(rollFb)
                            handles.rollFb = rollFb;
                            end
                            if ~isnan(pitchFb)
                                handles.pitchFb = pitchFb;
                            end
                            if ~isnan(yawFb)
                                handles.yawFb = yawFb;
                            end
                            if ~isnan(pFb)
                                handles.pFb = pFb;
                            end
                            if ~isnan(qFb)
                                handles.qFb = qFb;
                            end
                            if ~isnan(rFb)
                                handles.rFb = rFb;
                            end
                        end
                        i = i + 42;
                    elseif strcmp(flag, 'c')
                            if strcmp(receivedBuffer(i+7), 'p')...
                                && strcmp(receivedBuffer(i+14), 'q')...
                                && strcmp(receivedBuffer(i+21), 'r')...
                                && strcmp(receivedBuffer(i+28), 'a')...
                                && strcmp(receivedBuffer(i+35), 'b')
                                yawFb = str2double(receivedBuffer(i+1:i+6))/100.0;
                                pFb = str2double(receivedBuffer(i+8:i+13))/100.0;
                                qFb = str2double(receivedBuffer(i+15:i+20))/100.0;
                                rFb = str2double(receivedBuffer(i+22:i+27))/100.0;
                                rollFb = str2double(receivedBuffer(i+29:i+34))/100.0;
                                pitchFb = str2double(receivedBuffer(i+36:i+41))/100.0;
                                if ~isnan(rollFb)
                                    handles.rollFb = rollFb;
                                end
                                if ~isnan(pitchFb)
                                    handles.pitchFb = pitchFb;
                                end
                                if ~isnan(yawFb)
                                    handles.yawFb = yawFb;
                                end
                                if ~isnan(pFb)
                                    handles.pFb = pFb;
                                end
                                if ~isnan(qFb)
                                    handles.qFb = qFb;
                                end
                                if ~isnan(rFb)
                                    handles.rFb = rFb;
                                end
                            end
                            i = i + 42;
                        elseif strcmp(flag, 'p')
                                if strcmp(receivedBuffer(i+7), 'q')...
                                    && strcmp(receivedBuffer(i+14), 'r')...
                                    && strcmp(receivedBuffer(i+21), 'a')...
                                    && strcmp(receivedBuffer(i+28), 'b')...
                                    && strcmp(receivedBuffer(i+35), 'c')
                                    pFb = str2double(receivedBuffer(i+1:i+6))/100.0;
                                    qFb = str2double(receivedBuffer(i+8:i+13))/100.0;
                                    rFb = str2double(receivedBuffer(i+15:i+20))/100.0;
                                    rollFb = str2double(receivedBuffer(i+22:i+27))/100.0;
                                    pitchFb = str2double(receivedBuffer(i+29:i+34))/100.0;
                                    yawFb = str2double(receivedBuffer(i+36:i+41))/100.0;  
                                    if ~isnan(rollFb)
                                        handles.rollFb = rollFb;
                                    end
                                    if ~isnan(pitchFb)
                                        handles.pitchFb = pitchFb;
                                    end
                                    if ~isnan(yawFb)
                                        handles.yawFb = yawFb;
                                    end
                                    if ~isnan(pFb)
                                        handles.pFb = pFb;
                                    end
                                    if ~isnan(qFb)
                                        handles.qFb = qFb;
                                    end
                                    if ~isnan(rFb)
                                        handles.rFb = rFb;
                                    end
                                end
                                i = i + 42;
                            elseif strcmp(flag, 'q')
                                    if strcmp(receivedBuffer(i+7), 'r')...
                                        && strcmp(receivedBuffer(i+14), 'a')...
                                        && strcmp(receivedBuffer(i+21), 'b')...
                                        && strcmp(receivedBuffer(i+28), 'c')...
                                        && strcmp(receivedBuffer(i+35), 'p')
                                        qFb = str2double(receivedBuffer(i+1:i+6))/100.0;
                                        rFb = str2double(receivedBuffer(i+8:i+13))/100.0;
                                        rollFb = str2double(receivedBuffer(i+15:i+20))/100.0;
                                        pitchFb = str2double(receivedBuffer(i+22:i+27))/100.0;
                                        yawFb = str2double(receivedBuffer(i+29:i+34))/100.0;
                                        pFb = str2double(receivedBuffer(i+36:i+41))/100.0;
                                        if ~isnan(rollFb)
                                            handles.rollFb = rollFb;
                                        end
                                        if ~isnan(pitchFb)
                                            handles.pitchFb = pitchFb;
                                        end
                                        if ~isnan(yawFb)
                                            handles.yawFb = yawFb;
                                        end
                                        if ~isnan(pFb)
                                            handles.pFb = pFb;
                                        end
                                        if ~isnan(qFb)
                                            handles.qFb = qFb;
                                        end
                                        if ~isnan(rFb)
                                            handles.rFb = rFb;
                                        end
                                    end
                                i = i + 42;
                                elseif strcmp(flag, 'r')
                                        if strcmp(receivedBuffer(i+7), 'a')...
                                            && strcmp(receivedBuffer(i+14), 'b')...
                                            && strcmp(receivedBuffer(i+21), 'c')...
                                            && strcmp(receivedBuffer(i+28), 'p')...
                                            && strcmp(receivedBuffer(i+35), 'q')
                                            rFb = str2double(receivedBuffer(i+1:i+6))/100.0;
                                            rollFb = str2double(receivedBuffer(i+8:i+13))/100.0;
                                            pitchFb = str2double(receivedBuffer(i+15:i+20))/100.0;
                                            yawFb = str2double(receivedBuffer(i+22:i+27))/100.0;
                                            pFb = str2double(receivedBuffer(i+29:i+34))/100.0;
                                            qFb = str2double(receivedBuffer(i+36:i+41))/100.0;
                                            if ~isnan(rollFb)
                                                handles.rollFb = rollFb;
                                            end
                                            if ~isnan(pitchFb)
                                                handles.pitchFb = pitchFb;
                                            end
                                            if ~isnan(yawFb)
                                                handles.yawFb = yawFb;
                                            end
                                            if ~isnan(pFb)
                                                handles.pFb = pFb;
                                            end
                                            if ~isnan(qFb)
                                                handles.qFb = qFb;
                                            end
                                            if ~isnan(rFb)
                                                handles.rFb = rFb;
                                            end
                                        end
                                    i = i + 42;
                else
                    i = i+1;
                end
            end
        end
    end
    try
        set(handles.tbxRollFb, 'string', handles.rollFb);
        set(handles.tbxPitchFb, 'string', handles.pitchFb);
        set(handles.tbxYawFb, 'string', handles.yawFb);
    catch
    end
    %% %%
        handles.taskMng = 0;
        handles.plotMng = handles.plotMng + 1;
        
    end
    
    if (handles.plotMng == 2)
        try
        set(handles.tbxNorthFb, 'String', num2str(handles.northFb(end)));
        set(handles.tbxEastFb, 'String', num2str(handles.eastFb(end)));
        set(handles.tbxDownFb, 'String', num2str(handles.downFb(end)));
        plot(handles.grphPos, handles.eastFb,...
            handles.northFb,'-r','LineWidth',1.2) ;
        
        grid(handles.grphPos, 'on');
            set(handles.grphPos, 'XLim',[-30, 30]);
            set(handles.grphPos, 'YLim',[-30, 30]);
        fontSize = 9;
        
        ylabel(handles.grphPos, 'North [m]', 'FontSize', fontSize);
        catch
        end
        handles.plotMng = 0;
    end
    %% 
    guidata(mhObject, handles);
        

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
global mSerial;
if ~strcmp(handles.serialStatus,'open')
    warndlg('No Connection to vehicle');
else
    
    handles.btnDownSendAtt = 1;
end
guidata(hObject, handles);

% --- Executes on button press in btnSetHover.
function btnSetHover_Callback(hObject, eventdata, handles)
% hObject    handle to btnSetHover (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.tbxRoll, 'string', 0); 
set(handles.tbxPitch, 'string', 0);
% set(handles.tbxYaw, 'string', 0);
handles.rollSetpoint = 0;
handles.pitchSetpoint = 0;
% handles.yawSetpoint = 0;
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

set(handles.btnSendAttitude,'enable','off');

global mSerial;
if ~strcmp(handles.serialStatus,'open')
    warndlg('No Connection to vehicle');
else
    handles.btnDownSendPos = 1;
%     handles.northSetpoint = str2double(get(hObject, 'String'));
%     handles.eastSetpoint = str2double(get(hObject, 'String'));

%     buffer = strcat('n', num2str_norm(handles.northSetpoint, 5), ...
%         'i', num2str_norm(handles.eastSetpoint, 5), ...
%         'd', num2str_norm(0.9, 5));
%     fprintf(mSerial,'%s', buffer);
end
guidata(hObject, handles);

% --- Executes on button press in btnSetHome.
function btnSetHome_Callback(hObject, eventdata, handles)
% hObject    handle to btnSetHome (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.tbxNorthSet, 'string', 0);
set(handles.tbxEastSet, 'string', 0);
handles.btnDownResetPosControl = 1;
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
        cla(handles.grphPos);
        handles.northFb = 0;
        handles.eastFb = 0;
        handles.downFb = 0;
        global mSerial; 
        mSerial = serial('COM8');
        set(mSerial, 'BaudRate', 57600);
%         mSerial.BytesAvailableFcn = @(handles, eventdata)update_feedback(hObject, ...
%             eventdata);
%         mSerial.BytesAvailableFcnCount = 49;
%         mSerial.BytesAvailableFcnMode = 'byte';
        %guidata(hObject, handles);
       
    try
        fopen(mSerial);
        handles.serialStatus = 'open';
        guidata(hObject, handles);
        msgbox('Vehicle connected');
        set(handles.btnSync,'visible','on');
    catch
        %delete(handles.mSerial);
        warndlg('Connection failed');
        handles.serialStatus = 'closed';
        set(handles.btnConnect,'visible','on');
        set(handles.btnDisconnect,'visible','off');
    end
    
    % FT controller
    global ftSerial;
    ftSerial = serial('COM9');
    set(ftSerial, 'BaudRate', 9600);
    set(ftSerial, 'FlowControl', 'software');
    try 
        if ~strcmp(handles.ftSerialStatus, 'open')
            fopen(ftSerial);
        end
        handles.ftSerialStatus = 'open';
        guidata(hObject, handles);
        msgbox('FT connected');
    catch
        warndlg('Connection to FT sensor system failed');
        handles.ftSerialStatus = 'closed';
        guidata(hObject, handles);
    end

    guidata(hObject, handles);
    
    
function update_feedback(hObject,eventdata)
    handles = guidata(hObject);
%% %% Get DOWN
    handles.serialMng = handles.serialMng + 1;
    global mSerial;
    receivedBuffer = fscanf(mSerial, '%s', 49);
    if (handles.serialMng == 3)
        i = 1;
        while i<8
            flag = receivedBuffer(i);
            if strcmp(flag, 'a')
                rollFb = str2double(receivedBuffer(i+1:i+6))/100.0;
                if strcmp(receivedBuffer(i+7), 'b')...
                        && strcmp(receivedBuffer(i+14), 'c')...
                        && strcmp(receivedBuffer(i+21), 'p')...
                        && strcmp(receivedBuffer(i+28), 'q')...
                        && strcmp(receivedBuffer(i+35), 'r')

                    pitchFb = str2double(receivedBuffer(i+8:i+13))/100.0;
                    yawFb = str2double(receivedBuffer(i+15:i+20))/100.0;
                    pFb = str2double(receivedBuffer(i+22:i+27))/100.0;
                    qFb = str2double(receivedBuffer(i+29:i+34))/100.0;
                    rFb = str2double(receivedBuffer(i+36:i+41))/100.0;

                    if ~isnan(rollFb)
                        handles.rollFb = rollFb;
                    end
                    if ~isnan(pitchFb)
                        handles.pitchFb = pitchFb;
                    end
                    if ~isnan(yawFb)
                        handles.yawFb = yawFb;
                    end
                    if ~isnan(pFb)
                        handles.pFb = pFb;
                    end
                    if ~isnan(qFb)
                        handles.qFb = qFb;
                    end
                    if ~isnan(rFb)
                        handles.rFb = rFb;
                    end
                end
                i = i + 42;
            elseif  strcmp(flag, 'b')
                    if strcmp(receivedBuffer(i+7), 'c')...
                            && strcmp(receivedBuffer(i+14), 'p')...
                            && strcmp(receivedBuffer(i+21), 'q')...
                            && strcmp(receivedBuffer(i+28), 'r')...
                            && strcmp(receivedBuffer(i+35), 'a')
                        pitchFb = str2double(receivedBuffer(i+1:i+6))/100.0;
                        yawFb = str2double(receivedBuffer(i+8:i+13))/100.0;
                        pFb = str2double(receivedBuffer(i+15:i+20))/100.0;
                        qFb = str2double(receivedBuffer(i+22:i+27))/100.0;
                        rFb = str2double(receivedBuffer(i+29:i+34))/100.0;
                        rollFb = str2double(receivedBuffer(i+36:i+41))/100.0;
                        if ~isnan(rollFb)
                        handles.rollFb = rollFb;
                        end
                        if ~isnan(pitchFb)
                            handles.pitchFb = pitchFb;
                        end
                        if ~isnan(yawFb)
                            handles.yawFb = yawFb;
                        end
                        if ~isnan(pFb)
                            handles.pFb = pFb;
                        end
                        if ~isnan(qFb)
                            handles.qFb = qFb;
                        end
                        if ~isnan(rFb)
                            handles.rFb = rFb;
                        end
                    end
                    i = i + 42;
                elseif strcmp(flag, 'c')
                        if strcmp(receivedBuffer(i+7), 'p')...
                            && strcmp(receivedBuffer(i+14), 'q')...
                            && strcmp(receivedBuffer(i+21), 'r')...
                            && strcmp(receivedBuffer(i+28), 'a')...
                            && strcmp(receivedBuffer(i+35), 'b')
                            yawFb = str2double(receivedBuffer(i+1:i+6))/100.0;
                            pFb = str2double(receivedBuffer(i+8:i+13))/100.0;
                            qFb = str2double(receivedBuffer(i+15:i+20))/100.0;
                            rFb = str2double(receivedBuffer(i+22:i+27))/100.0;
                            rollFb = str2double(receivedBuffer(i+29:i+34))/100.0;
                            pitchFb = str2double(receivedBuffer(i+36:i+41))/100.0;
                            if ~isnan(rollFb)
                                handles.rollFb = rollFb;
                            end
                            if ~isnan(pitchFb)
                                handles.pitchFb = pitchFb;
                            end
                            if ~isnan(yawFb)
                                handles.yawFb = yawFb;
                            end
                            if ~isnan(pFb)
                                handles.pFb = pFb;
                            end
                            if ~isnan(qFb)
                                handles.qFb = qFb;
                            end
                            if ~isnan(rFb)
                                handles.rFb = rFb;
                            end
                        end
                        i = i + 42;
                    elseif strcmp(flag, 'p')
                            if strcmp(receivedBuffer(i+7), 'q')...
                                && strcmp(receivedBuffer(i+14), 'r')...
                                && strcmp(receivedBuffer(i+21), 'a')...
                                && strcmp(receivedBuffer(i+28), 'b')...
                                && strcmp(receivedBuffer(i+35), 'c')
                                pFb = str2double(receivedBuffer(i+1:i+6))/100.0;
                                qFb = str2double(receivedBuffer(i+8:i+13))/100.0;
                                rFb = str2double(receivedBuffer(i+15:i+20))/100.0;
                                rollFb = str2double(receivedBuffer(i+22:i+27))/100.0;
                                pitchFb = str2double(receivedBuffer(i+29:i+34))/100.0;
                                yawFb = str2double(receivedBuffer(i+36:i+41))/100.0;  
                                if ~isnan(rollFb)
                                    handles.rollFb = rollFb;
                                end
                                if ~isnan(pitchFb)
                                    handles.pitchFb = pitchFb;
                                end
                                if ~isnan(yawFb)
                                    handles.yawFb = yawFb;
                                end
                                if ~isnan(pFb)
                                    handles.pFb = pFb;
                                end
                                if ~isnan(qFb)
                                    handles.qFb = qFb;
                                end
                                if ~isnan(rFb)
                                    handles.rFb = rFb;
                                end
                            end
                            i = i + 42;
                        elseif strcmp(flag, 'q')
                                if strcmp(receivedBuffer(i+7), 'r')...
                                    && strcmp(receivedBuffer(i+14), 'a')...
                                    && strcmp(receivedBuffer(i+21), 'b')...
                                    && strcmp(receivedBuffer(i+28), 'c')...
                                    && strcmp(receivedBuffer(i+35), 'p')
                                    qFb = str2double(receivedBuffer(i+1:i+6))/100.0;
                                    rFb = str2double(receivedBuffer(i+8:i+13))/100.0;
                                    rollFb = str2double(receivedBuffer(i+15:i+20))/100.0;
                                    pitchFb = str2double(receivedBuffer(i+22:i+27))/100.0;
                                    yawFb = str2double(receivedBuffer(i+29:i+34))/100.0;
                                    pFb = str2double(receivedBuffer(i+36:i+41))/100.0;
                                    if ~isnan(rollFb)
                                        handles.rollFb = rollFb;
                                    end
                                    if ~isnan(pitchFb)
                                        handles.pitchFb = pitchFb;
                                    end
                                    if ~isnan(yawFb)
                                        handles.yawFb = yawFb;
                                    end
                                    if ~isnan(pFb)
                                        handles.pFb = pFb;
                                    end
                                    if ~isnan(qFb)
                                        handles.qFb = qFb;
                                    end
                                    if ~isnan(rFb)
                                        handles.rFb = rFb;
                                    end
                                end
                            i = i + 42;
                            elseif strcmp(flag, 'r')
                                    if strcmp(receivedBuffer(i+7), 'a')...
                                        && strcmp(receivedBuffer(i+14), 'b')...
                                        && strcmp(receivedBuffer(i+21), 'c')...
                                        && strcmp(receivedBuffer(i+28), 'p')...
                                        && strcmp(receivedBuffer(i+35), 'q')
                                        rFb = str2double(receivedBuffer(i+1:i+6))/100.0;
                                        rollFb = str2double(receivedBuffer(i+8:i+13))/100.0;
                                        pitchFb = str2double(receivedBuffer(i+15:i+20))/100.0;
                                        yawFb = str2double(receivedBuffer(i+22:i+27))/100.0;
                                        pFb = str2double(receivedBuffer(i+29:i+34))/100.0;
                                        qFb = str2double(receivedBuffer(i+36:i+41))/100.0;
                                        if ~isnan(rollFb)
                                            handles.rollFb = rollFb;
                                        end
                                        if ~isnan(pitchFb)
                                            handles.pitchFb = pitchFb;
                                        end
                                        if ~isnan(yawFb)
                                            handles.yawFb = yawFb;
                                        end
                                        if ~isnan(pFb)
                                            handles.pFb = pFb;
                                        end
                                        if ~isnan(qFb)
                                            handles.qFb = qFb;
                                        end
                                        if ~isnan(rFb)
                                            handles.rFb = rFb;
                                        end
                                    end
                                i = i + 42;
            else
                i = i+1;
            end
        end
        handles.serialMng = 0;
    end
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

    global mTimer;
    if strcmp(handles.timerStatus, 'running')
            stop(mTimer);
            handles.timerStatus = 'stopped';
    end
        % FT controller
    global ftTimer;
    if strcmp(handles.ftTimerStatus, 'running')
        %try
            stop(ftTimer);
            handles.ftTimerStatus = 'stopped';
        %catch
        %end
    end
    
    global mSerial;
    if strcmp(handles.serialStatus, 'open')...
        && ~strcmp(handles.ftTimerStatus, 'running')
        %&& ~strcmp(handles.ftTimerStatus, 'running')
        fclose(mSerial); 
    end
    handles.serialStatus = 'closed';
%     end
    if ~isempty(mSerial)
        delete(mSerial); 
    end
    handles.serialStatus = 'none';

    global ftSerial;
    if strcmp(handles.ftSerialStatus, 'open')...
            && ~strcmp(handles.ftTimerStatus, 'running')
        fprintf(ftSerial, '%s', char(13));
        fclose(ftSerial); 
        handles.ftSerialStatus = 'closed';
    end
    
    if ~isempty(ftSerial)
        delete(ftSerial); 
    end
    handles.ftSerialStatus = 'none';
    handles.ftStatus = 'none';    
    %
    
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
global mTimer;

stop(mTimer);
delete(mTimer);
clear mTimer;
handles.timerStatus = 'none';

if strcmp(handles.serialStatus, 'open')
    warndlg('The serial port will be closed automatically.');
    fclose(mSerial); 
end
delete(mSerial); 
clear mSerial;


% FT controller
global ftTimer;
    if strcmp(handles.ftTimerStatus, 'running')
        stop(ftTimer);
    end
    delete(ftTimer);
    handles.ftTimerStatus = 'stopped';
    
    global ftSerial;
    if strcmp(handles.ftSerialStatus, 'open')
        fclose(ftSerial); 
    end
    handles.ftSerialStatus = 'closed';
    %     end
    delete(ftSerial); 
    handles.ftSerialStatus = 'none';

    guidata(hObject, handles);

    
% --- Executes on button press in btnSync.
function btnSync_Callback(hObject, eventdata, handles)
% hObject    handle to btnSync (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% FT controller
global ftTimer;
if ~strcmp(handles.ftTimerStatus, 'running')
    start(ftTimer);
end
handles.ftTimerStatus = 'running';
guidata(hObject, handles);




function tbxHeightSet_Callback(hObject, eventdata, handles)
% hObject    handle to tbxHeightSet (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tbxHeightSet as text
%        str2double(get(hObject,'String')) returns contents of tbxHeightSet as a double
handles.heightSetpoint = str2double(get(hObject, 'String'));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function tbxHeightSet_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbxHeightSet (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tbxK2Pos_Callback(hObject, eventdata, handles)
% hObject    handle to tbxK2Pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tbxK2Pos as text
handles.k2Pos = str2double(get(hObject,'String')); %returns contents of tbxK2Pos as a double
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function tbxK2Pos_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbxK2Pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tbxK1Pos_Callback(hObject, eventdata, handles)
% hObject    handle to tbxK1Pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tbxK1Pos as text
handles.k1Pos = str2double(get(hObject,'String')); % returns contents of tbxK1Pos as a double
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function tbxK1Pos_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbxK1Pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tbxDownFb_Callback(hObject, eventdata, handles)
% hObject    handle to tbxDownFb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tbxDownFb as text
%        str2double(get(hObject,'String')) returns contents of tbxDownFb as a double


% --- Executes during object creation, after setting all properties.
function tbxDownFb_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbxDownFb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

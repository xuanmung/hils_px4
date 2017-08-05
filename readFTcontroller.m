%% 
global mSerial1;
mSerial1 = serial('COM4');
set(mSerial1, 'BaudRate', 9600);
mSerial1.BytesAvailableFcn = @(handles, eventdata)update_ft(hObject, ...
    eventdata);
mSerial1.BytesAvailableFcnCount = 57;
mSerial1.BytesAvailableFcnMode = 'byte';
%% 

fprintf(mSerial1, '%s', 'QS\r');

%% 
function update_ft(hObject, eventdata)
flag = fscanf(mSerial1, '%s', 1);
while ~strcmp(flag, '\n')
    flag = fscanf(mSerial1, '%s', 1);
end
ftBuffer = fscanf(mSerial1, '%s', 56);
fx = str2double(ftBuffer(3:10))/20.0;
fy = str2double(ftBuffer(12:19))/20.0;
fz = str2double(ftBuffer(21:28))/10.0;
% tx = str2double(ftBuffer(30:37))/400.0;
% ty = str2double(ftBuffer(39:46))/400.0;
% tz = str2double(ftBuffer(48:55))/400.0;
end



% parameters
clc; clear all; close all;

%% Determine the moment of inertia X, Y, Z
Lw=0.56;        % length of wire suspending disc from ceiling in m.
R=0.128;       % radius of disc in m
W=1.198*9.81;   % weight of the disc and quadrotor in kg
Tx=1.19;       % 1 ocsolation X-axis
Ty=1.15;     % 1 ocsolation Y-axis
Tz=1.60;     % 1 ocsolation Z-axis
% Inertial moment 
Ix=(W*R^2*Tx^2)/(4*pi^2*Lw) + 0.0008   %Kg.m2
Iy=(W*R^2*Ty^2)/(4*pi^2*Lw) + 0.0008   %Kg.m2
Iz=(W*R^2*Tz^2)/(4*pi^2*Lw)    %Kg.m2

%% parameters of Lego
L=0.23;                             % Arm length (m)
I = diag([Ix,Iy,Iz]);   % kg.m^2
% m = 1.122;              % Mass [kg] : Bat: Blue Core 14.8V-3300mAh; DJI Gear;
m = 1.780;              % Mass [kg] : Bat: Polytronics 14.8V-5200mAh; Xcopter landing skid;
g = 9.8;                            % Gravity constant [m/s^2]
%% Hover position (propeller) (PHAN NAY CHUA MEASURE)
%w0=4818;
%u0=(w0-1075)/37.625;    % RPM = 1075 + 37.625*u;
%w0r=w0*2*pi/60;         % convert RPM to Rad/s
%CThrust=m*g/(4*w0r^2)  % N/(rad/s)^2, Thrust coeficients: mg=4b*w0r^2

CThrust = 7.58721e-6;       % N/(rad/s)^2, at hover position u0=100
CDrag   = 1.35765e-7;       % N/(rad/s)^2
w0r=sqrt(m*g/(4*CThrust));
w0 = w0r*30/pi;
u0=(w0-1075)/37.625;

lamda=w0r^2/u0;         % w^2=lamda*u
%CDrag = 1.2181e-07;     % N/(rad/s)^2
Kth=CThrust*lamda;      % Kth=b*lamda
Kd=CDrag*lamda;         % Kd=d*lamda
%%
% display
disp('------Hover parameters-------')
formatSpec = 'Hover Vel. w0 = %d(RPM), %f(Rad/s)\n';
fprintf(formatSpec,w0,w0r);
formatSpec = 'Hover CMD  u0 = %4.2f(CMD)\n';
fprintf(formatSpec,u0);
formatSpec = 'Hover CThrust = %e (N/(rad/s)^2)\n';
fprintf(formatSpec,CThrust);
formatSpec = 'Hover Lamda   = %e ((rad/s)/scale)\n';
fprintf(formatSpec,lamda);
formatSpec = 'Hover Kth     = %f (N.s/(rad.scale))\n';
fprintf(formatSpec,Kth);
formatSpec = 'Hover CDrag   = %e (N/(rad/s)^2)\n';
fprintf(formatSpec,CDrag);
formatSpec = 'Hover Kd      = %e (N.s/(rad.scale))\n';
fprintf(formatSpec,Kd);

%% Controller design====================================================================================
Jx=(Ix/(2*Kth*L));      % Jx 
Jy=(Iy/(2*Kth*L));      % Jy 
Jz=(Iz/(4*Kd));         % Jz 
Mz=m/(4*Kth);          % Mz
%% Attitude control
% for roll 
omega_n=8.0;  %10 rad/s (rise time = 1.8/9.3=0.1935 sec)
zeta=0.7;
k1_roll=2*zeta*omega_n*Jx;
k2_roll=omega_n^2*Jx/k1_roll;

% for pitch
omega_p=8.0;  % 10.2 rad/s (rise time = 1.8/9.5=0.1895 sec)
zeta=0.7;
k1_pitch=2*zeta*omega_p*Jy;
k2_pitch=omega_p^2*Jy/k1_pitch;

% for yaw
omega_y=1.4; % 2 (rise time = 1.8/3 = 0.6sec)
zeta_y=0.7;
k1_yaw=2*zeta_y*omega_y*Jz;
k2_yaw=omega_y^2*Jz/k1_yaw;

% Display
disp('------Controller-------')

formatSpec = 'k1_roll  = %f \n';
fprintf(formatSpec,k1_roll);
formatSpec = 'k2_roll  = %f \n';
fprintf(formatSpec,k2_roll);

formatSpec = 'k1_pitch = %f \n';
fprintf(formatSpec,k1_pitch);
formatSpec = 'k2_pitch = %f \n';
fprintf(formatSpec,k2_pitch);

formatSpec = 'k1_yaw   = %f \n';
fprintf(formatSpec,k1_yaw);
formatSpec = 'k2_yaw   = %f \n';
fprintf(formatSpec,k2_yaw);

%% Control altitude----------------
omega_z = 1.2;  %rad/s
zetaz = 0.7;
k1_al = 2*zetaz*omega_z*Mz;
k2_al = omega_z^2*Mz/k1_al;

% omega_z = 0.45;  %rad/s
% zetaz = 0.7;
% k1_al = 2*zetaz*omega_z*m;
% k2_al = omega_z^2*m/k1_al;

% Display
formatSpec = 'k1_al    = %f \n';
fprintf(formatSpec,k1_al);
formatSpec = 'k2_al    = %f \n';
fprintf(formatSpec,k2_al);

%% position control 
Mx = m; 
omega_p = 0.6;  %rad/s
zetap = 0.65;  
k1_pos=2*zetap*omega_p*Mx; 
k2_pos=omega_p^2*Mx/k1_pos; 

% Display
formatSpec = 'k1_pos   = %f \n';
fprintf(formatSpec,k1_pos);
formatSpec = 'k2_pos   = %f \n';
fprintf(formatSpec,k2_pos);

%% Obstacle Avoidance control 
Mx = m; 
omega_obs = 1.1;  %rad/s 
zeta_obs = 0.65;  
k1_obs=2*zeta_obs*omega_obs*Mx; 
k2_obs=omega_obs^2*Mx/k1_obs; 

% Display
formatSpec = 'k1_obs   = %f \n';
fprintf(formatSpec,k1_obs);
formatSpec = 'k2_obs   = %f \n';
fprintf(formatSpec,k2_obs);

%% (Motor) (Not Important)
Iprop = 12*10^-6;                   % Motor/Prop inertia [kg*m^2], was 3.7404e-5
Rmot = 0.128;                       % Motor Resistance [Ohm]
Kmot = 1/(1900/60*2*pi);            % Motor Constant [V/(rad/s)], usally given by KV=1900 (RPM/V)
% Initial Condition
omegaInit = [0;0;0];
attitudeInit = [0;0;0];
velInit = [0;0;0];
posInit = [0;0;0];
posInit_2 = [5;5;0];
MotorSpeedInit = [300;300;300;300];
% %% -------------Hovering-----------------------
% w=sqrt(m*g/(4*CThrust))*60/2/pi;

%% ----LPF--
num=1;
den=[0.1 1];
[numd,dend]=c2dm(num,den,0.001,'zoh');
LPF=tf(numd,dend);

%% -------Velocity Control----------

Au=[-0.32 -9.8; 0 -8.74];
Cu=[-0.32 0];
[vu,su] = eig(Au);
Lu = (Au+5*0.32*eye(2))*(Au+5*8.74*eye(2))*inv([Cu;Cu*Au])*[0;1];

Av=[-0.32 9.8; 0 -9.07];
Cv=[-0.32 0];
[vv,sv] = eig(Av);
Lv = (Av+5*0.32*eye(2))*(Av+5*9.07*eye(2))*inv([Cv;Cv*Av])*[0;1];

%% ----- Base square and Lateral square

Sb = pi*0.3^2;
Sl = 2*pi*0.3*0.11;



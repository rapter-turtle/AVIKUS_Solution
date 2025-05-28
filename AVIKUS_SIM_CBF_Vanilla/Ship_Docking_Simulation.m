clc; clear; close all;

%% Load Path
addpath(genpath('dynamics'))
addpath(genpath('plot'))
addpath(genpath('utils'))
addpath(genpath('CBF'))
addpath(genpath('C:\Users\user\Desktop\qpSWIFT-main'))
addpath(genpath('C:\Users\user\Desktop\casadi-3.6.7-windows64-matlab2018b'))
P = load_parm();
%% Simulation parameters
dt = 0.01; % Simulation step (fine-grained)
control_update_dt = 0.5; % Control update interval
control_update_steps = control_update_dt / dt; % Control update every 10 steps
T = 200; % Simulation duration
t = 0:dt:T;
N = length(t);

%% Initialize states
x_state = zeros(1, N); 
y_state = zeros(1, N);
psi_state = zeros(1, N);
u_state = zeros(1, N);
v_state = zeros(1, N);
r_state = zeros(1, N);
Tau_TP = zeros(1, N);
Tau_TS = zeros(1, N);
Tau_delPR = zeros(1, N);
Tau_delSR = zeros(1, N);
Tau_TP_real = zeros(1, N);
Tau_TS_real = zeros(1, N);
Tau_delPR_real = zeros(1, N);
Tau_delSR_real = zeros(1, N);
XN_history = zeros(1, N);
YN_history = zeros(1, N);
NN_history = zeros(1, N);

x_state(1) = -4;
y_state(1) = -15;
psi_state(1) = -0.8*pi;

TP_cmd = 0;
TS_cmd = 0;
alloc_TP_cmd = 0;
alloc_TS_cmd = 0;
delPR_cmd = 0;
delSR_cmd = 0;
delPR = 0;
delSR = 0;
thrP = 0;
thrS = 0;
WX = 0;
WY = 0;
WN = 0;

Xn = 0;
Yn = 0;
Nn = 0;


mode = 0;
x1 = 10;
y1 = 1;
dock_count = 0;
dock_count2 = 0;
start_time = 0;
Num = 0;

alpha1 = 5;
alpha2 = 5;
alpha_l = 0.1;
alpha_r = 0.1;
a = 2;



%% Animation initialization
[ship_patch, path_line, h_thruster_L, h_thruster_R, q_thruster_L, q_thruster_R, subplot_axes] = plot_ship_animation_init(x_state(1), y_state(1), psi_state(1));


%% Animation MP4
video_filename = 'Docking_simulation.mp4'; % 저장할 파일명
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = 10; % 초당 프레임 수, 적절히 조정 가능
open(v);


%% Simulation loop 
for i = 2:N

    %% Control policy (MPC)
    if mod(i, control_update_steps) == 0
        
        eps = 0.00001;
        XH = P.Xu*u_state(i-1) + P.Xvr*v_state(i-1)*r_state(i-1);
        YH = P.Yv*v_state(i-1) + P.Yuv*u_state(i-1)*v_state(i-1) + P.Yvv*v_state(i-1)*sqrt(v_state(i-1)*v_state(i-1) + eps) + P.Yr*r_state(i-1);
        NH = P.Nv*v_state(i-1) + P.Nuv*u_state(i-1)*v_state(i-1) + P.Nvv*v_state(i-1)*sqrt(v_state(i-1)*v_state(i-1) + eps) + P.Nr*r_state(i-1) + P.Nrr*r_state(i-1)*sqrt(r_state(i-1)*r_state(i-1) + eps);
         
        x_dot = u_state(i-1)*cos(psi_state(i-1)) - v_state(i-1)*sin(psi_state(i-1)); 
        y_dot = u_state(i-1)*sin(psi_state(i-1)) + v_state(i-1)*cos(psi_state(i-1)); 

        Fx0 = 0.2*( - x_state(i-1)) - 5*x_dot;
        Fy0 = 0.2*(-1 - y_state(i-1)) - 5*y_dot;

        [Fx, Fy] = cbf(Fx0, Fy0, x_state(i-1), y_state(i-1), x_dot, y_dot, alpha1, alpha2, alpha_l, alpha_r, a, control_update_dt);

        Xn = -XH + (Fx*cos(psi_state(i-1)) + Fy*sin(psi_state(i-1)))*P.M;
        Yn = -YH + (- Fx*sin(psi_state(i-1)) + Fy*cos(psi_state(i-1)))*P.M;
        Nn = -NH + 100000*(-0.5*pi - psi_state(i-1)) - 100000*(r_state(i-1));

        [TP_cmd, TS_cmd, delPR_cmd, delSR_cmd]=allocation(Xn, Yn, Nn, TP_cmd, TS_cmd, delPR_cmd, delSR_cmd);
        

        [alloc_TP_cmd, alloc_TS_cmd] = thrust_allocation(TP_cmd, TS_cmd);
    end

    XN_history(i) = Xn;
    YN_history(i) = Yn;
    NN_history(i) = Nn;


    %% Dynamic updatae
    [u_state(i), v_state(i), r_state(i), x_state(i), y_state(i), psi_state(i), thrP, thrS, delPR, delSR] = update_ship_dynamics(u_state(i-1), v_state(i-1), r_state(i-1), x_state(i-1), y_state(i-1), psi_state(i-1), thrP, thrS, delPR, delSR, alloc_TP_cmd, alloc_TS_cmd, delPR_cmd, delSR_cmd, dt, WX, WY, WN);
    
    MPC_state = [x_state(i), y_state(i), psi_state(i), u_state(i), v_state(i), r_state(i), TP_cmd, TS_cmd, delPR_cmd, delSR_cmd]';
    
    
    %% Data log
    Tau_TP(i) = TP_cmd;
    Tau_TS(i) = TS_cmd;
    Tau_delPR(i) = delPR_cmd;
    Tau_delSR(i) = delSR_cmd;
    Tau_TP_real(i) = thrP;
    Tau_TS_real(i) = thrS;
    Tau_delPR_real(i) = delPR;
    Tau_delSR_real(i) = delSR;
    if mod(i,control_update_steps)==0  
        plot_ship_animation_update(i, ship_patch, path_line, h_thruster_L, h_thruster_R, q_thruster_L, q_thruster_R, x_state, y_state, psi_state, u_state, v_state, r_state, Tau_TP, Tau_TS, Tau_delPR, Tau_delSR, Tau_TP_real, Tau_TS_real, Tau_delPR_real, Tau_delSR_real, subplot_axes, t);
        
        %% Animation MP4
        frame = getframe(gcf); % 현재 figure 캡처
        writeVideo(v, frame);
    end  
end

%% Animation MP4
close(v);


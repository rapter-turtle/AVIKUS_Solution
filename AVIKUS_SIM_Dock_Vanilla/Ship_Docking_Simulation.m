clc; clear; close all;

%% Load Path
addpath(genpath('dynamics'))
addpath(genpath('plot'))
addpath(genpath('utils'))
addpath(genpath('NMPC'))
addpath(genpath('C:\Users\user\Desktop\qpSWIFT-main'))
addpath(genpath('C:\Users\user\Desktop\casadi-3.6.7-windows64-matlab2018b'))
addpath(genpath('C:\Users\leeck\Desktop\casadi-3.7.0-windows64-matlab2018b'))

%% Simulation parameters
dt = 0.1; % Simulation step (fine-grained)
control_update_dt = 0.5; % Control update interval
control_update_steps = control_update_dt / dt; % Control update every 10 steps
T = 150; % Simulation duration
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

MPC_state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]';
MPC_input = [0, 0, 0, 0]';
MPC_pred = [];
MPC_ref = [];

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

mode = 0;
x1 = 10;
y1 = 1;
dock_count = 0;
dock_count2 = 0;
start_time = 0;


%% MPC setting
% Control params
Num = 50; 

% MPC Weight Parameters
% Q = diag([0 0 0 100 100 10000 1e0 1e0 1e0 1e0]);
% R = diag([1e1 1e1 1e3 1e3]);
% Rd = diag([1e1 1e1 1e3 1e3]);
% QN = Num*diag([1000 1000 1000 10 10 100 1e0 1e0 1e0 1e0]);
Q = diag([0 0 0 1000 1000 100000 3e0 3e0 1e0 1e0]);
R = diag([1e1 1e1 1e4 1e4]);
Rd = diag([1e1 1e1 1e4 1e4]);
QN = Num*diag([1000 1000 10000 10 10 1000 1e0 1e0 1e0 1e0]);

% init casadi
c_sol = initialize_casadi(Num, control_update_dt, Q,R,Rd,QN);
c_sol.input.u0 = ones(Num,1).*MPC_input';
c_sol.input.X0 = repmat(MPC_state',1,Num+1)'; 


%% Animation initialization
[ship_patch, path_line, h_thruster_L, h_thruster_R, q_thruster_L, q_thruster_R, pred_path_plot, reference_path_plot, subplot_axes] = plot_ship_animation_init(x_state(1), y_state(1), psi_state(1), Num);


%% Animation MP4
save_video = 0;
if save_video
video_filename = 'Docking_simulation.mp4'; % 저장할 파일명
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = 10; % 초당 프레임 수, 적절히 조정 가능
open(v);
end


%% Simulation loop 
for i = 2:N

    %% Control policy (MPC)
    if mod(i, control_update_steps) == 0
        MPC_ref = [];

        % Dock mode planner
        fprintf('Mode : %d\n', mode);
        if mode == 0
            if (sqrt((x1 - x_state(i-1))^2 + (x1 - x_state(i-1))^2) < 1.0)
                dock_count = dock_count + 1;
                fprintf('Count: %d\n', dock_count);
    
                if (dock_count*control_update_dt >= 3) && (mode == 0)
                    mode = 1;
                    dock_count = 0;
                end
            else
                dock_count = 0;
            end
        end
        
        if mode == 1
            if (sqrt((0.5*pi + psi_state(i-1))^2 )< 10*pi/180)
                dock_count = dock_count + 1;
                fprintf('Count: %d\n', dock_count);
                if (dock_count*control_update_dt >= 3) && (mode == 1)
                    mode = 2;
                    start_time = i;
                end
            else
                dock_count = 0;
            end
        end

        % Dock mode MPC setting change
        if mode >= 1
            n_states = 10;
            c_sol.args.lbx(1:n_states:n_states*(Num+1),1) =  8.5;
            c_sol.args.ubx(1:n_states:n_states*(Num+1),1) =  12.5;
            c_sol.args.lbx(2:n_states:n_states*(Num+1),1) =  0;
            c_sol.args.ubx(2:n_states:n_states*(Num+1),1) =  15;
        end


        % MPC setting
        c_sol.args.p(1:c_sol.nx+c_sol.nu) = [MPC_state; MPC_input];
        
        for k = 1:Num+1
            [ref_x,ref_y,ref_psi,ref_u,ref_v,ref_r,ref_TP,ref_TS,ref_delPR,ref_delSR] = reference_trajectory(i, dt, control_update_dt, k, mode, start_time);
            ref_state = [ref_x,ref_y,ref_psi,ref_u,ref_v,ref_r,ref_TP,ref_TS,ref_delPR,ref_delSR];
            c_sol.args.p(c_sol.nu + c_sol.nx*k + 1:c_sol.nu + c_sol.nx*k + 10) = ref_state;       
            MPC_ref = [MPC_ref; ref_state];
        end

        c_sol.args.x0  = [reshape(c_sol.input.X0',c_sol.nx*(Num+1),1);
        reshape(c_sol.input.u0',c_sol.nu*Num,1)];
        
        % Solve the problem
        % tic
        sol = c_sol.solver('x0', c_sol.args.x0, 'lbx', c_sol.args.lbx, 'ubx', c_sol.args.ubx,...
            'lbg', c_sol.args.lbg, 'ubg', c_sol.args.ubg,'p',c_sol.args.p);
        usol = reshape(full(sol.x(c_sol.nx*(Num+1)+1:end))',c_sol.nu,Num)'; % get controls only from the solution
        xsol= reshape(full(sol.x(1:c_sol.nx*(Num+1)))',c_sol.nx,Num+1)'; % get solution TRAJECTORY
        % toc
        c_sol.input.X0 = [xsol(2:end,:);xsol(end,:)];
        c_sol.input.u0 = [usol(2:end,:);usol(end,:)];
        MPC_pred = xsol;

        % Get control input
        d_TP_cmd = usol(1,1);
        d_TS_cmd = usol(1,2);
        d_delPR_cmd = usol(1,3);
        d_delSR_cmd = usol(1,4);


        TP_cmd = xsol(1,7) + d_TP_cmd*control_update_dt;
        TS_cmd = xsol(1,8) + d_TS_cmd*control_update_dt;
        delPR_cmd = xsol(1,9) + d_delPR_cmd*control_update_dt;
        delSR_cmd = xsol(1,10) + d_delSR_cmd*control_update_dt;

        [alloc_TP_cmd, alloc_TS_cmd] = thrust_allocation(TP_cmd, TS_cmd);

        MPC_input = [d_TP_cmd, d_TS_cmd, d_delPR_cmd, d_delSR_cmd]';

    end

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
    % if mod(i,control_update_steps)==0  
    if mod(i,50)==0  
        plot_ship_animation_update(i, ship_patch, path_line, h_thruster_L, h_thruster_R, q_thruster_L, q_thruster_R, pred_path_plot, reference_path_plot, x_state, y_state, psi_state, u_state, v_state, r_state, Tau_TP, Tau_TS, Tau_delPR, Tau_delSR, Tau_TP_real, Tau_TS_real, Tau_delPR_real, Tau_delSR_real, subplot_axes, t, MPC_pred, MPC_ref);
        
        %% Animation MP4
        if save_video
        frame = getframe(gcf); % 현재 figure 캡처
        writeVideo(v, frame);
        end
    end  
end

if save_video
%% Animation MP4
close(v);
end


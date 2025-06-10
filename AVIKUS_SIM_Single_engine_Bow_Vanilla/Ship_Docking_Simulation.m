clc; clear; close all;

%% Load Path
addpath(genpath('dynamics'))
addpath(genpath('plot'))
addpath(genpath('utils'))
addpath(genpath('NMPC'))
addpath(genpath('C:\Users\user\Desktop\casadi-3.6.7-windows64-matlab2018b'))
addpath(genpath('C:\Users\leeck\Desktop\casadi-3.7.0-windows64-matlab2018b'))
USV_P = load_parm();

%% Simulation parameters
dt = 0.2; % Simulation step (fine-grained)
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
Tau_T = zeros(1, N);
Tau_del = zeros(1, N);
Tau_T_real = zeros(1, N);
rps_real = zeros(1, N);
Tau_del_real = zeros(1, N);
Tau_bow = zeros(1, N);

x_state(1) = 0;
y_state(1) = 0;
psi_state(1) = 0.0*pi;

MPC_state = [x_state(1), y_state(1), psi_state(1), 0, 0, 0, 0, 0]';
MPC_input = [0, 0, 0]';
MPC_input2 = [0, 0]';
MPC_pred = [];
MPC_ref = [];

T_cmd = 0;
alloc_T_cmd = 0;
T_bow_cmd = 0;
delP_cmd = 0;
del = 0;
thr = 0;
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

Q = diag([1 1 1e2 100 100 1000 1e-6 1]);
R = diag([1e-6 1 1e-3]);
R2 = diag([1e-6 1]);
QN = Num*diag([10 10 1e6 100 100 1000 1 1]);

% init casadi
c_sol = initialize_casadi(Num, control_update_dt, Q,R,QN);
c_sol.input.u0 = ones(Num,1).*MPC_input';
c_sol.input.X0 = repmat(MPC_state',1,Num+1)'; 

% init casadi 2nd
c_sol2 = initialize_casadi_2nd(Num, control_update_dt, Q,R2,QN);
c_sol2.input.u0 = ones(Num,1).*MPC_input2';
c_sol2.input.X0 = repmat(MPC_state',1,Num+1)'; 


%% Animation initialization
[ship_patch, path_line, h_thruster, q_thruster, bow_thruster, bow_thruster_arrow, pred_path_plot, reference_path_plot, subplot_axes] = plot_ship_animation_init(x_state(1), y_state(1), psi_state(1), Num);


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
    % if mod(i, control_update_steps) == 0
        tic;
        %% 1st MPC
        MPC_ref = [];
        
        % MPC setting
        c_sol.args.p(1:c_sol.nx+c_sol.nu) = [MPC_state; MPC_input];
        
        for k = 1:Num+1
            [ref_x,ref_y,ref_psi,ref_u,ref_v,ref_r,ref_T,ref_del] = reference_trajectory(i, dt, control_update_dt, k, mode, start_time);
            ref_state = [ref_x,ref_y,ref_psi,ref_u,ref_v,ref_r,ref_T,ref_del];
            c_sol.args.p(c_sol.nu + c_sol.nx*k + 1:c_sol.nu + c_sol.nx*k + 8) = ref_state;       
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
        % MPC_pred = xsol;

        % Get control input
        MPC_input = [usol(1,1), usol(1,2), usol(1,3)]';
        
        %% MILP
        dwell_time = 3;
        bow_array = bow_mapping(usol(:,3)/USV_P.bow_thrust,control_update_dt, Num, dwell_time);

        %% 2nd MPC
        MPC_ref = [];

        % MPC setting
        c_sol2.args.p(1:c_sol2.nx+c_sol2.nu) = [MPC_state; MPC_input2];
        
        for k = 1:Num+1
            [ref_x,ref_y,ref_psi,ref_u,ref_v,ref_r,ref_T,ref_del] = reference_trajectory(i, dt, control_update_dt, k, mode, start_time);
            ref_state = [ref_x,ref_y,ref_psi,ref_u,ref_v,ref_r,ref_T,ref_del];
            c_sol2.args.p(c_sol2.nu + c_sol2.nx*k + 1:c_sol2.nu + c_sol2.nx*k + 8) = ref_state;       
            MPC_ref = [MPC_ref; ref_state];
        end

        % Bow update
        n_controls = 2;
        n_states = 8;
        c_sol2.args.p(n_controls+n_states + n_states *(Num+1) + 1:n_controls+n_states + n_states *(Num+1) + Num) = bow_array;


        c_sol2.args.x0  = [reshape(c_sol2.input.X0',c_sol2.nx*(Num+1),1);
        reshape(c_sol2.input.u0',c_sol2.nu*Num,1)];
        
        % Solve the problem
        % tic
        sol2 = c_sol2.solver('x0', c_sol2.args.x0, 'lbx', c_sol2.args.lbx, 'ubx', c_sol2.args.ubx,...
            'lbg', c_sol2.args.lbg, 'ubg', c_sol2.args.ubg,'p',c_sol2.args.p);
        usol2 = reshape(full(sol2.x(c_sol2.nx*(Num+1)+1:end))',c_sol2.nu,Num)'; % get controls only from the solution
        xsol2 = reshape(full(sol2.x(1:c_sol2.nx*(Num+1)))',c_sol2.nx,Num+1)'; % get solution TRAJECTORY
        % toc
        c_sol2.input.X0 = [xsol2(2:end,:);xsol2(end,:)];
        c_sol2.input.u0 = [usol2(2:end,:);usol2(end,:)];
        MPC_pred = xsol2;

        d_T_cmd = usol2(1,1);
        d_del_cmd = usol2(1,2);

        T_cmd = xsol(1,7) + d_T_cmd*dt;
        del_cmd = xsol(1,8) + d_del_cmd*dt;
        T_bow_cmd = USV_P.bow_thrust*bow_array(1);

        [alloc_T_cmd] = thrust_allocation(T_cmd);
        MPC_input2 = [d_T_cmd, d_del_cmd]';
        toc
    % end
    WD = 0;
    WS = 0;

    %% Dynamic updatae
    [u_state(i), v_state(i), r_state(i), x_state(i), y_state(i), psi_state(i), thr, del, rps] = update_ship_dynamics(u_state(i-1), v_state(i-1), r_state(i-1), x_state(i-1), y_state(i-1), psi_state(i-1), thr, del, alloc_T_cmd, del_cmd, T_bow_cmd, dt, WD, WS);
        
    MPC_state = [x_state(i), y_state(i), psi_state(i), u_state(i), v_state(i), r_state(i), T_cmd, del_cmd]';
    
    
    %% Data log
    Tau_T(i) = alloc_T_cmd;
    Tau_del(i) = del_cmd;
    Tau_T_real(i) = thr;
    rps_real(i) = rps;
    Tau_del_real(i) = del;
    Tau_bow(i) = T_bow_cmd;
    % if mod(i,control_update_steps)==0  
    if mod(i,5)==0 && i > 0
        plot_ship_animation_update(i, ship_patch, path_line, h_thruster, q_thruster, bow_thruster, bow_thruster_arrow, pred_path_plot, reference_path_plot, x_state, y_state, psi_state, u_state, v_state, r_state, Tau_T, Tau_del, Tau_T_real, Tau_del_real, Tau_bow, subplot_axes, t, MPC_pred, MPC_ref, rps_real);
        
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


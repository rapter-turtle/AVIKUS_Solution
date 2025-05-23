clc; clear; close all;

%% Load Path
addpath(genpath('dynamics'))
addpath(genpath('plot'))
addpath(genpath('utils'))
addpath(genpath('NMPC'))
addpath(genpath('C:\Users\user\Desktop\qpSWIFT-main'))
addpath(genpath('C:\Users\user\Desktop\casadi-3.6.5-windows64-matlab2018b'))

%% Simulation parameters
dt = 0.01; % Simulation step (fine-grained)
control_update_dt = 0.1; % Control update interval
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
 


%% MPC setting
% Control params
Num = 50; 

% MPC Weight Parameters
Q = diag([100 100 400 0 0 0 1e-4 1e-4 1e-2 1e-2]);
R = diag([1e-4 1e-4 1e0 1e0]);
Rd = diag([1e-4 1e-4 1e0 1e0]);
QN = Q*Num;

% init casadi
c_sol = initialize_casadi(Num, control_update_dt, Q,R,Rd,QN);
c_sol.input.u0 = ones(Num,1).*MPC_input';
c_sol.input.X0 = repmat(MPC_state',1,Num+1)'; 


%% Animation initialization
[ship_patch, path_line, h_thruster_L, h_thruster_R, q_thruster_L, q_thruster_R, pred_path_plot, reference_path_plot, subplot_axes] = plot_ship_animation_init(x_state(1), y_state(1), psi_state(1), Num);


%% Simulation loop 
for i = 2:N

    %% Control policy (MPC)
    if mod(i, control_update_steps) == 0
        MPC_ref = [];

        % MPC setting
        c_sol.args.p(1:c_sol.nx+c_sol.nu) = [MPC_state; MPC_input];
        
        for k = 1:Num+1
            [ref_x,ref_y,ref_psi,ref_u,ref_v,ref_r,ref_TP,ref_TS,ref_delPR,ref_delSR] = reference_trajectory(i, dt, control_update_dt, k);
            ref_state = [ref_x,ref_y,ref_psi,ref_u,ref_v,ref_r,ref_TP,ref_TS,ref_delPR,ref_delSR];
            c_sol.args.p(c_sol.nu + c_sol.nx*k + 1:c_sol.nu + c_sol.nx*k + 10) = ref_state;       
            MPC_ref = [MPC_ref; ref_state];
        end

        c_sol.args.x0  = [reshape(c_sol.input.X0',c_sol.nx*(Num+1),1);
        reshape(c_sol.input.u0',c_sol.nu*Num,1)];
        
        % Solve the problem
        sol = c_sol.solver('x0', c_sol.args.x0, 'lbx', c_sol.args.lbx, 'ubx', c_sol.args.ubx,...
            'lbg', c_sol.args.lbg, 'ubg', c_sol.args.ubg,'p',c_sol.args.p);
        usol = reshape(full(sol.x(c_sol.nx*(Num+1)+1:end))',c_sol.nu,Num)'; % get controls only from the solution
        xsol= reshape(full(sol.x(1:c_sol.nx*(Num+1)))',c_sol.nx,Num+1)'; % get solution TRAJECTORY
        
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

        fprintf('The value of a is %.2f\n', TP_cmd);
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
    if mod(i,10)==0  
        plot_ship_animation_update(i, ship_patch, path_line, h_thruster_L, h_thruster_R, q_thruster_L, q_thruster_R, pred_path_plot, reference_path_plot, x_state, y_state, psi_state, u_state, v_state, r_state, Tau_TP, Tau_TS, Tau_delPR, Tau_delSR, Tau_TP_real, Tau_TS_real, Tau_delPR_real, Tau_delSR_real, subplot_axes, t, MPC_pred, MPC_ref);
    end  
end


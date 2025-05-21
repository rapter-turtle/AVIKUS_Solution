clc; clear; close all;
yalmip('clear');

%% Simulation parameters
dt = 0.01; % Simulation step (fine-grained)
control_update_dt = 0.1; % Control update interval
control_update_steps = control_update_dt / dt; % Control update every 10 steps
T = 100; % Simulation duration
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


TP_cmd = 0;
TS_cmd = 0;
delPR_cmd = 0;
delSR_cmd = 0;
delPR = 0;
delSR = 0;
thrP = 0;
thrS = 0;
WX = 0;
WY = 0;
WN = 0;



%% Animation initialization
[ship_patch, path_line, h_thruster_L, h_thruster_R, q_thruster_L, q_thruster_R, subplot_axes] = plot_ship_animation_init(x_state(1), y_state(1), psi_state(1));


%% Simulation loop
for i = 2:N

    %% Control policy
    if mod(i, control_update_steps) == 0
        TP_cmd = 5;
        TS_cmd = 5;
        delPR_cmd = 0;
        delSR_cmd = 0;
    end

    %% Dynamic updatae
    [u_state(i), v_state(i), r_state(i), x_state(i), y_state(i), psi_state(i), thrP, thrS, delPR, delSR] = update_ship_dynamics(u_state(i-1), v_state(i-1), r_state(i-1), x_state(i-1), y_state(i-1), psi_state(i-1), thrP, thrS, delPR, delSR, TP_cmd, TS_cmd, delPR_cmd, delSR_cmd, dt, WX, WY, WN);
    
    
    
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
        plot_ship_animation_update(i, ship_patch, path_line, h_thruster_L, h_thruster_R, q_thruster_L, q_thruster_R, x_state, y_state, psi_state, u_state, v_state, r_state, Tau_TP, Tau_TS, Tau_delPR, Tau_delSR, Tau_TP_real, Tau_TS_real, Tau_delPR_real, Tau_delSR_real, subplot_axes, t);
    end  
end


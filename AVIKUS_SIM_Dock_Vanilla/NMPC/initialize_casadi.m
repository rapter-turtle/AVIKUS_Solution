function obj = initialize_casadi(Num,dt,Q,R,Rd,QN)
import casadi.*
USV_P = load_parm();
%% Parameter setting - States
x = SX.sym('x'); 
y = SX.sym('u');
u = SX.sym('u');
v = SX.sym('v');
r = SX.sym('r');
psi = SX.sym('psi');
TP = SX.sym('TP'); 
TS = SX.sym('TS'); 
delPR = SX.sym('delPR');
delSR = SX.sym('delSR'); 
states = [x;y;psi;u;v;r;TP;TS;delPR;delSR]; 
n_states = length(states);

%% Parameter setting - Inputs
d_TP = SX.sym('d_TP'); 
d_TS = SX.sym('d_TS'); 
d_delPR = SX.sym('d_delPR');
d_delSR = SX.sym('d_delSR'); 
controls = [d_TP;d_TS;d_delPR;d_delSR]; 
n_controls = length(controls);


%% Casadi setting   
U = SX.sym('U',n_controls,Num); 
P = SX.sym('P',n_controls+n_states + n_states *(Num+1));
X = SX.sym('X',n_states,(Num+1));

objective = 0; % Objective function
g = [];  % constraints vector

st  = X(:,1); % initial state
g = [g;st-P(1:n_states)]; % initial condition constraints

%% calculate objective function
for k = 1:Num
    st = X(:,k);  
    con = U(:,k); 
    ref_state = P(n_controls+n_states*k + 1:n_controls+n_states*k + n_states);   
    objective = objective + (st-ref_state)'*Q*(st-ref_state);

    %% USV dynamics
    st_next = X(:,k+1);
    objective = objective + con'*R*con;
    
    if k < Num
        con_next = U(:,k+1);
        objective = objective + (con_next-con)'*Rd*(con_next-con);
    end
    % if k == 1
    %    pre_con = P(n_states+1:n_states+n_controls);
    %    objective = objective + (pre_con-con)'*Rd*(pre_con-con);
    % end

    xdot = usv_dynamics_mpc(st, con);
    st_next_euler = st + (dt*xdot);
    g = [g;st_next-st_next_euler]; % compute constraints


end

st = X(:,Num+1);  
ref_state = P(n_controls+n_states*(Num+1) + 1: n_controls+n_states*(Num+1) + n_states);   
objective = objective + (st-ref_state)'*QN*(st-ref_state);

OPT_variables = [reshape(X,n_states*(Num+1),1);reshape(U,n_controls*Num,1)];
nlp_prob = struct('f', objective, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 25;
opts.ipopt.print_level = 0; % 0 ~ 3
opts.print_time = 0;
opts.ipopt.acceptable_tol = 1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-8;

obj.solver = nlpsol('solver', 'ipopt', nlp_prob,opts);
n_params = 1;
obj.nx = n_states;
obj.nu = n_controls;
obj.np = n_params;

%% Equaility contraints : Dyanmic equations 
obj.args.lbg(1:n_states*(Num+1)) = -1e-10;  % -1e-20   % Equality constraints
obj.args.ubg(1:n_states*(Num+1)) =  1e-10;  %  1e-20   % Equality constraints


xmin = -1000;
xmax = 13;
ymin = -1;
ymax = 3;
psimin = -pi;
psimax = pi;
umin = -1.5;
umax = 1.5;
vmin = -1.5;
vmax = 1.5;
rmin = -1;
rmax = 1;
TPmin = -USV_P.ThrMax * USV_P.KPrvs;
TPmax = USV_P.ThrMax * USV_P.KPfwd;
TSmin = -USV_P.ThrMax * USV_P.KPrvs;
TSmax = USV_P.ThrMax * USV_P.KPfwd;
delPRmin = -USV_P.DelMax;
delPRmax = USV_P.DelMax;
delSRmin = -USV_P.DelMax;
delSRmax = USV_P.DelMax;

d_TPmin = -USV_P.ThrRate;
d_TPmax = USV_P.ThrRate;
d_TSmin = -USV_P.ThrRate;
d_TSmax = USV_P.ThrRate;
d_delPRmin = -USV_P.DelRate*0.5;
d_delPRmax = USV_P.DelRate*0.5;
d_delSRmin = -USV_P.DelRate*0.5;
d_delSRmax = USV_P.DelRate*0.5;

%% Inequaility contraints : States & Inputs
obj.args.lbx(1:n_states:n_states*(Num+1),1) =  xmin;
obj.args.ubx(1:n_states:n_states*(Num+1),1) =  xmax;
obj.args.lbx(2:n_states:n_states*(Num+1),1) =  ymin;
obj.args.ubx(2:n_states:n_states*(Num+1),1) =  ymax;
obj.args.lbx(3:n_states:n_states*(Num+1),1) =  psimin;
obj.args.ubx(3:n_states:n_states*(Num+1),1) =  psimax;
obj.args.lbx(4:n_states:n_states*(Num+1),1) =  umin;
obj.args.ubx(4:n_states:n_states*(Num+1),1) =  umax;
obj.args.lbx(5:n_states:n_states*(Num+1),1) =  vmin;
obj.args.ubx(5:n_states:n_states*(Num+1),1) =  vmax;
obj.args.lbx(6:n_states:n_states*(Num+1),1) =  rmin;
obj.args.ubx(6:n_states:n_states*(Num+1),1) =  rmax;
obj.args.lbx(7:n_states:n_states*(Num+1),1) =  TPmin;
obj.args.ubx(7:n_states:n_states*(Num+1),1) =  TPmax;
obj.args.lbx(8:n_states:n_states*(Num+1),1) =  TSmin;
obj.args.ubx(8:n_states:n_states*(Num+1),1) =  TSmax;
obj.args.lbx(9:n_states:n_states*(Num+1),1) =  delPRmin;
obj.args.ubx(9:n_states:n_states*(Num+1),1) =  delPRmax;
obj.args.lbx(10:n_states:n_states*(Num+1),1) =  delSRmin;
obj.args.ubx(10:n_states:n_states*(Num+1),1) =  delSRmax;


obj.args.lbx(n_states*(Num+1)+1:n_controls:n_states*(Num+1)+n_controls*Num,1) = d_TPmin;
obj.args.ubx(n_states*(Num+1)+1:n_controls:n_states*(Num+1)+n_controls*Num,1) = d_TPmax;
obj.args.lbx(n_states*(Num+1)+2:n_controls:n_states*(Num+1)+n_controls*Num,1) = d_TSmin;
obj.args.ubx(n_states*(Num+1)+2:n_controls:n_states*(Num+1)+n_controls*Num,1) = d_TSmax;
obj.args.lbx(n_states*(Num+1)+3:n_controls:n_states*(Num+1)+n_controls*Num,1) = d_delPRmin;
obj.args.ubx(n_states*(Num+1)+3:n_controls:n_states*(Num+1)+n_controls*Num,1) = d_delPRmax;
obj.args.lbx(n_states*(Num+1)+4:n_controls:n_states*(Num+1)+n_controls*Num,1) = d_delSRmin;
obj.args.ubx(n_states*(Num+1)+4:n_controls:n_states*(Num+1)+n_controls*Num,1) = d_delSRmax;

disp('*************** MPC setting ***************')
disp(strcat('Prediction Num :` ' , num2str(Num)))
disp(strcat('Prediction Ts :` ' , num2str(dt) ,'sec'))
disp('Integration algorithm : Euler method')
end

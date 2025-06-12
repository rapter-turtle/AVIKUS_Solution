function obj = initialize_casadi_2nd(Num,dt,Q,R,QN)
import casadi.*
USV_P = load_parm();
%% Parameter setting - States
x = SX.sym('x'); 
y = SX.sym('y');
psi = SX.sym('psi');
u = SX.sym('u');
v = SX.sym('v');
r = SX.sym('r');
T = SX.sym('T'); 
del = SX.sym('delPR');
states = [x;y;psi;u;v;r;T;del]; 
n_states = length(states);

%% Parameter setting - Inputs
d_T = SX.sym('d_T'); 
d_del = SX.sym('d_del');
controls = [d_T;d_del]; 
n_controls = length(controls);


%% Casadi setting   
U = SX.sym('U',n_controls,Num); 
P = SX.sym('P',n_controls+n_states + n_states *(Num+1) + Num);
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

    %% Bow
    Bow = P(n_controls+n_states + n_states *(Num+1) + k);

    %% USV dynamics
    st_next = X(:,k+1);
    objective = objective + con'*R*con;

    xdot = usv_dynamics_mpc_2nd(st, con, Bow);
    st_next_euler = st + (dt*xdot);
    g = [g;st_next-st_next_euler]; % compute constraints
    
    %% Dead zone
    param_s = 7;
    param_m = 10;
    
    lower_bound = -USV_P.dead_rps * USV_P.KPrvs;
    upper_bound =  USV_P.dead_rps * USV_P.KPfwd;
    sig_lo_7 = 1 / (1 + exp(-param_s*(X(7,k) - lower_bound)));
    sig_hi_7 = 1 / (1 + exp(-param_s*(upper_bound - X(7,k))));

    % objective = objective + param_m * (sig_lo_7 * sig_hi_7);
    
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


xmin = -10;
xmax = 13;
ymin = -10;
ymax = 10;
psimin = -pi;
psimax = pi;
umin = -1.5;
umax = 1.5;
vmin = -1.5;
vmax = 1.5;
rmin = -1;
rmax = 1;
Tmin = -USV_P.ThrMax * USV_P.KPrvs;
Tmax = USV_P.ThrMax * USV_P.KPfwd;
delmin = -USV_P.DelMax;
delmax = USV_P.DelMax;

d_Tmin = -USV_P.ThrRate;
d_Tmax = USV_P.ThrRate;
d_delmin = -USV_P.DelRate*0.5;
d_delmax = USV_P.DelRate*0.5;


%% Inequaility contraints : States & Inputs
obj.args.lbx(1:n_states:n_states*(Num+1),1) =  USV_P.xmin;
obj.args.ubx(1:n_states:n_states*(Num+1),1) =  USV_P.xmax;
obj.args.lbx(2:n_states:n_states*(Num+1),1) =  USV_P.ymin;
obj.args.ubx(2:n_states:n_states*(Num+1),1) =  USV_P.ymax;
obj.args.lbx(3:n_states:n_states*(Num+1),1) =  psimin;
obj.args.ubx(3:n_states:n_states*(Num+1),1) =  psimax;
obj.args.lbx(4:n_states:n_states*(Num+1),1) =  umin;
obj.args.ubx(4:n_states:n_states*(Num+1),1) =  umax;
obj.args.lbx(5:n_states:n_states*(Num+1),1) =  vmin;
obj.args.ubx(5:n_states:n_states*(Num+1),1) =  vmax;
obj.args.lbx(6:n_states:n_states*(Num+1),1) =  rmin;
obj.args.ubx(6:n_states:n_states*(Num+1),1) =  rmax;
obj.args.lbx(7:n_states:n_states*(Num+1),1) =  Tmin;
obj.args.ubx(7:n_states:n_states*(Num+1),1) =  Tmax;
obj.args.lbx(8:n_states:n_states*(Num+1),1) =  delmin;
obj.args.ubx(8:n_states:n_states*(Num+1),1) =  delmax;


obj.args.lbx(n_states*(Num+1)+1:n_controls:n_states*(Num+1)+n_controls*Num,1) = d_Tmin;
obj.args.ubx(n_states*(Num+1)+1:n_controls:n_states*(Num+1)+n_controls*Num,1) = d_Tmax;
obj.args.lbx(n_states*(Num+1)+2:n_controls:n_states*(Num+1)+n_controls*Num,1) = d_delmin;
obj.args.ubx(n_states*(Num+1)+2:n_controls:n_states*(Num+1)+n_controls*Num,1) = d_delmax;

disp('*************** MPC setting ***************')
disp(strcat('Prediction Num :` ' , num2str(Num)))
disp(strcat('Prediction Ts :` ' , num2str(dt) ,'sec'))
disp('Integration algorithm : Euler method')
end

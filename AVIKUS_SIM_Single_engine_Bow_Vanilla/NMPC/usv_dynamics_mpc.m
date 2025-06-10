function [sys] = usv_dynamics_mpc(x,uu)
    P = load_parm();

    % x_state = x(1);
    % y_state = x(2);
    psi_state = x(3);
    u_state = x(4);
    v_state = x(5);
    r_state = x(6);
    T_state = x(7);
    del_state = x(8);

    d_T = uu(1);
    d_del = uu(2);
    T_bow = uu(3);
    
    % state derivative vector
    eps = 0.0001;

    XH = P.Xu*u_state + P.Xvr*v_state*r_state;
    YH = P.Yv*v_state + P.Yuv*u_state*v_state + P.Yvv*v_state*sqrt(v_state*v_state + eps) + P.Yr*r_state;
    NH = P.Nv*v_state + P.Nuv*u_state*v_state + P.Nvv*v_state*sqrt(v_state*v_state + eps) + P.Nr*r_state + P.Nrr*r_state*sqrt(r_state*r_state + eps);

    % Propulsion force
    XP = T_state*cos(del_state);
    YP = -T_state*sin(del_state) + T_bow;
    NP = -P.xp*YP + 2.0*T_bow;

    % Dynamics
    ug_dot = (XH + XP + v_state*r_state)/P.M;
    vg_dot = (YH + YP - u_state*r_state)/P.M;
    rg_dot = (NH + NP)/P.Izz;

    xdot = [u_state*cos(psi_state) - v_state*sin(psi_state);...
            u_state*sin(psi_state) + v_state*cos(psi_state);...
            r_state;...
            ug_dot;...
            vg_dot;...
            rg_dot;...
            d_T;...
            d_del];   
    
    sys = xdot;
end

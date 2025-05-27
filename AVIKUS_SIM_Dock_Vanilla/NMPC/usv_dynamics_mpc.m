function [sys] = usv_dynamics_mpc(x,uu)
    P = load_parm();

    x_state = x(1);
    y_state = x(2);
    psi_state = x(3);
    u_state = x(4);
    v_state = x(5);
    r_state = x(6);
    TP_state = x(7);
    TS_state = x(8);
    delPR_state = x(9);
    delSR_state = x(10);

    d_TP = uu(1);
    d_TS = uu(2);
    d_delPR = uu(3);
    d_delSR = uu(4);
    
    % state derivative vector
    eps = 0.0001;

    XH = P.Xu*u_state + P.Xvr*v_state*r_state;
    YH = P.Yv*v_state + P.Yuv*u_state*v_state + P.Yvv*v_state*sqrt(v_state*v_state + eps) + P.Yr*r_state;
    NH = P.Nv*v_state + P.Nuv*u_state*v_state + P.Nvv*v_state*sqrt(v_state*v_state + eps) + P.Nr*r_state + P.Nrr*r_state*sqrt(r_state*r_state + eps);

    % Propulsion force
    XP = TP_state*cos(delPR_state) + TS_state*cos(delSR_state);
    YP = -TP_state*sin(delPR_state) - TS_state*sin(delSR_state);
    NP = -P.xp*YP + P.yp*(TP_state*cos(delPR_state) - TS_state*cos(delSR_state));

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
            d_TP;...
            d_TS;...
            d_delPR;...
            d_delSR];   
    
    sys = xdot;
end

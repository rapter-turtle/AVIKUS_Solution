function [u_next, v_next, r_next, x_next, y_next, psi_next, thr, del, rps] = update_ship_dynamics(u, v, r, x, y, psi, thr0, del0, T_cmd, del_cmd, T_bow_cmd, dt, WD, WS)

    % Actuator Dynamics
    DelRate = 100*3.141592/180; % deg/s
    DelMax = 30*3.141592/180;
    ThrRate = 100; % percent/s
    ThrMax = 100;

    P = load_parm();
    % Geometric
    L = P.L;
    M = P.M;
    Izz = P.Izz;
    yp = P.yp;
    xp = P.xp;
    AT = P.AT;
    AL = P.AL;

    uuu = zeros(1, 8);
    vvv = zeros(1, 8);
    rrr = zeros(1, 8);

    uuu(1) = P.Xu;
    uuu(2) = P.Xvv;
    uuu(3) = P.Xuvv;
    uuu(4) = P.Xvvvv;
    uuu(5) = P.Xrr;
    uuu(6) = P.Xvr;
    uuu(7) = P.KPfwd;
    uuu(8) = P.KPrvs;
    vvv(1) = P.Yv;
    vvv(2) = P.Yuv;
    vvv(3) = P.Yvv;
    vvv(4) = P.Yuvv;
    vvv(5) = P.Yr;
    vvv(6) = P.Yrr;
    vvv(7) = P.Yvvr;
    vvv(8) = P.Yvrr;
    rrr(1) = P.Nv;
    rrr(2) = P.Nuv;
    rrr(3) = P.Nvv;
    rrr(4) = P.Nuvv;
    rrr(5) = P.Nr;
    rrr(6) = P.Nrr;
    rrr(7) = P.Nvvr;
    rrr(8) = P.Nvrr;

    uuuF = uuu(7);
    uuuR = uuu(8);


    %% Steer Model
    del_new = sign(del_cmd-del0)*DelRate*dt + del0;
    
    if (del_cmd-del0)*(del_cmd-del_new)<0
        del_new = del_cmd;
    end
    
    del = minmax(-DelMax, del_new, DelMax);


    %% Thruster Model TP_cmd, TS_cmd
    thr_new = sign(T_cmd-thr0)*ThrRate*dt + thr0;

    if (T_cmd-thr0)*(T_cmd-thr_new)<0
        thr_new = T_cmd;
    end

    thr = minmax(-ThrMax, thr_new, ThrMax);

    % Dead zone
    rps = P.dead_rps*sign(thr) + thr;

    if rps >= 0
        T = uuuF*rps;
    else
        T = uuuR*rps;
    end


    %% Wind

    WD = deg2rad(WD)+pi;
    WS = WS*0.5144;
    AWS = sqrt(WS^2);
    spd = sqrt(u^2 + v^2);
    if(WS*cos(WD) == spd*cos(psi))
        if (WS*sin(WD) > spd*sin(psi))
            AWD = pi/2;
        elseif (WS*sin(WD) < spd*sin(psi))
            AWD = -pi/2;
        else
            AWD = 0;
        end
    else
        AWD = atan2((WS*sin(WD) - spd*sin(psi)),(WS*cos(WD)-spd*cos(psi)));
    end
    
    gamma = -AWD -pi + psi;
    
    % wind coefficient
    CX = -P.cx*cos(gamma);
    CY = P.cy*sin(gamma);
    CN = P.cn*sin(2*(gamma));
    
    % wind force
    WX = CX*(1/2 * 1.225 * AWS^2 * AT);
    WY = CY*(1/2 * 1.225 * AWS^2 * AL);
    WN = CN*(1/2 * 1.225 * AWS^2 * L* AL);

    %% Dynamic equation
    %  Hydrodynamic forces
    XH = uuu(1)*u + uuu(2)*v^2 + uuu(3)*u*v^2 + uuu(4)*v^4 + uuu(5)*r^2 + uuu(6)*v*r;
    YH = vvv(1)*v + vvv(2)*u*v + vvv(3)*v*abs(v) + vvv(4)*u*v*abs(v) + vvv(5)*r + vvv(6)*r*abs(r) + vvv(7)*v^2*r + vvv(8)*v*r^2;
    NH = rrr(1)*v + rrr(2)*u*v + rrr(3)*v*abs(v) + rrr(4)*u*v*abs(v) + rrr(5)*r + rrr(6)*r*abs(r) + rrr(7)*v^2*r + rrr(8)*v*r^2;
    % XH = Xu*u + Xvr*v*r;
    % YH = Yv*v + Yuv*u*v + Yvv*v*sqrt(v*v + eps) + Yr*r;
    % NH = Nv*v + Nuv*u*v + Nvv*v*sqrt(v*v + eps) + Nr*r + Nrr*r*sqrt(r*r + eps);

    % T= T_cmd;
    % Propulsion force
    XP = T*cos(del);
    YP = -T*sin(del) + T_bow_cmd;
    NP = -xp*YP + 2.0*T_bow_cmd;

    % Dynamics
    ug_dot = (XH + XP + WX + v*r)/M;
    vg_dot = (YH + YP + WY - u*r)/M;
    rg_dot = (NH + NP + WN)/Izz;

    % State integration
    u_next = u + ug_dot * dt;
    v_next = v + vg_dot * dt;
    r_next = r + rg_dot * dt;
    x_next = x + (u_next*cos(psi) - v_next*sin(psi)) * dt;
    y_next = y + (u_next*sin(psi) + v_next*cos(psi)) * dt;
    psi_next = Bound2Pi(psi + r_next*dt);
end

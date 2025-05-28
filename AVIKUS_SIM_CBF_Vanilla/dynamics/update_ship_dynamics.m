function [u_next, v_next, r_next, x_next, y_next, psi_next, thrP, thrS, delPR, delSR] = update_ship_dynamics(u, v, r, x, y, psi, thrP0, thrS0, delP0, delS0, TP_cmd, TS_cmd, delPR_cmd, delSR_cmd, dt, WX, WY, WN)

    % Actuator Dynamics
    DelRate = 100*3.141592/180; % deg/s
    DelMax = 30*3.141592/180;
    ThrRate = 100; % percent/s
    ThrMax = 100;

    % Geometric
    L = 9;
    B = 3;
    M = 4000; % mass [kg], Maximum mass is 4600
    kzz = 0.25*L;
    Izz = kzz^2*M;
    yp = 0.5;
    xp = 2;
    b = 15.722;
    AT = 4;
    AL = 10;

    % Dynamic model
    Xu    =  -189.76;
    Xvv   =  54.32;
    Xuvv  =  167.74;
    Xvvvv =  2076.9;
    Xrr   =  -15842;
    Xvr   =  -756.37;
    Yv    =  -950.31;
    Yuv   =  818.67;
    Yvv   =  -3054.3;
    Yuvv  =  -1589.1;
    Yr    =  1183.2;
    Yrr   =  9661.3;
    Yvvr  =  54001;
    Yvrr  =  71332;
    Nv    =  540.22;
    Nuv   =  -303.61;
    Nvv   =  1244.9;
    Nuvv  =  188.83;
    Nr    =  -6530.9;
    Nrr   =  23075;
    Nvvr  =  -36088;
    Nvrr  =  20781;
    KPfwd =  1.7;
    KPrvs =  1.0;

    uuu = zeros(1, 8);
    vvv = zeros(1, 8);
    rrr = zeros(1, 8);

    uuu(1) = Xu;
    uuu(2) = Xvv;
    uuu(3) = Xuvv;
    uuu(4) = Xvvvv;
    uuu(5) = Xrr;
    uuu(6) = Xvr;
    uuu(7) = KPfwd;
    uuu(8) = KPrvs;
    vvv(1) = Yv;
    vvv(2) = Yuv;
    vvv(3) = Yvv;
    vvv(4) = Yuvv;
    vvv(5) = Yr/2;
    vvv(6) = Yrr/2;
    vvv(7) = Yvvr/100;
    vvv(8) = Yvrr/100;
    rrr(1) = Nv;
    rrr(2) = Nuv;
    rrr(3) = Nvv;
    rrr(4) = Nuvv;
    rrr(5) = Nr;
    rrr(6) = Nrr;
    rrr(7) = Nvvr/100;
    rrr(8) = Nvrr/100;

    uuuF = uuu(7);
    uuuR = uuu(8);


    %% Steer Model
    delP_new = sign(delPR_cmd-delP0)*DelRate*dt + delP0;
    delS_new = sign(delSR_cmd-delS0)*DelRate*dt + delS0;
    
    if (delPR_cmd-delP0)*(delPR_cmd-delP_new)<0
        delP_new = delPR_cmd;
    end
    if (delSR_cmd-delS0)*(delSR_cmd-delS_new)<0
        delS_new = delSR_cmd;
    end
    
    delPR = minmax(-DelMax, delP_new, DelMax);
    delSR = minmax(-DelMax, delS_new, DelMax);


    %% Thruster Model TP_cmd, TS_cmd
    thrP_new = sign(TP_cmd-thrP0)*ThrRate*dt + thrP0;
    thrS_new = sign(TS_cmd-thrS0)*ThrRate*dt + thrS0;

    if (TP_cmd-thrP0)*(TP_cmd-thrP_new)<0
        thrP_new = TP_cmd;
    end
    if (TS_cmd-thrS0)*(TS_cmd-thrS_new)<0
        thrS_new = TS_cmd;
    end

    thrP = minmax(-ThrMax, thrP_new, ThrMax);
    thrS = minmax(-ThrMax, thrS_new, ThrMax);

    % Dead zone
    rpsP = 10*sign(thrP) + thrP;
    rpsS = 10*sign(thrS) + thrS;
    if abs(rpsP)<11
        rpsP = 0;
    end
    if abs(rpsS)<11
        rpsS = 0;
    end

    % RPM2FORCE
    if rpsP >= 0
        TP = uuuF*rpsP*abs(rpsP);
    else
        TP = uuuR*rpsP*abs(rpsP);
    end
    
    if rpsS >= 0
        TS = uuuF*rpsS*abs(rpsS);
    else
        TS = uuuR*rpsS*abs(rpsS);
    end


    %% Dynamic equation
    %  Hydrodynamic forces
    XH = uuu(1)*u + uuu(2)*v^2 + uuu(3)*u*v^2 + uuu(4)*v^4 + uuu(5)*r^2 + uuu(6)*v*r;
    YH = vvv(1)*v + vvv(2)*u*v + vvv(3)*v*abs(v) + vvv(4)*u*v*abs(v) + vvv(5)*r + vvv(6)*r*abs(r) + vvv(7)*v^2*r + vvv(8)*v*r^2;
    NH = rrr(1)*v + rrr(2)*u*v + rrr(3)*v*abs(v) + rrr(4)*u*v*abs(v) + rrr(5)*r + rrr(6)*r*abs(r) + rrr(7)*v^2*r + rrr(8)*v*r^2;
    % XH = Xu*u + Xvr*v*r;
    % YH = Yv*v + Yuv*u*v + Yvv*v*sqrt(v*v + eps) + Yr*r;
    % NH = Nv*v + Nuv*u*v + Nvv*v*sqrt(v*v + eps) + Nr*r + Nrr*r*sqrt(r*r + eps);
    
    Fx = 100;
    Fy = 100;
    WX = Fx*cos(psi) + Fy*sin(psi);
    WY = -Fx*sin(psi) + Fy*cos(psi);

    % Propulsion force
    XP = TP*cos(delPR) + TS*cos(delSR);
    YP = -TP*sin(delPR) - TS*sin(delSR);
    NP = -xp*YP + yp*(TP*cos(delPR) - TS*cos(delSR));

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

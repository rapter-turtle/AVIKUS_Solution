function [u_next, v_next, r_next, x_next, y_next, psi_next, thrP, thrS, delPR, delSR, rpsP, rpsS] = update_ship_dynamics(u, v, r, x, y, psi, thrP0, thrS0, delP0, delS0, TP_cmd, TS_cmd, delPR_cmd, delSR_cmd, dt, WX, WY, WN)

    % Actuator Dynamics
    DelRate = 100*3.141592/180; % deg/s
    DelMax = 30*3.141592/180;
    ThrRate = 100; % percent/s
    ThrMax = 100;

    P = load_parm();
    % Geometric
    L = P.L;
    M = P.M;
    kzz = P.kzz;
    Izz = P.Izz;
    yp = P.yp;
    xp = P.xp;
    b = P.b;
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
    rpsP = P.dead_rps*sign(thrP) + thrP;
    rpsS = P.dead_rps*sign(thrS) + thrS;
    % if abs(rpsP)<11
    %     rpsP = 0;
    % end
    % if abs(rpsS)<11
    %     rpsS = 0;
    % end


    
    % % RPM2FORCE
    % if rpsP >= 0
    %     TP = uuuF*rpsP*abs(rpsP);
    % else
    %     TP = uuuR*rpsP*abs(rpsP);
    % end
    % 
    % if rpsS >= 0
    %     TS = uuuF*rpsS*abs(rpsS);
    % else
    %     TS = uuuR*rpsS*abs(rpsS);
    % end
    % 
    % RPM2FORCE
    if rpsP >= 0
        TP = uuuF*rpsP;
    else
        TP = uuuR*rpsP;
    end

    if rpsS >= 0
        TS = uuuF*rpsS;
    else
        TS = uuuR*rpsS;
    end


    %% Dynamic equation
    %  Hydrodynamic forces
    XH = uuu(1)*u + uuu(2)*v^2 + uuu(3)*u*v^2 + uuu(4)*v^4 + uuu(5)*r^2 + uuu(6)*v*r;
    YH = vvv(1)*v + vvv(2)*u*v + vvv(3)*v*abs(v) + vvv(4)*u*v*abs(v) + vvv(5)*r + vvv(6)*r*abs(r) + vvv(7)*v^2*r + vvv(8)*v*r^2;
    NH = rrr(1)*v + rrr(2)*u*v + rrr(3)*v*abs(v) + rrr(4)*u*v*abs(v) + rrr(5)*r + rrr(6)*r*abs(r) + rrr(7)*v^2*r + rrr(8)*v*r^2;
    % XH = Xu*u + Xvr*v*r;
    % YH = Yv*v + Yuv*u*v + Yvv*v*sqrt(v*v + eps) + Yr*r;
    % NH = Nv*v + Nuv*u*v + Nvv*v*sqrt(v*v + eps) + Nr*r + Nrr*r*sqrt(r*r + eps);


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

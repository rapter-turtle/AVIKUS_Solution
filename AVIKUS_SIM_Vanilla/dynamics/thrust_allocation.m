function [alloc_TP_cmd, alloc_TS_cmd] = thrust_allocation(TP_cmd, TS_cmd)

    % Actuator Dynamics
    ThrRate = 100; % percent/s
    ThrMax = 100;
    KPfwd =  1.7;
    KPrvs =  1.0;    
    uuuF = KPfwd;
    uuuR = KPrvs;

    %TP
    if TP_cmd == 0
        alloc_TP_cmd = 0;
        return;
    end

    if TP_cmd > 0
        alloc_TP_cmd = sqrt(TP_cmd / uuuF);
    else
        alloc_TP_cmd = -sqrt(abs(TP_cmd) / uuuR);
    end

    %TS
    if TS_cmd == 0
        alloc_TS_cmd  = 0;
        return;
    end

    if TS_cmd > 0
        alloc_TS_cmd = sqrt(TS_cmd / uuuF);
    else
        alloc_TS_cmd = -sqrt(abs(TS_cmd) / uuuR);
    end

end

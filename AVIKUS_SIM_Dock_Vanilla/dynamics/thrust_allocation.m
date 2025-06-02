function [alloc_TP_cmd, alloc_TS_cmd] = thrust_allocation(TP_cmd, TS_cmd)

    % Actuator Dynamics
    P = load_parm();
    uuuF = P.KPfwd;
    uuuR = P.KPrvs;

    % Force -> RPS
    if TP_cmd <= 11*uuuF
        if TP_cmd >= -11*uuuR
            if TP_cmd > 0 
                alloc_TP_rps = 11;
            else
                alloc_TP_rps = -11;
            end
        end
    end

    if TP_cmd > 11*uuuF
        alloc_TP_rps = TP_cmd/uuuF;
    elseif TP_cmd < -11*uuuF
        alloc_TP_rps = TP_cmd/uuuR;
    end
    % RPS -> CMD
    alloc_TP_cmd = alloc_TP_rps - 10*sign(alloc_TP_rps);

    %TS

    % Force -> RPS
    if TS_cmd <= 11*uuuF
        if TS_cmd >= -11*uuuR
            if TS_cmd > 0 
                alloc_TS_rps = 11;
            else
                alloc_TS_rps = -11;
            end
        end
    end

    if TS_cmd > 11*uuuF
        alloc_TS_rps = TS_cmd/uuuF;
    elseif TS_cmd < -11*uuuF
        alloc_TS_rps = TS_cmd/uuuR;
    end
    % RPS -> CMD
    alloc_TS_cmd = alloc_TS_rps - 10*sign(alloc_TS_rps);

end

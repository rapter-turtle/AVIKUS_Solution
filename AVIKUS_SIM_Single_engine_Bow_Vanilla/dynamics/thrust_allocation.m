function [alloc_T_cmd] = thrust_allocation(T_cmd)

    % Actuator Dynamics
    P = load_parm();
    uuuF = P.KPfwd;
    uuuR = P.KPrvs;
    
    % Force -> RPS
    if T_cmd <= P.dead_rps*uuuF
        if T_cmd >= -P.dead_rps*uuuR
            if T_cmd > 0 
                alloc_T_rps = P.dead_rps;
            else
                alloc_T_rps = -P.dead_rps;
            end
        end
    end

    if T_cmd > P.dead_rps*uuuF
        alloc_T_rps = T_cmd/uuuF;
    elseif T_cmd < -P.dead_rps*uuuR
        alloc_T_rps = T_cmd/uuuR;
    end

    % RPS -> CMD
    alloc_T_cmd = alloc_T_rps - P.dead_rps*sign(alloc_T_rps);

   
end

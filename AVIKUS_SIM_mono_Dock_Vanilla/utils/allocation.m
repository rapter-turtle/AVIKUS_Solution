function [TP, TS, TP_delta, TS_delta]=allocation(Fx, Fy, Fn, TP_before, TS_before, TP_delta_before, TS_delta_before)
    import casadi.*
    P = load_parm();
    
    % Decision variables
    tp = MX.sym('x', 7);
    % tp1 = TP
    % tp2 = TS
    % tp3 = TP_delta
    % tp4 = TS_delta
    % tp5 = s1
    % tp6 = s2
    % tp7 = s3
    % XP = TP_state*cos(delPR_state) + TS_state*cos(delSR_state);
    % YP = -TP_state*sin(delPR_state) - TS_state*sin(delSR_state);
    % NP = -P.xp*YP + P.yp*(TP_state*cos(delPR_state) - TS_state*cos(delSR_state));

    % Objective function
    f = (tp(1)-TP_before)^2 + (tp(2)-TS_before)^2 + 1e7*(tp(3)-TP_delta_before)^2 + 1e7*(tp(4)-TS_delta_before)^2 + 1e20*tp(5)^2 + 1e20*tp(6)^2 + 1e20*tp(7)^2;
    
    % Constraints
    g = [tp(1)*cos(tp(3)) + tp(2)*cos(tp(4)) - Fx + tp(5);
         -tp(1)*sin(tp(3)) - tp(2)*sin(tp(4)) - Fy + tp(6);
         -P.xp*Fy + P.yp*(tp(1)*cos(tp(3)) - tp(2)*cos(tp(4))) - Fn + tp(7);
         tp(1);
         tp(2);
         tp(3);
         tp(4)];
    
    % NLP 설정
    nlp = struct('x', tp, 'f', f, 'g', g);
    
    % Solver 생성
    opts = struct('ipopt', struct( ...
        'print_level', 0, ...        % 이 부분을 ipopt 옵션 안에 넣기
        'tol', 1e-4, ...
        'constr_viol_tol', 1e-4, ...
        'compl_inf_tol', 1e-4, ...
        'max_iter', 100));
    opts.print_time = false;
    solver = nlpsol('solver', 'ipopt', nlp, opts);
    
    % 초기 추정값
    x0 = [TP_before; TS_before; TP_delta_before; TS_delta_before; 0; 0; 0];
    DelRate = 20*pi/180;
    % 경계 설정
    % lbx = [-20; -20; -30*pi/180; -30*pi/180;-inf;-inf;-inf];
    % ubx = [20; 20; 30*pi/180; 30*pi/180;inf;inf;inf];
    lbx = [TP_before - 0.5*20; TS_before - 0.5*20;TP_delta_before - 0.5*DelRate; TS_delta_before - 0.5*DelRate;-inf;-inf;-inf];
    ubx = [TP_before + 0.5*20; TS_before + 0.5*20;TP_delta_before + 0.5*DelRate; TS_delta_before + 0.5*DelRate;inf;inf;inf];   


    lbg = [-1e-5; -1e-5; -1e-5;-20; -20; -30*pi/180; -30*pi/180];
    ubg = [1e-5; 1e-5; 1e-5; 20; 20;30*pi/180; 30*pi/180];
    
    % 최적화 실행
    sol = solver('x0', x0, 'lbx', lbx, 'ubx', ubx, 'lbg', lbg, 'ubg', ubg);
    
    % 결과 출력
    x_opt = full(sol.x);


    TP = x_opt(1);
    TS = x_opt(2);
    TP_delta = x_opt(3);
    TS_delta = x_opt(4);

end
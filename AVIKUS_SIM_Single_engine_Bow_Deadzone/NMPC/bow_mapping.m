function new_bow_array = bow_mapping(bow_array,dt,Num, dwell_time,stop_dwell_time)

    bow_left_array = max(bow_array, 0);
    bow_right_array = -min(bow_array, 0);

        
    %% Cost
    total_variables = 2 * Num + 1;
    Q = zeros(total_variables);

    c = zeros(total_variables, 1); 
    c(1) = 1;

    %% 제약 변수 조건 설정 
    Aineq = [];
    bineq = [];

    % 실수 변수 제약 조건
    lb = zeros(total_variables, 1);
    ub = ones(total_variables, 1);

    % 이진 변수 설정 (0 또는 1)
    vartypes = repmat('B', 2 * Num, 1);
    
    % 실수 변수 설정
    vartypes = ['C';vartypes];  

    % 실수 변수 제약
    lb(1) = -50; 
    ub(1) = 50; 

    %% CIA 제약 조건
    % Mode switch 제약 조건
    for i = 1:Num-1
        Aineq = [Aineq; [zeros(1, 2*i-1), 1, 1, zeros(1, total_variables - 2*i - 1)]]; 
        bineq = [bineq; 2.0];  
    end
    Aineq = [Aineq; [zeros(1, 2*Num-1), 1, 1]];  
    bineq = [bineq; 2.0];  

    eps = 0.5;

    % Error sum
    for i = 1:Num-1
        bow_right_sum = sum(bow_right_array(1:i));
        bow_left_sum = sum(bow_left_array(1:i));

        % Left
        base_pattern_left = [1, 0];
        pattern_left = repmat(base_pattern_left, 1, i);
        
        Aineq = [Aineq; [-1, dt*pattern_left, zeros(1, total_variables - 2*i -1)]];
        bineq = [bineq; dt*(bow_left_sum) + eps];  
    
        Aineq = [Aineq; [-1, -dt*pattern_left, zeros(1, total_variables - 2*i -1)]]; 
        bineq = [bineq; -dt*(bow_left_sum) + eps];

        % Right
        base_pattern_right = [0, 1];
        pattern_right = repmat(base_pattern_right, 1, i);
        
        Aineq = [Aineq; [-1, dt*pattern_right, zeros(1, total_variables - 2*i -1)]];
        bineq = [bineq; dt*(bow_right_sum) + eps];
    
        Aineq = [Aineq; [-1, -dt*pattern_right, zeros(1, total_variables - 2*i -1)]];  
        bineq = [bineq; -dt*(bow_right_sum) + eps];
 
    end

    bow_right_sum = sum(bow_right_array);
    bow_left_sum = sum(bow_left_array);
    
    base_pattern_left = [1, 0];
    pattern_left = repmat(base_pattern_left, 1, Num);
    base_pattern_right = [0, 1];
    pattern_right = repmat(base_pattern_right, 1, Num);

    Aineq = [Aineq; [-1, -dt*pattern_left]];
    bineq = [bineq; -dt*bow_left_sum + eps];

    Aineq = [Aineq; [-1, dt*pattern_left]];
    bineq = [bineq; dt*bow_left_sum + eps];

    Aineq = [Aineq; [-1, dt*pattern_right]];
    bineq = [bineq; dt*bow_right_sum + eps];

    Aineq = [Aineq; [-1, -dt*pattern_right]];
    bineq = [bineq; -dt*bow_right_sum + eps];


    % % % Dwell constraints
    dwell_count = dwell_time/dt;

    % Turn on
    for i = 1:Num-2
        Aineq = [Aineq; [zeros(1, 2*i - 1), -1, 0, 1, 0, -1, 0, zeros(1, total_variables - (2*i - 1) - 6)]];
        bineq = [bineq; 0];
        Aineq = [Aineq; [zeros(1, 2*i - 1), 0, -1, 0, 1, 0, -1, zeros(1, total_variables - (2*i - 1) - 6)]];
        bineq = [bineq; 0];

        for j = 1:dwell_count-1
            if (2*i - 1 + 2*j + 6) < total_variables
                Aineq = [Aineq; [zeros(1, 2*i - 1), -1, 0, 1, 0, zeros(1, 2*j), -1, 0, zeros(1,total_variables - (2*i - 1) - 2*j - 6)]];
                bineq = [bineq; 0];
                Aineq = [Aineq; [zeros(1, 2*i - 1), 0, -1, 0, 1, zeros(1, 2*j), 0, -1, zeros(1,total_variables - (2*i - 1) - 2*j - 6)]];
                bineq = [bineq; 0];
            elseif (2*i - 1 + 2*j + 6) == total_variables
                Aineq = [Aineq; [zeros(1, 2*i - 1), -1, 0, 1, 0, zeros(1, 2*j), -1, 0]];
                bineq = [bineq; 0];            
                Aineq = [Aineq; [zeros(1, 2*i - 1), 0, -1, 0, 1, zeros(1, 2*j), 0, -1]];
                bineq = [bineq; 0];            
            end
        end
    end

    % % Turn off
    for i = 1:Num-2
        Aineq = [Aineq; [zeros(1, 2*i - 1), 1, 0, -1, 0, 1, 0, zeros(1, total_variables - (2*i - 1) - 6)]];
        bineq = [bineq; 1];
        Aineq = [Aineq; [zeros(1, 2*i - 1), 0, 1, 0, -1, 0, 1, zeros(1, total_variables - (2*i - 1) - 6)]];
        bineq = [bineq; 1];

        for j = 1:stop_dwell_time-1
            if (2*i - 1 + 2*j + 6) < total_variables
                Aineq = [Aineq; [zeros(1, 2*i - 1), 1, 0, -1, 0, zeros(1, 2*j), 1, 0, zeros(1,total_variables - (2*i - 1) - 2*j - 6)]];
                bineq = [bineq; 1];
                Aineq = [Aineq; [zeros(1, 2*i - 1), 0, 1, 0, -1, zeros(1, 2*j), 0, 1, zeros(1,total_variables - (2*i - 1) - 2*j - 6)]];
                bineq = [bineq; 1];
            elseif (2*i - 1 + 2*j + 6) == total_variables
                Aineq = [Aineq; [zeros(1, 2*i - 1), 1, 0, -1, 0, zeros(1, 2*j), 1, 0]];
                bineq = [bineq; 1];            
                Aineq = [Aineq; [zeros(1, 2*i - 1), 0, 1, 0, -1, zeros(1, 2*j), 1, 0]];
                bineq = [bineq; 1];            
            end
        end
    end


    %% MIQP 문제 정의
    model.Q = sparse(Q); 
    model.obj = c; 
    model.A = sparse(Aineq); 
    model.rhs = bineq; 
    model.lb = lb;  
    model.ub = ub;  
    model.vtype = vartypes; 
    model.sense = repmat('<', size(Aineq, 1), 1); 

    %% MIQP 풀기
    params.outputflag = 0; 
    result = gurobi(model, params);
    
    % result.x
    even_indices = 2:2:2*Num+1;
    odd_indices = 3:2:2*Num+1;
    new_left = result.x(even_indices);
    new_right = result.x(odd_indices);
    
    new_bow_array = new_left - new_right; 

end
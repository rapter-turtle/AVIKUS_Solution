function new_bow_array = one_check(bow_array,dt,Num, dwell_time)

    bow_left_array = bow_array;


        
    %% Cost
    total_variables = Num + 1;
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
    vartypes = repmat('B', Num, 1);
    
    % 실수 변수 설정
    vartypes = ['C';vartypes];  

    % 실수 변수 제약
    lb(1) = -50; 
    ub(1) = 50; 

    %% CIA 제약 조건
    % Mode switch 제약 조건
    for i = 1:Num-1
        Aineq = [Aineq; [zeros(1, i), 1, 1, zeros(1, total_variables - i - 2)]]; 
        bineq = [bineq; 2.0];  
    end
    Aineq = [Aineq; [zeros(1, Num-1), 1, 1]];  
    bineq = [bineq; 2.0];  

    eps = 0.5;

    % Error sum
    for i = 1:Num-1
        bow_left_sum = sum(bow_left_array(1:i));

        % Left
        
        Aineq = [Aineq; [-1, dt*ones(1, i), zeros(1, total_variables - i -1)]];
        bineq = [bineq; dt*bow_left_sum + eps];  
    
        Aineq = [Aineq; [-1, -dt*ones(1, i), zeros(1, total_variables - i -1)]]; 
        bineq = [bineq; -dt*bow_left_sum + eps];
 
    end

    bow_left_sum = sum(bow_left_array);
    
    Aineq = [Aineq; [-1, -dt*ones(1, Num)]];
    bineq = [bineq; -dt*bow_left_sum + eps];

    Aineq = [Aineq; [-1, dt*ones(1, Num)]];
    bineq = [bineq; dt*bow_left_sum + eps];

    % Dwell Time




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
    
    new_bow_array = result.x(2:Num+1);

end
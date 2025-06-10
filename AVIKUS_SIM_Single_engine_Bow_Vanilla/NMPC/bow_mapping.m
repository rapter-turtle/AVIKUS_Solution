function new_bow_array = bow_mapping(bow_array,dt,Num, dwell_time)

    bow_left_array = max(bow_array, 0);
    bow_right_array = -min(bow_array, 0);

        
    % QP 비용 함수 설정 (실수 변수 2개에 대해서만 이차 항목 적용)
    total_variables = 2 * Num + 1;  % 이진 변수 2*Num 개, 실수 변수 2개
    Q = zeros(total_variables);  % Q 행렬 초기화

    % 실수 변수에 대해서만 이차 항목을 적용 (실수 변수 2개에 대해서만 weight 부여)
    Q(1,1) = 0;  % 실수 변수에 대해 이차 항목을 설정
    
    % 선형 항목 (선형 항목이 없으므로 0으로 설정)
    c = zeros(total_variables, 1);  % 선형 항목 0으로 설정
    c(1) = 1;

    % 제약 조건 설정 (Aineq와 bineq는 예시로 작성된 제약)
    Aineq = [];
    bineq = [];

    % 실수 변수에 대한 제약 조건 (0 <= x <= 1)
    lb = zeros(total_variables, 1);  % 변수의 하한 설정
    ub = ones(total_variables, 1);   % 변수의 상한 설정

    % 이진 변수 설정 (0 또는 1)
    vartypes = repmat('B', 2 * Num, 1);  % 첫 번째부터 2*Num 번째까지 이진 변수
    
    % 실수 변수 설정 (선형 항목 없음)
    vartypes = ['C';vartypes];  


    % 실수 변수 2개에 대한 제약 (예시로 예측값 범위 설정)
    lb(1) = -50;  % 예시: 실수 변수의 하한
    ub(1) = 50;   % 예시: 실수 변수의 상한

    %% 제약 조건
    % Mode switch 제약 조건
    for i = 1:Num-1
        Aineq = [Aineq; [zeros(1, 2*i-1), 1, 1, zeros(1, total_variables - 2*i - 1)]]; 
        bineq = [bineq; 1.01];  
    end
    Aineq = [Aineq; [zeros(1, 2*Num-1), 1, 1]];  
    bineq = [bineq; 1.01];  

    eps = 0.0001;
    % Error sum
    for i = 1:Num-1
        % bow_right_sum = sum(bow_right_array(1:i));
        % bow_left_sum = sum(bow_left_array(1:i));
        bow_right_sum = sum(bow_right_array(1:i));
        bow_left_sum = sum(bow_left_array(1:i));

        % Left
        base_pattern_left = [1, 0];
        pattern_left = repmat(base_pattern_left, 1, i);
        
        Aineq = [Aineq; [-1, -dt*pattern_left, zeros(1, total_variables - 2*i -1)]];
        bineq = [bineq; -dt*(bow_left_sum) + eps];  
    
        Aineq = [Aineq; [-1, dt*pattern_left, zeros(1, total_variables - 2*i -1)]]; 
        bineq = [bineq; dt*(bow_left_sum) + eps];

        % Right
        base_pattern_right = [0, 1];
        pattern_right = repmat(base_pattern_right, 1, i);
        
        Aineq = [Aineq; [-1, -dt*pattern_right, zeros(1, total_variables - 2*i -1)]];
        bineq = [bineq; -dt*(bow_right_sum) + eps];
    
        Aineq = [Aineq; [-1, dt*pattern_right, zeros(1, total_variables - 2*i -1)]];  
        bineq = [bineq; dt*(bow_right_sum) + eps];
 
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

    % Aineq

    % MIQP 문제 정의
    model.Q = sparse(Q);  % 비용 함수의 이차 항목
    model.obj = c;  % 선형 항목 (여기서는 0으로 설정)
    model.A = sparse(Aineq);  % 부등식 제약 조건
    model.rhs = bineq;  % 제약 조건 우변
    model.lb = lb;  % 변수 하한
    model.ub = ub;  % 변수 상한
    model.vtype = vartypes;  % 변수 유형 (이진 변수)
    model.sense = repmat('<', size(Aineq, 1), 1); 

    % MIQP 풀기
    params.outputflag = 0;  % Gurobi의 출력을 활성화
    result = gurobi(model, params);
    
    even_indices = 2:2:2*Num+1;
    odd_indices = 3:2:2*Num+1;
    new_left = result.x(even_indices);
    new_right = result.x(odd_indices);
    
    % result.x
    % result.x(Num+2:2*Num+1)
    new_bow_array = new_left - new_right;  % 2번째부터 Num+1번째 값들만 추출


end
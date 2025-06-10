function [ref_x,ref_y,ref_psi,ref_u,ref_v,ref_r,ref_T, ref_del] = reference_trajectory(i, dt, control_update_dt, k, mode, start_time)    
    

    if mode <= 1
        
        % y 위치는 sin 곡선 (x에 비례)
        ref_y = 5;
   
    elseif mode == 2
        % a = 0.05;
        % vd = 0.5;
        % start_y = 1;
        % end_y = 14;
        % switch_time1 = vd/a;
        % switch_time2 = (end_y - start_y - vd*vd/a)/vd;
        % t2 = (i*dt + k*control_update_dt - start_time*dt);
        % % x 위치는 일정 속도로 직진 (부드러운 시작 적용)
        % 
        % if t2 <= switch_time1    
        % 
        %     ref_y = start_y + 0.5*a*t2*t2;
        % 
        % elseif (t2 > switch_time1) && (t2 <= switch_time1 + switch_time2)
        % 
        %     ref_y = start_y + 0.5*a*switch_time1*switch_time1 + vd*(t2 - switch_time1);
        % 
        % elseif (t2 > switch_time2) && (t2 <= 2*switch_time1 + switch_time2)
        % 
        %     ref_y = start_y + 0.5*a*switch_time1*switch_time1 + vd*switch_time2 + 0.5*a*(t2 - switch_time1 - switch_time2)*(t2 - switch_time1 - switch_time2);
        % 
        % else
        % 
        %     ref_y = end_y;
        % 
        % end
        ref_y = 14;
    end

    ref_y = 5;
    ref_x = 10;
    ref_u = 0;
    % y 방향 속도 (ref_v) 계산 (x에 대한 y 위치의 미분)
    ref_v = 0;

    % 선박의 방향(ref_psi)은 위치의 접선 방향 (atan2(dy/dt, dx/dt))
    ref_psi = 0;

    % yaw rate은 속도의 시간 변화에 따라 근사
    ref_r = 0; % 간단히 0으로 둬도 무방, 필요 시 미분 구현 가능

    % 기타 입력은 0으로 설정
    ref_T = 0;
    ref_del = 0;
end



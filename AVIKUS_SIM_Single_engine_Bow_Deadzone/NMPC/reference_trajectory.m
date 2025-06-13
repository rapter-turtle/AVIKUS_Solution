function [ref_x,ref_y,ref_psi,ref_u,ref_v,ref_r,ref_T, ref_del] = reference_trajectory(i, dt, control_update_dt, k, mode, start_time)    
    

   
    ref_y = 5;
    ref_x = 2;
    ref_u = 0;
    % y 방향 속도 (ref_v) 계산 (x에 대한 y 위치의 미분)
    ref_v = 0;

    % 선박의 방향(ref_psi)은 위치의 접선 방향 (atan2(dy/dt, dx/dt))
    ref_psi = 0.0*pi;
    %% 

    % yaw rate은 속도의 시간 변화에 따라 근사
    ref_r = 0; % 간단히 0으로 둬도 무방, 필요 시 미분 구현 가능

    % 기타 입력은 0으로 설정
    ref_T = 0;
    ref_del = 0;
end



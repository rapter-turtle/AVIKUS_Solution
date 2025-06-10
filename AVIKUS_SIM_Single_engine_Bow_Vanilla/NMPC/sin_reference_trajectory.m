function [ref_x,ref_y,ref_psi,ref_u,ref_v,ref_r,ref_TP,ref_TS,ref_delPR,ref_delSR] = sin_reference_trajectory(i, dt, control_update_dt, k)    
    
    a = 0.01;               % 가속도 관련 상수 (초기 천천히 시작)
    desired_speed = 1.0;    % 목표 속도 (m/s)
    amplitude = 3;          % sin 곡선 진폭 (m)
    wavelength = 50;        % sin 곡선 파장 (m)
    
    t = i*dt + k*control_update_dt;
    t_1 = i*dt + (k-1)*control_update_dt;

    % 부드러운 시작 속도 프로파일 (t=0에서 0, 점차 desired_speed로 증가)
    speed_profile = desired_speed * (1 - exp(-a*t)); 

    % x 위치는 일정 속도로 직진 (부드러운 시작 적용)
    ref_x = speed_profile *(1 - exp(-a*t))* t;
    ref_x_1 = speed_profile *(1 - exp(-a*t_1))* t_1;

    % y 위치는 sin 곡선 (x에 비례)
    ref_y = amplitude * sin(2*pi/wavelength * ref_x);

    % 속도 계산 (x, y 방향 속도)
    ref_u = speed_profile *(1 - exp(-a*t))* t - speed_profile *(1 - exp(-a*t_1))* t_1;

    % y 방향 속도 (ref_v) 계산 (x에 대한 y 위치의 미분)
    ref_v = amplitude * (2*pi/wavelength) * speed_profile * cos(2*pi/wavelength * ref_x) - amplitude * (2*pi/wavelength) * speed_profile * cos(2*pi/wavelength * ref_x_1);

    % 선박의 방향(ref_psi)은 위치의 접선 방향 (atan2(dy/dt, dx/dt))
    ref_psi = 0;

    % yaw rate은 속도의 시간 변화에 따라 근사
    ref_r = 0; % 간단히 0으로 둬도 무방, 필요 시 미분 구현 가능

    % 기타 입력은 0으로 설정
    ref_TP = 0;
    ref_TS = 0;
    ref_delPR = 0;
    ref_delSR = 0;
end

% 보조함수: ref_u 계산 (x방향 속도) - 부드럽게 시작하도록 설계
function ux = diff_with_smooth(t, amplitude, wavelength, desired_speed, a)
    speed_profile = desired_speed * (1 - exp(-a*t));
    % x = speed_profile * t 이므로 d/dt x = speed_profile + t * d/dt speed_profile
    dspeed_dt = a * desired_speed * exp(-a*t);
    ux = speed_profile + t * dspeed_dt;
end


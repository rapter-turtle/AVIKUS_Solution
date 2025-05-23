function [ref_x,ref_y,ref_psi,ref_u,ref_v,ref_r,ref_TP,ref_TS,ref_delPR,ref_delSR] = reference_trajectory(i, dt, control_update_dt, k)    
    
    a = 0.01;
    desired_speed = 1.0;
    desired_direction = 30*pi/180;

    t = (i*dt + k*control_update_dt);
    ref_u = 0;
    ref_v = 0;%desired_speed*sin(desired_direction);
    ref_r = 0;
    ref_x = desired_speed*cos(desired_direction)*(t + exp(-a*t)/a - 1/a);
    ref_y = desired_speed*sin(desired_direction)*(t + exp(-a*t)/a - 1/a);
    ref_psi = desired_direction;
    ref_TP = 0;
    ref_TS = 0;
    ref_delPR = 0;
    ref_delSR = 0;
end
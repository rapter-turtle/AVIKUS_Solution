function plot_ship_animation_update(i, ship_patch, path_line, ...
    h_thruster_L, h_thruster_R, q_thruster_L, q_thruster_R,...
    x_state, y_state, psi_state, u_state, v_state, r_state, ...
    Tau_TP, Tau_TS, Tau_delPR, Tau_delSR, ...
    Tau_TP_real, Tau_TS_real, Tau_delPR_real, Tau_delSR_real, ...
    subplot_axes, t)

    L = 9;
    B = 3;
    Lt = 1.5;
    Bt = 0.5;
    arrow_max_len = 10;

    DelRate = 100*3.141592/180; % deg/s
    DelMax = 30*3.141592/180;

    x = x_state(i);
    y = y_state(i);
    psi = psi_state(i);

    %% 회전 행렬
    R = [cos(psi), -sin(psi); sin(psi), cos(psi)];

    %% 선박 형상 업데이트
    shape = [ L/2, 0; L/3, -B/2; -L/2, -B/2; -L/2, B/2; L/3, B/2 ]';
    ship_shape = R * shape + [x; y];
    set(ship_patch, 'XData', ship_shape(1,:), 'YData', ship_shape(2,:));

    %% 경로 업데이트
    addpoints(path_line, x, y);

    %% Thruster 형상 및 위치
    offset_L = [-L/2.1; +B/4];
    offset_R = [-L/2.1; -B/4];
    pos_L = R * offset_L + [x; y];
    pos_R = R * offset_R + [x; y];

    thruster_shape = [ Lt/2, 0;
                       Lt/3, -Bt/2;
                      -Lt/3, -Bt/2;
                      -Lt/3,  Bt/2;
                       Lt/3,  Bt/2 ]';

    R_thruster_L = R * [cos(-Tau_delSR_real(i)), -sin(-Tau_delSR_real(i)); sin(-Tau_delSR_real(i)), cos(-Tau_delSR_real(i))];
    R_thruster_R = R * [cos(-Tau_delPR_real(i)), -sin(-Tau_delPR_real(i)); sin(-Tau_delPR_real(i)), cos(-Tau_delPR_real(i))];

    % R_thruster_L = R * [cos(Tau_delSR_real(i)), -sin(Tau_delSR_real(i)); sin(Tau_delSR_real(i)), cos(Tau_delSR_real(i))];
    % R_thruster_R = R * [cos(Tau_delPR_real(i)), -sin(Tau_delPR_real(i)); sin(Tau_delPR_real(i)), cos(Tau_delPR_real(i))];

    thruster_L_shape = R_thruster_L * thruster_shape + pos_L;
    thruster_R_shape = R_thruster_R * thruster_shape + pos_R;

    set(h_thruster_L, 'XData', thruster_L_shape(1,:), 'YData', thruster_L_shape(2,:));
    set(h_thruster_R, 'XData', thruster_R_shape(1,:), 'YData', thruster_R_shape(2,:));

    %% Thrust 방향 화살표
    dir_L = R_thruster_L * [1; 0];
    dir_R = R_thruster_R * [1; 0];
    len_L = Tau_TS_real(i) / 100 * arrow_max_len;
    len_R = Tau_TP_real(i) / 100 * arrow_max_len;

    set(q_thruster_L, 'XData', pos_L(1), 'YData', pos_L(2), ...
        'UData', len_L * dir_L(1), 'VData', len_L * dir_L(2));
    set(q_thruster_R, 'XData', pos_R(1), 'YData', pos_R(2), ...
        'UData', len_R * dir_R(1), 'VData', len_R * dir_R(2));


    %% === Subplot 9개 수동 업데이트 (최근 10초, 개별 ylim 설정) ===
    time_window = 10;  % seconds
    t_start = t(i) - time_window;
    idx_start = find(t >= t_start, 1, 'first');

    cla(subplot_axes(1));
    plot(subplot_axes(1), t(idx_start:i), u_state(idx_start:i), 'b'); % 첫 번째 그래프
    title(subplot_axes(1), 'u [m/s]'); ylabel(subplot_axes(1), 'u [m/s]'); xlabel(subplot_axes(1), 'Time [s]');
    %legend(subplot_axes(1), {'u\_state', 'v\_state'}, 'Location', 'best');
    ylim(subplot_axes(1), [-3 3]);xlim(subplot_axes(1), [t(i)-10, t(i)]);
    grid(subplot_axes(1), 'on');

    cla(subplot_axes(2));
    plot(subplot_axes(2), t(idx_start:i), v_state(idx_start:i), 'r');
    title(subplot_axes(2), 'v [m/s]'); ylabel(subplot_axes(2), 'v [m/s]'); xlabel(subplot_axes(2), 'Time [s]');
    ylim(subplot_axes(2), [-3 3]);xlim(subplot_axes(2), [t(i)-10, t(i)]);
    grid(subplot_axes(2), 'on');

    cla(subplot_axes(3));
    plot(subplot_axes(3), t(idx_start:i), r_state(idx_start:i), 'g');
    title(subplot_axes(3), 'r [rad/s]'); ylabel(subplot_axes(3), 'r [rad/s]'); xlabel(subplot_axes(3), 'Time [s]');
    ylim(subplot_axes(3), [-1 1]);xlim(subplot_axes(3), [t(i)-10, t(i)]);
    grid(subplot_axes(3), 'on');

    cla(subplot_axes(4));
    plot(subplot_axes(4), t(idx_start:i), u_state(idx_start:i), 'b');
    title(subplot_axes(4), 'u [m/s]'); ylabel(subplot_axes(4), 'u [m/s]'); xlabel(subplot_axes(4), 'Time [s]');
    ylim(subplot_axes(4), [-3 3]);xlim(subplot_axes(4), [t(i)-10, t(i)]);
    grid(subplot_axes(4), 'on');

    cla(subplot_axes(5));
    hold(subplot_axes(5), 'on');
    plot(subplot_axes(5), t(idx_start:i), Tau_delPR_real(idx_start:i), 'b'); % 첫 번째 그래프
    plot(subplot_axes(5), t(idx_start:i), Tau_delPR(idx_start:i), 'r'); % 두 번째 그래프
    hold(subplot_axes(5), 'off');
    title(subplot_axes(5), 'Left Thrust Angle'); ylabel(subplot_axes(5), 'delta [rad]'); xlabel(subplot_axes(5), 'Time [s]');
    legend(subplot_axes(5), {'real', 'commend'}, 'Location', 'best');
    ylim(subplot_axes(5), [-DelMax DelMax]);xlim(subplot_axes(5), [t(i)-10, t(i)]);
    grid(subplot_axes(5), 'on');

    cla(subplot_axes(6));
    hold(subplot_axes(6), 'on');
    plot(subplot_axes(6), t(idx_start:i), Tau_delSR_real(idx_start:i), 'b'); % 첫 번째 그래프
    plot(subplot_axes(6), t(idx_start:i), Tau_delSR(idx_start:i), 'r'); % 두 번째 그래프
    hold(subplot_axes(6), 'off');
    title(subplot_axes(6), 'Right Thrust Angle'); ylabel(subplot_axes(6), 'delta [rad]'); xlabel(subplot_axes(6), 'Time [s]');
    legend(subplot_axes(6), {'real', 'commend'}, 'Location', 'best');    
    ylim(subplot_axes(6), [-DelMax DelMax]);xlim(subplot_axes(6), [t(i)-10, t(i)]);
    grid(subplot_axes(6), 'on');

    cla(subplot_axes(7));
    plot(subplot_axes(7), t(idx_start:i), u_state(idx_start:i), 'm');
    title(subplot_axes(7), 'u [m/s]'); ylabel(subplot_axes(7), 'u [m/s]'); xlabel(subplot_axes(7), 'Time [s]');
    ylim(subplot_axes(7), [-3 3]);xlim(subplot_axes(7), [t(i)-10, t(i)]);
    grid(subplot_axes(7), 'on');

    cla(subplot_axes(8));
    hold(subplot_axes(8), 'on');
    plot(subplot_axes(8), t(idx_start:i), Tau_TP_real(idx_start:i), 'b'); % 첫 번째 그래프
    plot(subplot_axes(8), t(idx_start:i), Tau_TP(idx_start:i), 'r'); % 두 번째 그래프
    hold(subplot_axes(8), 'off');
    title(subplot_axes(8), 'Left Thrust Force'); ylabel(subplot_axes(8), 'Force [N]'); xlabel(subplot_axes(8), 'Time [s]');
    legend(subplot_axes(8), {'real', 'commend'}, 'Location', 'best');    
    ylim(subplot_axes(8), [-50 50]);xlim(subplot_axes(8), [t(i)-10, t(i)]);
    grid(subplot_axes(8), 'on');

    cla(subplot_axes(9));
    hold(subplot_axes(9), 'on');
    plot(subplot_axes(9), t(idx_start:i), Tau_TS_real(idx_start:i), 'b'); % 첫 번째 그래프
    plot(subplot_axes(9), t(idx_start:i), Tau_TS(idx_start:i), 'r'); % 두 번째 그래프
    hold(subplot_axes(9), 'off');
    title(subplot_axes(9), 'Right Thrust Force'); ylabel(subplot_axes(9), 'Force [N]'); xlabel(subplot_axes(9), 'Time [s]');
    legend(subplot_axes(9), {'real', 'commend'}, 'Location', 'best');    
    ylim(subplot_axes(9), [-50 50]);xlim(subplot_axes(9), [t(i)-10, t(i)]);
    grid(subplot_axes(9), 'on');

    drawnow limitrate;
end

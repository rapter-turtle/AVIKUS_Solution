function plot_ship_animation_update(i, ship_patch, path_line, ...
    h_thruster, q_thruster, bow_thruster, bow_thruster_arrow, pred_path_plot, reference_path_plot,...
    x_state, y_state, psi_state, u_state, v_state, r_state, ...
    Tau_T, Tau_del, ...
    Tau_T_real, Tau_del_real, Tau_bow,...
    subplot_axes, t, MPC_pred, MPC_ref, rps_real)

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
    offset = [-L/2.1; 0];
    pos = R * offset + [x; y];

    thruster_shape = [ Lt/2, 0;
                       Lt/3, -Bt/2;
                      -Lt/3, -Bt/2;
                      -Lt/3,  Bt/2;
                       Lt/3,  Bt/2 ]';

    R_thruster = R * [cos(-Tau_del_real(i)), -sin(-Tau_del_real(i)); sin(-Tau_del_real(i)), cos(-Tau_del_real(i))];

    thruster_shape = R_thruster * thruster_shape + pos;

    set(h_thruster, 'XData', thruster_shape(1,:), 'YData', thruster_shape(2,:));


    %% Bow Thruster 위치 및 도형
    bow_offset = [2;0];
    bow_pos = R * bow_offset + [x; y];
    bow_thruster_shape = [Lt/4, Lt; Lt/4, -Lt; -Lt/4, -Lt; -Lt/4, Lt]';
    bow_shape = R * bow_thruster_shape + bow_pos;
    
    set(bow_thruster, 'XData', bow_shape(1,:), 'YData', bow_shape(2,:));

    %% Bow Thrust 방향 화살표
    bow_R_thruster = R * [cos(0.5*pi), -sin(0.5*pi); sin(0.5*pi), cos(0.5*pi)];
    bow_dir = bow_R_thruster * [1; 0];
    len = Tau_bow(i) / 150 * arrow_max_len;

    set(bow_thruster_arrow, 'XData', bow_pos(1), 'YData', bow_pos(2), ...
        'UData', len * bow_dir(1), 'VData', len * bow_dir(2));

    %% Thrust 방향 화살표
    dir = R_thruster * [1; 0];
    len = Tau_T_real(i) / 100 * arrow_max_len;

    set(q_thruster, 'XData', pos(1), 'YData', pos(2), ...
        'UData', len * dir(1), 'VData', len * dir(2));

    % Prediction
    % MPC_pred(:,1)
    % MPC_pred(:,2)
    % MPC_ref(:,1)
    % MPC_ref(:,2)
    set(pred_path_plot, 'XData', MPC_pred(:,1), 'YData', MPC_pred(:,2));

    % Reference
    set(reference_path_plot, 'XData', MPC_ref(:,1), 'YData', MPC_ref(:,2));    


    %% === Subplot 9개 수동 업데이트 (최근 10초, 개별 ylim 설정) ===
    time_window = 70;  % seconds
    t_start = t(i) - time_window;
    idx_start = find(t >= t_start, 1, 'first');

    cla(subplot_axes(1));
    plot(subplot_axes(1), t(idx_start:i), u_state(idx_start:i), 'b'); % 첫 번째 그래프
    title(subplot_axes(1), 'u [m/s]'); ylabel(subplot_axes(1), 'u [m/s]'); xlabel(subplot_axes(1), 'Time [s]');
    %legend(subplot_axes(1), {'u\_state', 'v\_state'}, 'Location', 'best');
    ylim(subplot_axes(1), 'tight');xlim(subplot_axes(1), [t(i)-time_window, t(i)]);
    grid(subplot_axes(1), 'on');

    % cla(subplot_axes(2));
    % plot(subplot_axes(2), t(idx_start:i), v_state(idx_start:i), 'r');
    % title(subplot_axes(2), 'v [m/s]'); ylabel(subplot_axes(2), 'v [m/s]'); xlabel(subplot_axes(2), 'Time [s]');
    % ylim(subplot_axes(2), 'tight');xlim(subplot_axes(2), [t(i)-time_window, t(i)]);
    % grid(subplot_axes(2), 'on');
    % cla(subplot_axes(3));
    % plot(subplot_axes(3), t(idx_start:i), r_state(idx_start:i), 'g');
    % title(subplot_axes(3), 'r [rad/s]'); ylabel(subplot_axes(3), 'r [rad/s]'); xlabel(subplot_axes(3), 'Time [s]');
    % ylim(subplot_axes(3), 'tight');xlim(subplot_axes(3), [t(i)-time_window, t(i)]);
    % grid(subplot_axes(3), 'on');

    cla(subplot_axes(2));
    plot(subplot_axes(2), t(idx_start:i), r_state(idx_start:i), 'g');
    title(subplot_axes(2), 'r [rad/s]'); ylabel(subplot_axes(2), 'r [rad/s]'); xlabel(subplot_axes(2), 'Time [s]');
    ylim(subplot_axes(2), 'tight');xlim(subplot_axes(2), [t(i)-time_window, t(i)]);
    grid(subplot_axes(2), 'on');

    cla(subplot_axes(3));
    plot(subplot_axes(3), t(idx_start:i), Tau_bow(idx_start:i), 'r');
    title(subplot_axes(3), 'Bow thruster [N]'); ylabel(subplot_axes(3), 'Bow thruster [N]'); xlabel(subplot_axes(3), 'Time [s]');
    ylim(subplot_axes(3), 'tight');xlim(subplot_axes(3), [t(i)-time_window, t(i)]);
    grid(subplot_axes(3), 'on');

    cla(subplot_axes(4));
    plot(subplot_axes(4), t(idx_start:i), rps_real(idx_start:i), 'b');
    ylabel(subplot_axes(4), 'port [rps]'); xlabel(subplot_axes(4), 'Time [s]');
    ylim(subplot_axes(4), 'tight');xlim(subplot_axes(4), [t(i)-time_window, t(i)]);
    grid(subplot_axes(4), 'on');

    cla(subplot_axes(5));
    hold(subplot_axes(5), 'on');
    plot(subplot_axes(5), t(idx_start:i), Tau_del_real(idx_start:i), 'b'); % 첫 번째 그래프
    plot(subplot_axes(5), t(idx_start:i), Tau_del(idx_start:i), 'r'); % 두 번째 그래프
    hold(subplot_axes(5), 'off');
    title(subplot_axes(5), 'Left Thrust Angle'); ylabel(subplot_axes(5), 'delta [rad]'); xlabel(subplot_axes(5), 'Time [s]');
    legend(subplot_axes(5), {'real', 'commend'}, 'Location', 'best');
    ylim(subplot_axes(5), [-DelMax DelMax]);xlim(subplot_axes(5), [t(i)-time_window, t(i)]);
    grid(subplot_axes(5), 'on');

    cla(subplot_axes(6));
    hold(subplot_axes(6), 'on');
    plot(subplot_axes(6), t(idx_start:i), Tau_del_real(idx_start:i), 'b'); % 첫 번째 그래프
    plot(subplot_axes(6), t(idx_start:i), Tau_del(idx_start:i), 'r'); % 두 번째 그래프
    hold(subplot_axes(6), 'off');
    title(subplot_axes(6), 'Right Thrust Angle'); ylabel(subplot_axes(6), 'delta [rad]'); xlabel(subplot_axes(6), 'Time [s]');
    legend(subplot_axes(6), {'real', 'commend'}, 'Location', 'best');    
    ylim(subplot_axes(6), [-DelMax DelMax]);xlim(subplot_axes(6), [t(i)-time_window, t(i)]);
    grid(subplot_axes(6), 'on');

    cla(subplot_axes(7));
    plot(subplot_axes(7), t(idx_start:i), rps_real(idx_start:i), 'm');
    ylabel(subplot_axes(7), 'stb. [rps]'); xlabel(subplot_axes(7), 'Time [s]');
    ylim(subplot_axes(7), 'tight');xlim(subplot_axes(7), [t(i)-time_window, t(i)]);
    grid(subplot_axes(7), 'on');

    cla(subplot_axes(8));
    hold(subplot_axes(8), 'on');
    plot(subplot_axes(8), t(idx_start:i), Tau_T_real(idx_start:i), 'b'); % 첫 번째 그래프
    plot(subplot_axes(8), t(idx_start:i), Tau_T(idx_start:i), 'r'); % 두 번째 그래프
    hold(subplot_axes(8), 'off');
    title(subplot_axes(8), 'Left Thrust CMD'); ylabel(subplot_axes(8), '%'); xlabel(subplot_axes(8), 'Time [s]');
    legend(subplot_axes(8), {'real', 'commend'}, 'Location', 'best');    
    ylim(subplot_axes(8), 'tight');
    xlim(subplot_axes(8), [t(i)-time_window, t(i)]);
    grid(subplot_axes(8), 'on');

    cla(subplot_axes(9));
    hold(subplot_axes(9), 'on');
    plot(subplot_axes(9), t(idx_start:i), Tau_T_real(idx_start:i), 'b'); % 첫 번째 그래프
    plot(subplot_axes(9), t(idx_start:i), Tau_T(idx_start:i), 'r'); % 두 번째 그래프
    hold(subplot_axes(9), 'off');
    title(subplot_axes(9), 'Right Thrust CMD'); ylabel(subplot_axes(9), '%'); xlabel(subplot_axes(9), 'Time [s]');
    legend(subplot_axes(9), {'real', 'commend'}, 'Location', 'best');    
    ylim(subplot_axes(9), 'tight');
    xlim(subplot_axes(9), [t(i)-time_window, t(i)]);
    grid(subplot_axes(9), 'on');

    drawnow limitrate;
end

function [ship_patch, path_line, h_thruster_L, h_thruster_R, ...
          q_thruster_L, q_thruster_R, subplot_axes] = ...
          plot_ship_animation_init(x_init, y_init, psi_init)

    L = 9; B = 3; Lt = 1.5; Bt = 0.5;
    ship_color = [0 0.4470 0.7410];
    thruster_color = [0.8500 0.3250 0.0980];
    Dock_color = [0.2 0.2 0.2];
    arrow_color = 'r';

    %% 전체 figure
    f = figure('Position', [100 100 1600 800]);

    %% 왼쪽: 선박 애니메이션
    left_panel  = uipanel(f, 'Position', [0.00 0.05 0.5 0.9]);
    mainAx = axes(left_panel);
    axis(mainAx, 'equal');
    grid(mainAx, 'on');
    xlabel(mainAx, 'X [m]');
    ylabel(mainAx, 'Y [m]');
    title(mainAx, 'Ship Trajectory Animation');
    hold(mainAx, 'on');

    % DOCK
    Dock1 = [ 0, 0; 100, -2*100]';
    Dock1_patch = fill(mainAx, Dock1(1,:), Dock1(2,:), Dock_color, 'FaceAlpha', 0.7);
    Dock2 = [ 0,0; -100, -2*100]';
    Dock2_patch = fill(mainAx, Dock2(1,:), Dock2(2,:), Dock_color, 'FaceAlpha', 0.7);
    % Dock3 = [ -100, -5; -100, -100; 100, -100; 100, -5]';
    % Dock3_patch = fill(mainAx, Dock3(1,:), Dock3(2,:), Dock_color, 'FaceAlpha', 0.7);

    % 선박 본체
    shape = [ L/2, 0; L/3, -B/2; -L/2, -B/2; -L/2, B/2; L/3, B/2 ]';
    R = [cos(psi_init), -sin(psi_init); sin(psi_init), cos(psi_init)];
    ship_shape = R * shape + [x_init; y_init];
    ship_patch = fill(mainAx, ship_shape(1,:), ship_shape(2,:), ship_color, 'FaceAlpha', 0.7);
    path_line = animatedline(mainAx, 'LineStyle', '--', 'Color', 'k');

    % Thruster 위치 및 도형
    offset_L = [-L/2.1; +B/4];
    offset_R = [-L/2.1; -B/4];
    pos_L = R * offset_L + [x_init; y_init];
    pos_R = R * offset_R + [x_init; y_init];
    thruster_shape = [ Lt/2, 0; Lt/3, -Bt/2; -Lt/3, -Bt/2; -Lt/3, Bt/2; Lt/3, Bt/2 ]';
    shape_L = R * thruster_shape + pos_L;
    shape_R = R * thruster_shape + pos_R;
    h_thruster_L = fill(mainAx, shape_L(1,:), shape_L(2,:), thruster_color, 'FaceAlpha', 0.8);
    h_thruster_R = fill(mainAx, shape_R(1,:), shape_R(2,:), thruster_color, 'FaceAlpha', 0.8);

    % 초기 thrust 화살표 (길이 0)
    q_thruster_L = quiver(mainAx, pos_L(1), pos_L(2), 0, 0, 0, 'Color', arrow_color, 'LineWidth', 1.5);
    q_thruster_R = quiver(mainAx, pos_R(1), pos_R(2), 0, 0, 0, 'Color', arrow_color, 'LineWidth', 1.5);



    xlim(mainAx, [x_init-20, x_init+20]);
    ylim(mainAx, [y_init-10, y_init+20]);



    %% 오른쪽: 3x3 subplot
    right_panel = uipanel(f, 'Position', [0.5 0.05 0.5 0.9]);
    subplot_axes = gobjects(9, 1);

    % 레이아웃 설정
    rows = 3;
    cols = 3;
    margin_x = 0.07;  % x 방향 여백
    margin_y = 0.1;  % y 방향 여백
    ax_width  = (1 - margin_x*(cols+1)) / cols;
    ax_height = (1 - margin_y*(rows+1)) / rows;
    
    for row = 1:rows
        for col = 1:cols
            idx = (row - 1)*cols + col;
            pos_x = margin_x*col + ax_width*(col - 1);
            pos_y = 1 - (margin_y*row + ax_height*row);
            subplot_axes(idx) = axes('Parent', right_panel, ...
                                      'Position', [pos_x, pos_y, ax_width, ax_height]);
            %plot(subplot_axes(idx), [], []);
            if idx == 5 || idx == 6 || idx == 8 || idx == 9  % 예: subplot 1과 3에 그래프 2개 그리기
                hold(subplot_axes(idx), 'on');
                plot(subplot_axes(idx), 1:10, rand(1,10));  % 첫 그래프
                plot(subplot_axes(idx), 1:10, rand(1,10)*2); % 두 번째 그래프
                hold(subplot_axes(idx), 'off');
            else  % 나머지는 그래프 1개만 그림
                plot(subplot_axes(idx), 1:10, rand(1,10));
            end
            title(subplot_axes(idx), sprintf('subplot %d', idx));
        end
    end

    drawnow;
end

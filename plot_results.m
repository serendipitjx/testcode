function plot_results(q_history, vi_history, phidot_history, vc_history, t_history,path)
   params = config();
    figure('Position', [100, 100, 1200, 800]);
  path_x=path(1,:);
  path_y=path(2,:);
    % Fig5：运动轨迹
    subplot(2,2,1);
    plot(path_x, path_y, 'r-', 'LineWidth', 2, 'DisplayName', '期望路径');
    hold on; plot(q_history(:,1), q_history(:,2), 'bx', 'LineWidth', 1.5, 'DisplayName', '实际路径');
    xlabel('X (m)'); ylabel('Y (m)'); title('Fig5: 机器人运动轨迹');
    legend(); grid on; axis equal;

    % Fig6：车轮转向速度
    subplot(2,2,2);
    plot(t_history, phidot_history(:,1:4), 'LineWidth', 1);
    hold on; plot([0, max(t_history)], [params.phidotmax, params.phidotmax], 'b--', 'LineWidth', 2, 'DisplayName', '最大转向速度');
    xlabel('时间 (s)'); ylabel('转向速度 (rad/s)'); title('Fig6: 车轮转向速度');
    legend('车轮1','车轮2','车轮3','车轮4','最大转向速度'); grid on;

    % Fig7：车轮驱动速度
    subplot(2,2,3);
    plot(t_history, vi_history(:,1:4), 'LineWidth', 1);
    hold on; plot([0, max(t_history)], [params.vimax, params.vimax], 'b--', 'LineWidth', 2, 'DisplayName', '最大驱动速度');
    xlabel('时间 (s)'); ylabel('驱动速度 (m/s)'); title('Fig7: 车轮驱动速度');
    legend('车轮1','车轮2','车轮3','车轮4','最大驱动速度'); grid on;

    % Fig8：基座速度指令
    subplot(2,2,4);
    plot(t_history, vc_history, 'b-', 'LineWidth', 1.5);
    xlabel('时间 (s)'); ylabel('基座速度指令 (m/s)'); title('Fig8: 基座速度指令');
    grid on;
end
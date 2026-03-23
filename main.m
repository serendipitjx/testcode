function main()
    % ================= 清零 =================
    clear control
    clear static_counter
    cvx_clear
    clear all
    params = config();
    % ================= 获取初始轨迹 =================
    nu_ref = params.nu_ref;
    % ================= 初始化=================
    
    q = [0.0, 0.0];         % 初始位置(m)(1*2)
    psi0 = 0.2;                  % 初始朝向(rad)
    v_c = 0.141;               % 初始速度大小(m/s)
    vx = 0.1;
    vy = 0.1;
    omega_b = 0.01;
    last_vel = [vx ; vy ; omega_b];    %3*1
    state = [q , psi0];               %1*3
    % ================= 定义历史记录 =================
    q_history = zeros(params.num_steps, 2);
    vi_history = zeros(params.num_steps, 4);
    phidot_history = zeros(params.num_steps, 4);
    psi_history = zeros(params.num_steps, 1);
    t_history = zeros(params.num_steps, 1);
    phi_history = zeros(params.num_steps, 4);
    nu_history = zeros(params.num_steps, 2);
    figure('Position', [100, 100, 1500, 300]);
    
    subplot(1, 3, 1);
    plot(1, 1, 'O','Color',slanCL(1148,4),'LineWidth', 2, 'DisplayName', 'Reference Velocity'); hold on;
    h_q = animatedline('Marker', 'x', 'LineStyle', 'none', 'Color',slanCL(1148,2), 'LineWidth', 1.5, 'DisplayName', 'Simulation Results');
    xlabel('\nu_x (m)','FontName','Times New Roman');
    ylabel('\nu_y (m)','FontName','Times New Roman');
    title('Velocity Tracking Performance','FontName','Times New Roman');
    legend('FontSize', 6,'FontName','Times New Roman');
    grid on; set(gca,'FontName','Times New Roman');
    
     subplot(1, 3, 2); 
    % 绘制水平直线（y=1，x从1到20）
    % 修正：x轴正确生成（0.1到2，步长0.1），y轴为全1数组
    x = 0:0.01:0.2;        
    y = ones(size(x));     
    plot(x, y, '-', 'Color', slanCL(1148,4), 'LineWidth', 2, 'DisplayName', 'Reference Velocity');
    hold on;
    h_psi = animatedline('Marker', 'x', 'LineStyle', 'none', 'Color',slanCL(1148,2), 'LineWidth', 1.5, 'DisplayName', 'Simulation Results');
    xlabel('t(s)','FontName','Times New Roman');
    ylabel('\omega (m/s)','FontName','Times New Roman');
    title('Velocity Tracking Performance','FontName','Times New Roman');
    legend('FontSize', 6,'FontName','Times New Roman');
    grid on;  set(gca,'FontName','Times New Roman');

    subplot(1, 3, 3); hold on;
    yline(params.phidotmax, '--', 'Color', slanCL(1148,5), 'LineWidth', 2, 'DisplayName', 'Constraints');
    yline(-params.phidotmax, '--', 'Color', slanCL(1148,5), 'LineWidth', 2, 'HandleVisibility', 'off');
    h_phidot = gobjects(4,1);
    for idx = 1:4
        h_phidot(idx) = animatedline('Color', slanCL(1148,idx), 'LineWidth', 1.7, 'DisplayName', ['Wheel ' num2str(idx)]);
    end
    xlabel('Time(s)','FontName','Times New Roman'); 
    ylabel('Steering rate (rad/s)','FontName','Times New Roman'); 
    title('Steering Rate of Each Wheel','FontName','Times New Roman');
    legend('FontSize', 6,'FontName','Times New Roman'); 
    grid on;
    set(gca,'FontName','Times New Roman');
    
   
    

    
    prev_phi = zeros(4, 1);
    
 % ======================================= 仿真循环 =========================================
    for k = 1: 20
        t = (k - 1) * params.dt;
        t_history(k) = t;
        q_history(k, :) = q;
        psi_history(k) = psi0;
        
     %=================当与终点差较远距离时继续行驶================
        % if((path(1, end)-q(1))^2 + (path(2, end)-q(2))^2 > 0.01)    
        [new_state_dot, velocity] = control_RSS(nu_ref,path, k, last_vel, state);
        % end
        last_vel = velocity;
     %=======================得到反馈量======================
        
        vx = new_state_dot(1);
        vy = new_state_dot(2);
        omega_b= new_state_dot(3);
        phi = [0; 0; 0; 0];
        vi = [0; 0; 0; 0];
     %=====================计算轮子角速度========================、
        for i = 1:4
            Hj = [1, 0, -params.wheel_pos(i,2);0, 1, params.wheel_pos(i,1)];
            zn = Hj * velocity;
            vxi = zn(1);
            vyi = zn(2);
            vi(i) = sqrt(vxi^2 + vyi^2);
            phi(i) = atan2(vyi, vxi);
        end
     %==========================记录=============================
        vi_history(k,:) = vi;
        phi_history(k,:) = phi;
     %=========================更新状态========================
        psi0 = psi0 + omega_b * params.dt;
        q = q + [vx , vy]* params.dt;
        state_dot = new_state_dot;
        state = [q , psi0];  
        
        addpoints(h_q, velocity(1), velocity(2));
       
            addpoints(h_psi, t, velocity(3));
     
        if k > 1
            for idx = 1:4
                d_phi = phi(idx) - prev_phi(idx);
                d_phi = mod(d_phi + pi, 2*pi) - pi;
                curr_phidot = d_phi / params.dt;
                addpoints(h_phidot(idx), t, curr_phidot);
            end
        end
        drawnow limitrate;
        prev_phi = phi;
        
    end%仿真结束
    phidot_history = diff(phi_history);
    % 角度归一化：将phidot_history每个元素限制在(-π, π]区间
    for m = 1:params.num_steps
            phi_val = phidot_history(m);
            phi_val = mod(phi_val + pi, 2 * pi) - pi;
            phidot_history(m) = phi_val / params.dt;
    end
    
    % plot_results(q_history, vi_history, [[0, 0, 0, 0]; phidot_history], psi_history, t_history, path);
    
    global solver_time_array

    fprintf('总用时是：%f\n',sum(solver_time_array));
    
    save('solve_time_data.mat', 'solver_time_array'); 
end
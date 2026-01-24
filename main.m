function main()
    % ================= 清零 =================

    clear control
    clear static_counter
    cvx_clear
    clear all
    params = config();
    % ================= 获取初始轨迹 =================


    [path] = bezier_path(params.ctrl_pts, params.num_path_pts);


    % ================= 初始化=================
    
    q = [-0.050, -0.1];         % 初始位置(m)(1*2)
    psi0 = 0.5;                  % 初始朝向(rad)
    v_c = 0.141;               % 初始速度大小(m/s)
    vx = 0.01;
    vy = 0.01;
    omega_b = 0.01;
    state_dot = [vx ; vy ; omega_b];    %3*1
    state = [q , psi0];               %1*3
    % ================= 定义历史记录 =================

    q_history = zeros(params.num_steps, 2);
    vi_history = zeros(params.num_steps, 4);
    phidot_history = zeros(params.num_steps, 4);
    psi_history = zeros(params.num_steps, 1);
    t_history = zeros(params.num_steps, 1);
    phi_history = zeros(params.num_steps, 4);
    
    

 % ======================================= 仿真循环 =========================================
    for k = 1:params.num_steps

        t = (k - 1) * params.dt;
        t_history(k) = t;
        q_history(k, :) = q;
        psi_history(k) = psi0;
        
     %=================当与终点差较远距离时继续行驶================


        if((path(1, end)-q(1))^2 + (path(2, end)-q(2))^2 > 0.01)    
        [new_state_dot, velocity] = control_RSS(path, k, state_dot, state);
        end


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
        
    end%仿真结束

    phidot_history = diff(phi_history);
    % 角度归一化：将phidot_history每个元素限制在(-π, π]区间
    for m = 1:params.num_steps
            phi_val = phidot_history(m);
            phi_val = mod(phi_val + pi, 2 * pi) - pi;
            phidot_history(m) = phi_val / params.dt;
    end
    
    plot_results(q_history, vi_history, [[0, 0, 0, 0]; phidot_history], psi_history, t_history, path);
    
end


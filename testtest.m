% --- 参数初始化 ---
psi0 = 0;
K = 3;
dt = 0.05;
q = [0, 0];
current_nu = [0; 0; 0]; % 修正为列向量，习惯上方便计算
current_xy = [0; 0];
path = [0, 0, 0; 1, 2, 1; 2, 4, 2]; % 3x3 矩阵
rho = 1; % 给 rho 一个值，不能是 0 否则后面可能报错或无正则化效果

% 记录一下 u_prev 用于平滑约束
u_last_opt = zeros(3, K); 

fprintf('开始仿真 loop...\n');

for k = 1:3
    % 提取当前状态用于 CVX
    current_xy = q'; % 转为列向量 [x;y]
    
    cvx_begin quiet
        variable u(3, K) % 控制增量
        
        % 定义中间表达式
        expression nu(3, K+1)
        expression J
        
        % --- 关键修正 1: J 必须初始化为 0 ---
        J = 0; 
        
        nu(:, 1) = current_nu;
        
        % 动力学约束
        for t = 1:K
            nu(:, t+1) = nu(:, t) + u(:, t);
        end
        
        % 旋转矩阵 (注意：这里依然存在"僵尸朝向"问题，仅适用于小角度变化)
        R_psi0 = [cos(psi0), -sin(psi0); sin(psi0), cos(psi0)];
        S = [1, 0, 0; 0, 1, 0]; % 选择矩阵，只取 vx, vy
        C_1 = R_psi0 * S * dt; % [2x3] 矩阵
        
        % 代价函数计算
        for t = 1:K
            % 位置积分 (累加速度)
            expression NU_sum(3,1)
            NU_sum = sum(nu(:, 1:t), 2); % 使用 sum 函数更简洁
            
            % 预测位置 = 当前位置 + 旋转后的累积位移
            pred_pos = current_xy + C_1 * NU_sum;
            
            % 路径点索引 (防止越界)
            path_idx = min(size(path, 2), k + t); 
            ref_pos = path(1:2, path_idx);
            
            % 累加误差
            J = J + sum_square(pred_pos - ref_pos);
        end

        % 添加正则项 (平滑控制)
        J = J + 0.001 * sum_square(u(:));
        % J = J + rho * sum_square(u(:) - u_last_opt(:)); % 可选
        
        minimize(J)
        
        % --- 关键修正 2: 必须加约束防止飞车 ---
        subject to
            abs(u(1,:)) <= 5; 
            abs(u(2,:)) <= 5;
            abs(u(3,:)) <= 2;
    cvx_end
    
    % --- 检查 CVX 状态 ---
    if ~strcmp(cvx_status, 'Solved') && ~strcmp(cvx_status, 'Inaccurate/Solved')
        disp('CVX 未解出，跳过更新');
        u_opt = zeros(3,1);
    else
        u_opt = u(:, 1); % 取第一步控制量
        u_last_opt = u;  % 记录用于下一次热启动(如果有的话)
    end
    
    % --- 状态更新 (仿真器) ---
    % 1. 更新机体速度
    current_nu = current_nu + u_opt;
    
    % 2. 坐标变换: 机体速度 -> 全局速度
    % 修正变量名: state(3) -> psi0
    R_body_to_global = [cos(psi0), -sin(psi0), 0; 
                        sin(psi0),  cos(psi0), 0; 
                        0,          0,         1];
    
    global_vel = R_body_to_global * current_nu;
    
    vx = global_vel(1);
    vy = global_vel(2);
    omega_b = global_vel(3);
    
    % 3. 更新位置和角度 (欧拉积分)
    psi0 = psi0 + omega_b * dt;
    q = q + [vx, vy] * dt;
    
    % --- 打印结果 ---
    fprintf('Step %d: Pos=[%.2f, %.2f], Head=%.2f, Nu=[%.2f, %.2f]\n', ...
            k, q(1), q(2), psi0, current_nu(1), current_nu(2));
end
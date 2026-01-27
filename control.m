function [new_state_dot,velocity] = control(path, k , state_dot , state)   
    
    params = config();
    
    q = [state(1),state(2)];
    theta_b = state(3);
    
    %% 路径参数计算
    path_x = path(1, :);
    path_y = path(2, :);
    thetad = path(3, :);
    
    % 计算弧长参数
    dx = diff(path_x); dy = diff(path_y);
    s_list =  [0, cumsum(sqrt(dx.^2 + dy.^2))];
    Lp = s_list(end);
    
    % 目标点
    x = path_x(k);
    y = path_y(k);
    Pd = [x, y];
    
    % 导数计算 (注意：梯度法对噪声敏感，实际工程中建议离线平滑)
    dthetad_ds = (4*pi / Lp^2) * s_list(k);
    d2thetad_ds2 = 4*pi / Lp^2;
    
    dx_ds_vec = gradient(path_x, s_list);
    d2x_ds2_vec = gradient(dx_ds_vec, s_list);
    dy_ds_vec = gradient(path_y, s_list);
    d2y_ds2_vec = gradient(dy_ds_vec, s_list);
    
    dx_ds = dx_ds_vec(k);
    dy_ds = dy_ds_vec(k);
    d2x_ds2 = d2x_ds2_vec(k);
    d2y_ds2 = d2y_ds2_vec(k);
    theta_d = thetad(k);    
    
    % 切向角和曲率
    psit = atan2(dy_ds , dx_ds);
    denom = (dx_ds^2 + dy_ds^2)^(3/2) + 1e-6;
    Cc = (dx_ds * d2y_ds2 - dy_ds * d2x_ds2) / denom;
    
    % 误差计算 (全局转Frenet)
    URT = [cos(psit), sin(psit); sin(psit), -cos(psit)];
    error_xy = URT * (q - Pd)';
    xe = error_xy(1);
    ye = error_xy(2);
     
    % 当前状态
    vx = state_dot(1);
    vy = state_dot(2);
    v_c = sqrt(vx^2 + vy^2); % 仅供参考
    psiv = atan2(vy, vx);    % 仅供参考
    
    % 角度误差
    % 注意：new_psiv 计算逻辑似乎有特定设计，这里保持原样
    sigma_ye = -asin(params.k2 * ye / (abs(ye) + params.eps));
    new_psiv = psit - sigma_ye;
    
    psie = psit - new_psiv; % 这里原代码可能有笔误，通常是 psit - psiv
    thete = theta_d - theta_b;
    
    % 辅助变量计算
    eps_sigma = params.eps;           
    denom_sigma = abs(ye) + eps_sigma;
    u = params.k2 * ye ./ denom_sigma;
    du_dye = params.k2 * (denom_sigma - ye .* sign(ye)) ./ (denom_sigma.^2 + 1e-12);
    dsigma_dye = -du_dye ./ (sqrt(1 - u.^2) + 1e-12);
    
    k_s = params.k1 * xe + cos(sigma_ye);
    k_b = params.k3 * thete + dthetad_ds * k_s;
   
    % ================== 速度约束计算核心 ==================
    v_candidates = [];
    
    for i = 1:4
        % 【修正1】必须在循环内用 atan2 计算每个轮子的正确角度
        betai = atan2(params.wheel_pos(i,2), params.wheel_pos(i,1));
        li = norm(params.wheel_pos(i,:));
        
        alpha = new_psiv - theta_b;
        etai = alpha*(-1)^(i+1) - betai*(-1)^i;
        sin_etai = sin(etai);
        cos_etai = cos(etai);
        
        % 计算驱动速度约束
        term_sqrt = 1 + (k_b^2)*li^2 + 2*li*k_b*sin_etai;
        denom_a = sqrt(max(1e-6, term_sqrt)); % 保护：防止复数和除0
        v_candidate_a = params.vimax / denom_a;  
        v_candidates = [v_candidates; v_candidate_a]; 
        
        % 计算参数
        kx = k_s * (Cc * ye - 1) + cos(psie);
        ky = -(k_s * Cc * xe + sin(psie));
        ktheta = dthetad_ds * k_s - k_b;
       
        kv = (1 + dsigma_dye*xe)*Cc*k_s + dsigma_dye*sin(psie);
        kb_prime = params.k2*ktheta + d2thetad_ds2*k_s^2 + dthetad_ds*params.k1*kx - dthetad_ds*dsigma_dye*ky*sin(sigma_ye);
        
        numer_b = 1 + k_b^2*li^2 + 2*li*k_b*sin_etai;
        denom_b = kb_prime*li*cos_etai + (kv - k_b)*(1 + li*k_b*sin_etai);
        
        % 保护：防止除以极小值
        if abs(denom_b) < 1e-4
             denom_b = sign(denom_b) * 1e-4; 
             if denom_b == 0, denom_b = 1e-4; end
        end
        
        v_candidates = [v_candidates; abs(params.phidotmax * numer_b / denom_b)];
    end
    
    % ================== 最终速度决策逻辑 (修正版) ==================
    
    % 1. 清洗无效数据
    v_candidates = v_candidates(~isnan(v_candidates));
    v_candidates = v_candidates(~isinf(v_candidates));
    v_candidates = abs(v_candidates);
    
    % 2. 物理极限保底：如果所有计算都失效，至少不能超过最大设计速度
    v_candidates = [v_candidates; params.vimax];
    
    % 3. 得到当前的"物理限速"
    v_limit = min(v_candidates);
    
    % 4. 计算"为了追上点k"的需求速度
    dist_error = norm(path(1:2,k) - state(1:2));
    v_demand = dist_error / params.dt;
    
    % 5. 【核心修正】取需求和物理极限的较小值
    % 这样既试图追赶(v_demand)，又绝不超速(v_limit)
    new_v_c = min(v_demand, v_limit);
    
    % ================== 输出计算 ==================
    new_vx = new_v_c * cos(new_psiv);
    new_vy = new_v_c * sin(new_psiv);
    
    % 更新主体角速度 
    new_omega_b = k_b * new_v_c;
    new_state_dot = [new_vx; new_vy; new_omega_b];
    
    % 转换到车体坐标系 (Global -> Body)
    % 旋转矩阵转置 R'
    psi0 = theta_b;
    R_transpose = [cos(psi0), sin(psi0), 0; -sin(psi0), cos(psi0), 0; 0, 0, 1];
    velocity = R_transpose * new_state_dot;

end
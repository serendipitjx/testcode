function [new_state_dot] = control_easiest(path, step_idx, state_dot, current_state)
    params = config();
   
   
    
    k_pos = params.k1;    % 位置(x/y)比例增益
    k_theta = params.k3;  % 朝向(theta)比例增益
   

   
    current_x = current_state(1);    % 当前x坐标 (mm)
    current_y = current_state(2);    % 当前y坐标 (mm)
    current_theta = current_state(3);% 当前朝向 (rad)

  
    num_path_pts = size(path, 2);   
    target_idx = min(step_idx, num_path_pts);  
    target_x = path(1, target_idx);  % 目标点x坐标
    target_y = path(2, target_idx);  % 目标点y坐标
    target_theta = path(3, target_idx);  % 目标点朝向

 
    delta_x = target_x - current_x;          % x方向差值
    delta_y = target_y - current_y;          % y方向差值
    delta_theta = target_theta - current_theta;  % 朝向差值
    % 朝向差值归一化到[-π, π]，避免大角度绕圈
    delta_theta = mod(delta_theta + pi, 2*pi) - pi;

    
    x_dot = k_pos * delta_x;
    y_dot = k_pos * delta_y;
    theta_dot = k_theta * delta_theta;

  
    %{
 v_total = sqrt(x_dot^2 + y_dot^2);  % 计算合线速度
    if v_total > params.Vd_max
        % 按比例缩放x_dot/y_dot，保持方向不变
        scale_factor = params.Vd_max / v_total;
        x_dot = x_dot * scale_factor;
        y_dot = y_dot * scale_factor;
    end
   

    %  角速度约束：绝对值不超过dot_phi_max（0.4rad/s）
    theta_dot = max(min(theta_dot, params.dot_phi_max), -params.dot_phi_max);
    %}
  
    new_state_dot = [x_dot; y_dot; theta_dot];

end
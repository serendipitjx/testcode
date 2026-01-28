function [new_state_dot, velocity] = control_RSS(path, step, state_dot, state)
    
    params = config();
   
    % ================= Param Setup =================
    K = 6; rho = 0.01; k1 = 1; epsilon = 0;
    current_xy = [state(1), state(2)]';
    psi0 = state(3); current_nu = state_dot;
    R_psi0 = [cos(psi0), -sin(psi0); sin(psi0), cos(psi0)];

    % ================= H =================
    H = cell(1, 4);
    for n = 1:4
        H{n} = [1, 0, -params.wheel_pos(n, 2); 0, 1, params.wheel_pos(n, 1)];
    end

    % ================= 迭代 Setup =================
    max_iter = 3; u_hat = zeros(3,K);
    global solver_time_array;
    if ~exist('solver_time_array', 'var') || isempty(solver_time_array)
        solver_time_array = [];
    end

    % ================= 一层循环开始 =======================================
    for m = 1 : max_iter
        % ========== 1. 日志捕获（确保完整捕获Runtime行） ==========
        log_file = 'cvx_log_temp.txt';
        % 先删除旧日志（避免残留）
        if exist(log_file, 'file'), delete(log_file); end
        diary(log_file); diary on;

        % ========== 2. CVX求解（彻底清理高版本指令+修正psi声明） ==========
        cvx_begin % 无quiet、无cvx_options，纯低版本语法
            cvx_solver ECOS;
          
            variable u(3, K)
            variable nu(3, K)
            % 低版本CVX必须单独声明expression，彻底解决psi未识别
            expression NU(2, K);
            expression psi(K);  % 单独声明，语法正确
            expression J;       % 单独声明J
            
            % 定义状态序列
            NU(:, 1) = current_nu(1:2) * params.dt;
            psi(1) = psi0 + current_nu(3) * params.dt;

            for k = 1:K-1
                NU(:, k + 1) = NU(:, k) + nu(1:2, k) * params.dt;
                psi(k + 1) = psi(k) + nu(3, k) * params.dt;
            end
            
            % 代价函数
            J = 0;
            for k = 2:K
                J = J + 30 * sum_square(current_xy - path(1:2, min(size(path, 2), step + k) ) + R_psi0 * NU(:, k) );
            end
            for k = 1:K
                J = J + k1 * sum_square(psi(k) - path(3, min(size(path, 2), step + k) ) );
            end
            minimize(J + 0.3 * sum_square(u(:)) + rho * sum_square(u(:) - u_hat(:)));

            % 约束（完全保留你的逻辑）
            subject to
                % 动力学约束
                nu(:, 1) == current_nu + u(:, 1);
                for k = 1:K-1
                    nu(:, k + 1) == nu(:, k) + u(:, k + 1); 
                end
                % 凸约束
                for k = 1:K
                    for n = 1:4
                        norm(H{n} * nu(:, k), 2) <= params.vimax;
                    end
                end
                % 非凸约束
                delta_theta = params.dt * params.phidotmax;
                R = [sin(delta_theta), -cos(delta_theta); cos(delta_theta), sin(delta_theta)];
                nu_hat = zeros(3, K);
                nu_hat(:, 1) = current_nu + u_hat(:, 1);
                for k = 1:K-1
                    nu_hat(:, k + 1) = nu_hat(:, k)+ u_hat(:, k + 1);
                end
                for k = 1:K
                    for n = 1:4
                        expression LH;  % 单独声明LH
                        LH = 0;
                        if k > 1
                            for l = 1:k-1
                                LH = LH + 2 * ((eye(2) + R)* H{n} * nu_hat(:, k - 1) + R * H{n} * u_hat(:, k))' * ( eye(2) + R ) * H{n} * (u(:, l) - u_hat(:, l) );
                            end
                            LH = LH + 2 * ((eye(2) + R)* H{n} * nu_hat(:, k - 1) + R * H{n} * u_hat(:, k))' * R * H{n} * (u(:, k) - u_hat(:, k) );
                        end
                        if k > 1
                            sum_square(H{n} * nu(:, k - 1)) + sum_square( H{n} * (nu(:, k - 1) + u(:, k)) )...
                                - sum_square( (eye(2) + R) * H{n} * nu_hat(:, k-1) + R * H{n} * u_hat(:, k) ) - LH <= 0;
                        end
                        if k == 1
                            current_nu' * H{n}' * R * H{n} * (current_nu + u(:, 1)) >= epsilon;
                        end
                    end
                end
                R = [sin(delta_theta), cos(delta_theta); -cos(delta_theta), sin(delta_theta)];
                for k = 1:K
                    for n = 1:4
                        expression LH;  % 单独声明LH
                        LH = 0;
                        if k > 1
                            for l = 1:k-1
                                LH = LH + 2 * ((eye(2) + R)* H{n} * nu_hat(:, k - 1) + R * H{n} * u_hat(:, k))' * ( eye(2) + R ) * H{n} * (u(:, l) - u_hat(:, l) );
                            end
                            LH = LH + 2 * ((eye(2) + R)* H{n} * nu_hat(:, k - 1) + R * H{n} * u_hat(:, k))' * R * H{n} * (u(:, k) - u_hat(:, k) );
                        end
                        if k > 1
                            sum_square(H{n} * nu(:, k - 1)) + sum_square( H{n} * (nu(:, k - 1) + u(:, k)) )...
                                - sum_square( (eye(2) + R) * H{n} * nu_hat(:, k-1) + R * H{n} * u_hat(:, k) ) - LH <= 0;
                        end
                        if k == 1
                            current_nu' * H{n}' * R * H{n} * (current_nu + u(:, 1)) >= epsilon;
                        end
                    end
                end
        cvx_end

        % ========== 3. 停止日志+完整读取 ==========
        diary off;
        % 强制刷新日志文件（避免读取不完整）
        pause(0.01);
        fid = fopen(log_file, 'r');
        time_str = '未匹配到';
        inner_solve_time = 0;

        if fid == -1
            fprintf('警告：日志文件打开失败\n');
        else
            % 读取完整日志（按行读，确保抓到Runtime行）
            cvx_log = fread(fid, '*char')';
            fclose(fid);
            delete(log_file); % 立即删除，避免残留

            % ========== 4. 核心修正：匹配你的ECOS日志格式（Runtime: x.xxxx seconds） ==========
            % 你的日志格式是：Runtime: 0.008767 seconds. 修正正则表达式！
            pattern = 'Runtime:\s*(\d+\.?\d*e?[-+]?\d*)\s*seconds';
            time_match = regexp(cvx_log, pattern, 'tokens');

            % 提取有效时间（处理所有情况）
            if ~isempty(time_match)
                % 过滤空匹配，找到第一个有效值
                valid_matches = cellfun(@(x) ~isempty(x), time_match);
                if any(valid_matches)
                    time_str = time_match{find(valid_matches, 1)}{1};
                    inner_solve_time = str2double(time_str);
                end
            end

            % 调试：打印匹配到的时间和日志中Runtime行
            runtime_line = regexp(cvx_log, 'Runtime:.*seconds', 'match');
            fprintf('===== 调试：日志中Runtime行 =====\n%s\n', runtime_line{1});
        end

        % ========== 5. 存入数组（保留你的索引逻辑） ==========
        solver_time_array(3*step + m - 3) = inner_solve_time;

        % 打印结果（清晰显示提取的时间）
        fprintf('第%d步第%d次迭代 - ECOS内部求解时间：%.6f秒（原始值：%s）\n', ...
            step, m, inner_solve_time, time_str);
        fprintf('最优代价: %f | 求解状态: %s\n', cvx_optval, cvx_status);

        % 迭代更新
        u_hat = u;
    end % max_iter

    % 输出状态导数
    new_state_dot =  [cos(state(3)), -sin(state(3)), 0;
                     sin(state(3)),  cos(state(3)), 0;
                         0,              0, 1] * (state_dot + 1.00 * u(:, 1));
    velocity = current_nu + u(:, 1);
end
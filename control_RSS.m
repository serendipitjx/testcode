function [new_state_dot, velocity] = control_RSS(path, step, state_dot, state)
    
    params = config();
   
    % ================= Param Setup =================
    
    global K
    K = 6;               % 预测时域 (Prediction Horizon)
    rho = 0.01;                 % 正则化权重 (Regularization weight)
    k1 = 1;

    
    
    % ================= 初态 =================

    current_xy = [state(1), state(2)]';
    psi0 = state(3);
    global current_nu
    current_nu = [cos(psi0), sin(psi0), 0;
                 -sin(psi0), cos(psi0), 0;
                          0,         0, 1] * state_dot;     % [vx; vy; omega] (Body Frame)

    R_psi0 = [cos(psi0), -sin(psi0);
              sin(psi0),  cos(psi0)];

    % ================= H =================
    H = cell(1, 4);
    for n = 1:4
        H{n} = [1, 0, -params.wheel_pos(n, 2);
                0, 1,  params.wheel_pos(n, 1)];
    end



    % ================= 迭代 Setup =================
    max_iter = 3; 
    u_hat = zeros(3,K); % 猜测解

    % ================= 一层循环开始 =======================================

    for m = 1 : max_iter
   
        cvx_begin
        
            cvx_solver ECOS;
            % cvx_solver SCS;
            % cvx_solver SDPT3;
          
            variable u(3, K)
            variable nu(3, K)
            expression NU(2, K)
            expression psi(K)
            
            % 定义状态序列 nu (机体速度)

            % nu(:, 1) = current_nu + u(:, 1);
            NU(:, 1) = current_nu(1:2) * params.dt;
            psi(1) = psi0 + current_nu(3) * params.dt;

            for k = 1:K-1
                % nu(:, k + 1) = nu(:, k) + u(:, k + 1);
                NU(:, k + 1) = NU(:, k) + nu(1:2, k) * params.dt;
                psi(k + 1) = psi(k) + nu(3, k) * params.dt;
            end
            
 
            % ================= 代价 =================

            expression J
            J = 0;

            for k = 2:K
               J = J + 30 * sum_square(current_xy - path(1:2, min(size(path, 2), min(params.num_steps, step + k) )) + R_psi0 * NU(:, k) + [0 0; 0 0] * state_dot(1:2) * (psi(k-1) - psi0));
            end

            for k = 2:K
                J = J + k1 * sum_square(psi(k) - path(3, min(size(path, 2), min(params.num_steps, step + k) )) );
            end
            
            minimize(J + 0.3 * sum_square(u(:)) + rho * sum_square(u(:) - u_hat(:)));
            % minimize(0);


    


            % ================= 约束 =================
            
            subject to



            % ================= 动力学约束 =================

            nu(:, 1) == current_nu + u(:, 1);
            % NU(:, 1) == current_nu(1:2) * params.dt;
            % psi(1) == psi0 + current_nu(3) * params.dt;

            for k = 1:K-1
                nu(:, k + 1) == nu(:, k) + u(:, k + 1); 
                % NU(:, k + 1) == NU(:, k) + nu(1:2, k) * params.dt;
                % psi(k + 1) == psi(k) + nu(3, k) * params.dt; 
            end


            % ================= 凸约束 =================

            for k = 1:K
                for n = 1:4
                    norm(H{n} * nu(:, k), 2) <= params.vimax;
                    % norm(nu(:, k), 2) <= params.vimax;
                end
            end


            % ================= 非凸约束 =================

            delta_theta = params.dt * params.phidotmax;

            R = [sin(delta_theta), -cos(delta_theta);
                 cos(delta_theta),  sin(delta_theta)];

            nu_hat = zeros(3, K);
            nu_hat(:, 1) = current_nu + u_hat(:, 1);
            for k = 1:K-1
                nu_hat(:, k + 1) = nu_hat(:, k)+ u_hat(:, k + 1);
            end

            for k = 1:K
                for n = 1:4
                    
                    expression LH;
                    LH = 0;

                    if k > 1
                        for l = 1:k-1
                            LH = LH + 2 * ((eye(2) + R)* H{n} * nu_hat(:, k - 1) + R * H{n} * u_hat(:, k))' * ( eye(2) + R ) * H{n} * (u(:, l) - u_hat(:, l) );
                        end

                        LH = LH + 2 * ((eye(2) + R)* H{n} * nu_hat(:, k - 1) + R * H{n} * u_hat(:, k))' * R * H{n} * (u(:, k) - u_hat(:, k) );
                    end


                    if k > 1
                        sum_square(H{n} * nu(:, k - 1)) + sum_square( H{n} * (nu(:, k - 1) + u(:, k)) )...
                            - sum_square( (eye(2) + R) * H{n} * nu_hat(:, k-1) + R * H{n} * u_hat(:, k) )...
                            - LH <= 0;

                    end


                    if k == 1
                        % current_nu' * H{n}' * R * H{n} * (current_nu + u(:, 1)) >= 0;

                        sum_square(H{n} * current_nu) + sum_square( H{n} * (current_nu + u(:, k)) )...
                            - sum_square( (eye(2) + R) * H{n} * current_nu + R * H{n} * u_hat(:, k) )...
                            - 2 * ((eye(2) + R)* H{n} * current_nu + R * H{n} * u_hat(:, k))' * R * H{n} * (u(:, k) - u_hat(:, k) )...
                            <= 0;
                    end


                end
            end


            R = [sin(delta_theta), cos(delta_theta);
                 -cos(delta_theta),  sin(delta_theta)];

            for k = 1:K
                for n = 1:4
                    
                    expression LH;
                    LH = 0;

                    if k > 1
                        for l = 1:k-1
                            LH = LH + 2 * ((eye(2) + R)* H{n} * nu_hat(:, k - 1) + R * H{n} * u_hat(:, k))' * ( eye(2) + R ) * H{n} * (u(:, l) - u_hat(:, l) );
                        end

                        LH = LH + 2 * ((eye(2) + R)* H{n} * nu_hat(:, k - 1) + R * H{n} * u_hat(:, k))' * R * H{n} * (u(:, k) - u_hat(:, k) );
                    end


                    if k > 1
                        sum_square(H{n} * nu(:, k - 1)) + sum_square( H{n} * (nu(:, k - 1) + u(:, k)) )...
                            - sum_square( (eye(2) + R) * H{n} * nu_hat(:, k-1) + R * H{n} * u_hat(:, k) )...
                            - LH <= 0;

                    end


                    if k == 1
                        % current_nu' * H{n}' * R * H{n} * (current_nu + u(:, 1)) >= 0;

                        sum_square(H{n} * current_nu) + sum_square( H{n} * (current_nu + u(:, k)) )...
                            - sum_square( (eye(2) + R) * H{n} * current_nu + R * H{n} * u_hat(:, k) )...
                            - 2 * ((eye(2) + R)* H{n} * current_nu + R * H{n} * u_hat(:, k))' * R * H{n} * (u(:, k) - u_hat(:, k) )...
                            <= 0;
                    end


                end
            end

        cvx_end




        % ================= 进程打印 =================

        fprintf('最优代价是: %f\n', cvx_optval);
        if strcmp(cvx_status, 'Solved') || strcmp(cvx_status, 'Failed')
            fprintf('正在执行，已经进行第%d步/%d\n', step , m );
        else
                % 如果求解失败，使用上一时刻的解或全零
                % disp(['Optimization failed at step ', num2str(k), ': ', cvx_status]);
        end
           
    
        % ================= 为下一个凸问题更新的部分 =================
        u_hat = u;


    end % max_iter


    % 输出下一步的状态导数 (即下一步的速度)
    % nu^{k+1} = nu^k + u^k
    new_state_dot = state_dot + 1.00 * [cos(state(3)), -sin(state(3)), 0;
                                         sin(state(3)),  cos(state(3)), 0;
                                                     0,              0, 1] * u(:, 1);

    velocity = current_nu + u(:, 1);

end

function [new_state_dot] = control_RSS(path, step, state_dot, state)
    
    params = config();
   
    % ================= Param Setup =================
    
    global K
    K = 10;               % 预测时域 (Prediction Horizon)
    rho = 0.01;                 % 正则化权重 (Regularization weight)
    k1 = 0.01;


    
    
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
    global H;
    H = cell(1, 4);
    for n = 1:4
        H{n} = [1, 0, -params.wheel_pos(n, 2);
                0, 1,  params.wheel_pos(n, 1)];
    end



    % ================= 迭代 Setup =================
    max_iter = 1; 
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

            for k = 1:K
               J = J + sum_square(current_xy - path(1:2, min(params.num_steps, step + k )) + R_psi0 * NU(:, k) + [0 1; -1 0] * current_nu(1:2) * (psi(k) - psi0));
            end

            for k = 1:K
                J = J + k1 * sum_square(psi(k) - path(3, min(params.num_steps, step + k )) );
            end
            
            minimize(J + 0.001 * sum_square(u(:)) + rho * sum_square(u(:) - u_hat(:)));
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


            % ================= 轮速度约束 =================

            for k = 1:K
                for n = 1:4
                    norm(H{n} * nu(:, k), 2) <= params.vimax;
                    % norm(nu(:, k), 2) <= params.vimax;
                    % Cons1(u_hat, u, nu, t, n) <= 0
                    % square(nu(3,1)) <= 9;
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
    new_state_dot = current_nu + 0.98 * [cos(state(3)), -sin(state(3)), 0;
                                         sin(state(3)),  cos(state(3)), 0;
                                                     0,              0, 1] * u(:, 1);
end



function out = Cons1(alpha, u, nu, l,ii)
    global R;
    global K;
    global xInit;
    global H;
    u_cumsum = [zeros(3, 1), cumsum(alpha(:, 1:K-1), 2)]; 
    xs= repmat(xInit, 1, K) + u_cumsum;
    LH = 0;
    for j = 1:l-1
        LH = LH +2 * ((eye(2)+R)* H{ii}* xs(:,j)+ R * H{ii} * alpha(:,j))' * ( eye(2) + R ) * H{ii}*(u(:,j)-alpha(:,j));
    end
    LH = LH + 2 * ((eye(2)+R) * H{ii} * xs(:,l)+R*H{ii}*alpha(:,l))'*(eye(2) + R)*H{ii}*(u(:,l)-alpha(:,l));
    
    out =  ( H{ii} * nu(:,l))'*(H{ii} * nu(:,l))  + ( H{ii} * (nu(:,l) + u(:,l)))'*(H{ii} * (nu(:,l) + u(:,l)) )...
           - ( ( eye(2) + R ) * H{ii} * xs(:,l) + R * H{ii} * alpha(:,l))'* (( eye(2) + R ) * H{ii} * xs(:,l) + R * H{ii} * alpha(:,l))...
          -  2*( ( eye(2) + R ) * H{ii} * xs(:,l) + R * H{ii} * alpha(:,l))'* R * H{ii} * ( u(:,l) - alpha(:,l))- LH;
end

%{
function out = Cons2(alpha, u, nu , i)
    global R;
    global K;
    global xInit;
    global H;
    xs = repmat(xInit, [1, K]) + alpha * (triu(ones(K)) - eye(K));

    LH = 0;
    for j = 1:i-1
        LH = LH + 2 * (u(:, j) - alpha(:, j))' * (xs(:, i) + R * alpha(:, i));
    end
    LH = LH - 2 * (u(:, i) - alpha(:, i))' * R * (xs(:, i) + R * alpha(:, i));

    out = u(:, i)' * R' * R * u(:, i) - (xs(:, i) + R * alpha(:, i))' * (xs(:, i) + R * alpha(:, i)) - LH;
end
%}

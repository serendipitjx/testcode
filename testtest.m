 
cvx_precision(1e-2); % CVX层面调小误差
cvx_solver SCS;
cvx_solver_settings( ...
    'maxiter', 1000, ...  % 迭代次数上限设80
    'tol', 1e-2 ...     % SDPT3的误差容忍度设1e-3
);
cvx_begin
cvx_solver SCS
    variable x(2)
    minimize(norm(x,2))
    subject to
        x(1) + x(2) >= 1
cvx_end
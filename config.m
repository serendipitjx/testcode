function params = config()
    % ====================== 机器人几何参数 ======================
    params.Lx = 0.655;          % 车身长度(m)
    params.Ly = 0.335;          % 车身宽度(m)
    params.a = params.Lx/2;   % 车轮x向偏移(m)
    params.b = params.Ly/2;   % 车轮y向偏移(m)
    params.wheel_pos = [params.a, params.b; -params.a, params.b; -params.a, -params.b; params.a, -params.b]; % 4车轮位置
    params.vimax = 6;     % 最大驱动速度(m/s)
    params.phidotmax = 0.05*pi;   % 最大转向速度(rad/s)

    % ====================== 控制器参数 ======================
    params.k1 = 0.1;
    params.k2 = 0.1;
    params.k3 = 0.1;
    params.eps = 0.001;
    % ====================== 仿真参数 ======================
    params.dt = 0.01;         % 时间步长(s)
    params.t_end = 0.6;       % 仿真时长(s)
    params.num_steps = round(params.t_end/params.dt);

    % ====================== 期望路径参数 ======================
    params.ctrl_pts = [0,0; 0.25,0.25; 0.75,-0.25; 1,0]; % 贝塞尔曲线控制点
    params.num_path_pts = params.num_steps; % 路径离散点数
    %===================================================================
    
    
    params.vm = 0.5;           % 最小基座速度（论文命题1要求v≥vm>0，避免速度为0导致稳定性问题）
end
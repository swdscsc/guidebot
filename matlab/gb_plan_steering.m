function [best_delta, eff_clear, ttc_min, horizon, path] = gb_plan_steering( ...
        x, y, psi, v, prev_delta, obs, drp, half_width, max_unknown_range, ...
        p, front_clear, t_now, wheelbase, robot_radius)
%GB_PLAN_STEERING  转向空间 DWA —— 专为"前向速度由人提供"设计的采样规划器。
%
%   为什么不能直接用机器人工具箱里的现成 planner？
%   因为这类平台的控制空间退化了一维：经典 DWA 在 (v, omega) 二维速度空间
%   采样，而这里 **v 是外部输入、不可命令**，可行集只剩 {delta}，
%   且预测时域内的可控程度强烈依赖 v（见 gb_achievable_lateral）。
%
%   问题形式化：
%       给定人的当前速度 v，选 delta，使未来 T(v) 秒内预测轨迹的
%       "最小通行净空"最大，同时贴合目标方向、足够平滑、不贴墙。
%
%   代价函数里最关键的三个教训（都是真踩过的坑）：
%     A) 净空必须写成**屏障函数**（40*max(0,-clear) + 8/(clear+0.25)），
%        不能写成线性罚项。线性罚项会让"穿过坑洞"只罚 1.5 分、而"提前绕开"
%        因端点偏移罚 3.7 分 —— 规划器做了正确的两害相权，却选了撞上去的路，
%        然后安全层刹车 → 人停下 → 转向权限归零 → **永久死锁**。
%        结论：凡是"会不会撞"这种二元约束，不许拿去和舒适性做 trade-off。
%        这一条改动把完成率从 40% 提到 87%，是全项目收益最大的一次修改。
%     B) 预测要按**绝对时间**外推（t_now + tau），不能按相对量。
%        否则"感知时刻"与"预测时刻"错位，运动障碍的预测全部失效
%        （v0.5 之前"运动预测"这条消融实验整整空转了好几轮，结论全是假的）。
%     C) 评估避障要**跳过自身站位**（self_ignore_dist）。
%        障碍已被 ROBOT_RADIUS 膨胀，停在离柱子 0.5m 处时首点净空天然为负
%        ⇒ 所有候选都是"必撞" ⇒ 安全层永久刹车 ⇒ 死锁。
%
%   输入 obs : Nx5 矩阵 [x, y, r, vx, vy]（已做感知信息隔离，绝不能喂真值）
%         drp : Mx3 矩阵 [x, y, r]（下视探头发现的负障碍）
%   wheelbase / robot_radius 从 p.const 传入，避免跨文件硬编码
%
%   输出 path: 最优候选的预测轨迹 (steps+1)x2

    v_pred = max(v, p.v_floor);

    % 速度自适应前瞻：始终保持大致恒定的"向前看距离"
    horizon = p.preview_dist / v_pred;
    if horizon < p.horizon_min, horizon = p.horizon_min; end
    if horizon > p.horizon_max, horizon = p.horizon_max; end

    % 瞄准点：走廊中心线上前方 preview_dist 处
    aim_x = x + p.preview_dist;
    aim_y = 0.0;

    dmax  = p.delta_max_deg * pi/180.0;
    n     = p.n_candidates;
    steps = fix(horizon / p.dt_sim);      % Python int() 向零截断，这里必须一致
    if steps < 3, steps = 3; end

    best_cost  = inf;
    best_delta = 0.0;
    eff_clear  = 1e9;
    ttc_min    = 1e9;
    path       = zeros(steps+1, 2);

    for i = 1:n
        if n == 1
            delta = 0.0;
        else
            delta = -dmax + 2.0*dmax*(i-1)/(n-1);
        end

        cx = x;  cy = y;  cpsi = psi;
        cClear  = 1e9;
        cTtc    = 1e9;
        wallPen = 0.0;
        curPath = zeros(steps+1, 2);
        curPath(1, :) = [cx, cy];

        for k = 1:steps
            cpsi = cpsi + v_pred * tan(delta) / wheelbase * p.dt_sim;
            cx   = cx   + v_pred * cos(cpsi)  * p.dt_sim;
            cy   = cy   + v_pred * sin(cpsi)  * p.dt_sim;
            curPath(k+1, :) = [cx, cy];
            tau = k * p.dt_sim;

            % 绝对时间外推（教训 B）
            if p.use_prediction > 0
                t_rel = t_now + min(tau, p.conflict_window);
            else
                t_rel = 0.0;
            end

            % 跳过自身站位这一小段（教训 C）
            if hypot(cx - x, cy - y) < p.self_ignore_dist
                continue;
            end

            cl = gb_clearance_at(cx, cy, obs, drp, t_rel, robot_radius);

            if tau <= p.conflict_window
                if cl < cClear
                    cClear = cl;
                    cTtc   = min(cTtc, tau);
                end
            else
                % 窗口外只作为软性偏好：避免 freezing robot problem
                cClear = min(cClear, max(cl, p.target_clear));
            end

            over = abs(cy) - (half_width - 0.45);
            if over > wallPen, wallPen = over; end

            if p.unknown_is_obstacle > 0 && hypot(cx - x, cy - y) > max_unknown_range
                cClear = min(cClear, 0.4);
            end
        end

        % 低速时预测轨迹很短，会"看不见正前方的墙" —— 用前向锥形测距兜底
        eClear = min(cClear, front_clear);

        % ---- 代价函数（教训 A：屏障，不是线性罚）----
        clearanceCost = p.w_clear * max(0.0, p.target_clear - eClear);
        collisionPen  = p.w_collision * max(0.0, -eClear);
        barrier       = p.w_barrier / (max(eClear, 0.01) + 0.25);
        clearanceCost = clearanceCost + collisionPen + barrier;

        endDist  = hypot(cx - aim_x, cy - aim_y);
        goalCost = p.w_goal * endDist;
        headCost = p.w_heading * abs(gb_wrap_angle(cpsi));
        smoCost  = p.w_smooth * abs(delta - prev_delta) / max(dmax, 1e-6);
        wallCost = p.w_wall * wallPen;

        total = clearanceCost + goalCost + headCost + smoCost + wallCost;
        if total < best_cost
            best_cost  = total;
            best_delta = delta;
            eff_clear  = eClear;
            ttc_min    = cTtc;
            path       = curPath;
        end
    end
end

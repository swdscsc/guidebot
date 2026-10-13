function results = run_mil(opts)
%RUN_MIL  模型在环（MIL）仿真：用 MATLAB 独立实现整条闭环，与 Python 侧对照。
%
%   与 verify_against_python.m 的区别（这两个是两层不同的验证，别混为一谈）：
%     SIL / 交叉验证  —— 同一套控制律两种语言实现，**逐帧 bit-level 一致**
%                        （已验证：2617 帧偏差 0）
%     MIL             —— 整条闭环（感知+规划+控制+人）在 MATLAB 里独立重跑，
%                        验证的是**统计层面的行为一致性**：完成率、碰撞率、
%                        刹停率、横向误差应落在同一量级。
%                        这里不要求逐帧一致 —— 因为两侧随机数发生器不同
%                        （Python random.gauss vs MATLAB randn），
%                        强行要求逐帧一致反而说明随机数被同步了，那就不是在验证。
%
%   用法：
%     run_mil()                                % 常规条件，15 个场景
%     run_mil(struct('stress', true))          % 压力条件
%     run_mil(struct('noHigh', true))          % 消融：关掉悬空感知层

    if nargin < 1, opts = struct(); end
    here = fileparts(mfilename('fullpath'));
    dataDir = fullfile(here, 'data');
    addpath(here);

    p = gb_params(fullfile(dataDir, 'params.json'));
    c = p.control;  pc = p.planner;  s = p.sensor;  h = p.human;
    WB  = p.const.WHEELBASE;
    RR  = p.const.ROBOT_RADIUS;

    if isfield(opts, 'stress') && opts.stress
        c.latency_s = 0.30;   s.dropout_prob = 0.06;
    end
    if isfield(opts, 'noHigh') && opts.noHigh,  s.enable_high = 0; end
    if isfield(opts, 'noProbe') && opts.noProbe, s.enable_probe = 0; end
    if isfield(opts, 'noSafety') && opts.noSafety
        c.enable_brake_rule = 0; c.enable_gate = 0;
    end
    if isfield(opts, 'noPred') && opts.noPred, pc.use_prediction = 0; end
    if ~isfield(opts, 'maxTime'), opts.maxTime = 75.0; end
    if ~isfield(opts, 'plot'), opts.plot = true; end

    worlds = jsondecode(fileread(fullfile(dataDir, 'scenarios.json')));
    names  = fieldnames(worlds);
    rng(2024, 'twister');            % MIL 可复现

    dt = c.dt;
    results = struct('scenario', {}, 'dist', {}, 'goal', {}, 'stuck', {}, ...
        'collisions', {}, 'dropFalls', {}, 'stops', {}, 'nearMiss', {}, ...
        'latRmse', {}, 'avgV', {}, 'traj', {}, 'world', {});

    fprintf('\n===== MIL 仿真 (%s) =====\n', datestr(now, 'HH:MM:SS'));
    for w = 1:numel(names)
        wname = names{w};
        wd = worlds.(wname);
        obs = gb_load_world_obstacles(wd);
        drp = gb_load_world_drops(wd);
        lethal = (obs(:,4) <= 1.70) & (obs(:,5) >= 0.0);   % HUMAN_HEIGHT = 1.70

        x = 1.0; y = 0.0; psi = 0.0; delta = 0.0; t = 0.0;
        H = humanInit(h);
        traveled = 0.0; latSq = []; vSum = 0.0; vCnt = 0;
        coll = 0; dfall = 0; stops = 0; near = 0;
        stopCd = 0.0; brakeHeld = 0.0; bestX = x; lastProg = 0.0;
        hitFlag = false(size(obs,1), 1); dropFlag = false(size(drp,1), 1);
        nearCd = -9.0 * ones(size(obs,1), 1);
        goal = false; stuck = false;
        latSteps = max(0, round(c.latency_s / dt));
        queue = {};
        traj = zeros(0, 5);

        while t < opts.maxTime
            t = t + dt;

            % --- 1) 感知 ---
            angles = gb_angles(s);
            low  = gb_cast_layer(x, y, psi, obs, s, s.h_low, t);
            if s.enable_high > 0
                high = gb_cast_layer(x, y, psi, obs, s, s.h_high, t);
            else
                high = s.max_range * ones(s.n_rays, 1);
            end
            [drops, nearest] = gb_probe_drops(x, y, psi, drp, s);
            seen = gb_perceived_obstacles(x, y, psi, obs, s, t);

            % 正前方锥形自由距离（给规划器做低速时的兜底视野）
            frontClear = s.max_range;
            for i = 1:numel(angles)
                if abs(angles(i)) <= 0.20
                    frontClear = min([frontClear, low(i), high(i)]);
                end
            end

            % --- 2) 规划 ---
            if pc.unknown_is_obstacle > 0
                mux = 1e9;
            else
                mux = s.max_range * 0.9;
            end
            [pDelta, minClear, ttcMin, horizon, path] = gb_plan_steering( ...
                x, y, psi, H.v, delta, seen, drops, wd.half_width, mux, ...
                pc, frontClear, t, WB, RR);

            % --- 3) 控制 + 安全监督（延迟用队列真实实现）---
            over = (s.enable_high > 0) && any(high < s.max_range & low > high + 0.25);
            [dOut, brake, cue, ~, ~] = gb_supervise(pDelta, minClear, H.v, ...
                delta, c, horizon, pc.delta_max_deg*pi/180.0, WB, ...
                pc.target_clear, over, ~isempty(nearest), frontClear, ttcMin);
            queue{end+1} = struct('delta', dOut, 'brake', brake, 'cue', cue); %#ok<SAGROW>
            if numel(queue) > latSteps + 1
                act = queue{1};  queue(1) = [];
            else
                act = queue{1};
            end

            % --- 4) 人机闭环 ---
            H = humanHear(H, act.cue, t, h);
            H = humanUpdate(H, t, dt, act.brake, h);

            % --- 5) 运动学积分（无动力轮：v 由人给，delta 由机器给）---
            prevDelta = delta;
            delta = act.delta;
            if abs(H.v) > 1e-4
                psi = psi + H.v * tan(delta) / WB * dt;
                nx = x + H.v * cos(psi) * dt;
                ny = y + H.v * sin(psi) * dt;
                traveled = traveled + hypot(nx - x, ny - y);
                x = nx;  y = ny;
                lim = wd.half_width - RR;
                if abs(y) > lim
                    y = sign(y) * lim;  psi = 0.0;
                end
            end
            latSq(end+1) = y*y; %#ok<AGROW>
            vSum = vSum + H.v;  vCnt = vCnt + 1;
            traj(end+1, :) = [t, x, y, H.v, act.brake]; %#ok<AGROW>

            if x > bestX + 0.05, bestX = x; lastProg = t; end

            % --- 6) 事件统计（真值必须用当前绝对时刻的位置）---
            if act.brake && abs(H.v) < 0.06
                brakeHeld = brakeHeld + dt;
                if brakeHeld > 5.0
                    H = humanBackoff(H, t);
                    brakeHeld = 0.0;
                end
            else
                brakeHeld = 0.0;
            end

            if act.cue == 4 && stopCd <= 0
                stops = stops + 1;  stopCd = 1.0;
            end
            stopCd = stopCd - dt;

            cx = obs(:,1) + obs(:,6)*t;
            cy = obs(:,2) + obs(:,7)*t;
            d = hypot(x - cx, y - cy) - obs(:,3) - RR;
            newHit = (d < 0) & ~hitFlag & lethal;
            if any(newHit)
                coll = coll + sum(newHit);
                hitFlag = hitFlag | newHit;
            end
            nm = (d >= 0) & (d < 0.20);
            nm = nm & ((t - nearCd) > 2.0);
            if any(nm), near = near + sum(nm); nearCd(nm) = t; end

            if ~isempty(drp)
                inDrop = (x - drp(:,1)).^2 + (y - drp(:,2)).^2 <= drp(:,3).^2;
                newDrop = inDrop & ~dropFlag;
                if any(newDrop), dfall = dfall + sum(newDrop); dropFlag = dropFlag | newDrop; end
            end

            if x >= wd.goal_x, goal = true; break; end
            if t - lastProg > 12.0, stuck = true; break; end
        end

        results(w) = struct('scenario', wname, ...
            'dist', max(traveled, x - 1.0), 'goal', goal, 'stuck', stuck, ...
            'collisions', coll, 'dropFalls', dfall, 'stops', stops, ...
            'nearMiss', near, 'latRmse', sqrt(mean(latSq)), ...
            'avgV', vSum / max(vCnt,1), 'traj', traj, 'world', wd); %#ok<AGROW>
        fprintf('  %-14s dist=%6.1fm goal=%d stuck=%d col=%d drop=%d stop=%d lat=%.0fcm\n', ...
            wname, results(w).dist, goal, stuck, coll, dfall, stops, 100*results(w).latRmse);
    end

    % ---- 汇总 ----
    km = sum([results.dist]) / 1000.0;
    nGoal = sum([results.goal]);
    nCol  = sum([results.collisions]);
    nDrop = sum([results.dropFalls]);
    nStop = sum([results.stops]);
    fprintf('\n  ---- MIL 汇总 ----\n');
    fprintf('  完成率        : %d/%d (%.1f%%)\n', nGoal, numel(results), 100*nGoal/numel(results));
    fprintf('  碰撞 / km     : %.2f\n', nCol/km);
    fprintf('  踩空 / km     : %.2f\n', nDrop/km);
    fprintf('  STOP / km     : %.0f\n', nStop/km);
    fprintf('  总里程        : %.0f m\n', km*1000);

    outDir = fullfile(here, '..', 'results');
    if ~exist(outDir, 'dir'), mkdir(outDir); end
    agg = struct('goal_rate', nGoal/numel(results), 'total_km', km, ...
        'collisions_per_km', nCol/km, 'drop_per_km', nDrop/km, ...
        'stops_per_km', nStop/km, 'n_goal', nGoal, 'n_ep', numel(results));
    fid = fopen(fullfile(outDir, 'mil_metrics.json'), 'w');
    fprintf(fid, '%s', jsonencode(agg));
    fclose(fid);

    if opts.plot
        milPlot(results, outDir);
    end
end

% -------------------------------------------------------------------------
function a = gb_angles(s)
    half = s.fov_deg * pi/180.0 / 2.0;
    a = -half + (2.0*half) * ((0:s.n_rays-1) ./ max(s.n_rays-1, 1));
end

function H = humanInit(h)
    H = struct('v', h.preferred_speed*0.5, 'target', h.preferred_speed, ...
               'pending', [], 'backoffUntil', []);
end

function H = humanHear(H, cue, t, h)
    % cue: 0 none 1 left 2 right 3 slow 4 stop 5 overhead 6 drop
    if h.obey_cues <= 0 || ~isempty(H.backoffUntil), return; end
    if cue == 4
        H.pending = [H.pending; t + h.reaction_s, 0.0];
    elseif cue == 3
        H.pending = [H.pending; t + h.reaction_s, h.preferred_speed*h.slow_factor];
    else
        if H.target < h.preferred_speed - 1e-6
            H.pending = [H.pending; t + h.reaction_s, h.preferred_speed];
        end
    end
end

function H = humanUpdate(H, t, dt, brake, h)
    if ~isempty(H.backoffUntil) && t >= H.backoffUntil
        H.backoffUntil = [];  H.target = h.preferred_speed;  H.pending = [];
    end
    if isempty(H.backoffUntil) && ~isempty(H.pending)
        ready = H.pending(H.pending(:,1) <= t, :);
        if ~isempty(ready), H.target = ready(end, 2); end
        H.pending = H.pending(t < H.pending(:,1) + 3.0, :);
    end
    tgt = H.target + (rand()*2 - 1) * h.speed_jitter;
    a = h.accel_max;  if tgt <= H.v, a = -h.decel_max; end
    dv = a * dt;
    if abs(tgt - H.v) < abs(dv), H.v = tgt; else, H.v = H.v + dv; end
    if brake && isempty(H.backoffUntil)
        H.v = max(0.0, H.v - 3.2*dt);
    end
    H.v = max(-0.45, min(2.0, H.v));
end

function H = humanBackoff(H, t)
    H.backoffUntil = t + 1.2;
    H.target = -0.30;
end

function milPlot(results, outDir)
    f = figure('Visible', 'off', 'Position', [50 50 1400 850]);
    n = min(numel(results), 6);
    for i = 1:n
        r = results(i);
        subplot(2, 3, i);
        hold on; grid on;
        % 障碍
        O = r.world.obstacles;
        for k = 1:numel(O)
            col = [0.70 0.70 0.75];
            if strcmp(O(k).kind, 'pedestrian'), col = [1.0 0.55 0.35]; end
            if strcmp(O(k).kind, 'overhead'),   col = [0.85 0.30 0.85]; end
            rectangle('Position', [O(k).x-O(k).r, O(k).y-O(k).r, 2*O(k).r, 2*O(k).r], ...
                'Curvature', [1 1], 'FaceColor', col, 'EdgeColor', 'none');
        end
        for k = 1:numel(r.world.drops)
            D = r.world.drops(k);
            rectangle('Position', [D.x-D.r, D.y-D.r, 2*D.r, 2*D.r], ...
                'Curvature', [1 1], 'FaceColor', 'none', 'EdgeColor', 'k', 'LineWidth', 1.6);
        end
        yline(r.world.half_width, 'k--'); yline(-r.world.half_width, 'k--');
        tr = r.traj;
        brk = tr(:,5) > 0;
        plot(tr(:,2), tr(:,3), 'Color', [0.15 0.45 0.85], 'LineWidth', 1.6);
        if any(brk)
            scatter(tr(brk,2), tr(brk,3), 26, 'r', 'filled');
        end
        plot(r.world.goal_x, 0, 'g*', 'MarkerSize', 12);
        title(sprintf('%s  d=%.1fm goal=%d col=%d', r.scenario, r.dist, r.goal, r.collisions), ...
            'Interpreter', 'none');
        xlabel('x [m]'); ylabel('y [m]'); axis equal;
        xlim([0 r.world.length]); ylim([-r.world.half_width-0.4 r.world.half_width+0.4]);
    end
    sgtitle('GuideBot MIL — MATLAB/Simulink 侧仿真轨迹（红点=主动刹车）', ...
        'FontSize', 13, 'FontWeight', 'bold');
    out = fullfile(outDir, 'mil_traj.png');
    print(f, out, '-dpng', '-r130');
    close(f);
    fprintf('  -> %s\n', out);
end

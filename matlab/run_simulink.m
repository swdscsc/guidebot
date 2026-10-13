function r = run_simulink(scenarioName)
%RUN_SIMULINK  构建并仿真 Simulink 模型，输出轨迹图与统计到 results/。
%
%   这是 MBD 链路的最后一环：同一个算法在 Simulink（含 Stateflow 安全状态机）
%   里跑出来的结果，与 Python / 纯 MATLAB 两种实现三方对照。
%
%   用法：  run_simulink('mixed_1')

    if nargin < 1 || isempty(scenarioName), scenarioName = 'mixed_1'; end
    here  = fileparts(mfilename('fullpath'));
    addpath(here);
    outDir = fullfile(fileparts(here), 'results');
    if ~exist(outDir, 'dir'), mkdir(outDir); end

    fprintf('[1/3] 构建模型 ...\n');
    mdl = build_guidebot_model(scenarioName);

    fprintf('[2/3] 仿真 ...\n');
    assignin('base', 'slxState', []);
    simOut = sim(mdl, 'StopTime', '60.0', 'SaveOutput', 'on');

    % 兼容不同版本的仿真输出格式
    if isprop(simOut, 'slxState') || isfield(simOut, 'slxState')
        S = simOut.slxState;
    else
        S = evalin('base', 'slxState');
    end
    if isstruct(S) && isfield(S, 'signals')
        X = S.signals.values;   % N x 8
        tt = S.time;
    else
        X = S; tt = (0:size(S,1)-1)' * 0.05;
    end
    X = squeeze(X);
    if size(X,1) < size(X,2), X = X'; end

    % ---------- 统计 ----------
    d   = sum(sqrt(sum(diff(X(:,1:2),1,1).^2, 2)));
    vAvg = mean(X(:,4));
    nStop = numel(find(diff([0; X(:,4)]) < 0 & X(:,4) < 0.06));
    r = struct('scenario', scenarioName, 'dist_m', d, 'avg_speed', vAvg, ...
               'n_steps', size(X,1), 'stops', nStop, ...
               'final_x', X(end,1), 'reached', X(end,1) >= 38.0);

    fprintf('[3/3] 出图 ...\n');
    worlds = jsondecode(fileread(fullfile(here, 'data', 'scenarios.json')));
    wd = worlds.(scenarioName);
    obs = gb_load_world_obstacles(wd);
    drp = gb_load_world_drops(wd);

    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1180 520]);
    subplot(1,2,1); hold on; grid on; box on;
    HW = wd.half_width;
    patch([0 40 40 0], [-HW -HW HW HW], [0.96 0.96 0.92], 'EdgeColor', 'none');
    th = 0:0.05:2*pi;
    for i = 1:size(obs,1)
        fill(obs(i,1)+obs(i,3)*cos(th), obs(i,2)+obs(i,3)*sin(th), ...
             [0.85 0.55 0.55], 'EdgeColor', [0.6 0.2 0.2]);
    end
    for i = 1:size(drp,1)
        fill(drp(i,1)+drp(i,3)*cos(th), drp(i,2)+drp(i,3)*sin(th), ...
             [0.55 0.70 0.90], 'EdgeColor', [0.15 0.35 0.65]);
    end
    plot(X(:,1), X(:,2), 'Color', [0.10 0.55 0.25], 'LineWidth', 2.0);
    plot(X(1,1), X(1,2), 'o', 'MarkerFaceColor', [0.1 0.6 0.3], 'MarkerEdgeColor', 'k');
    plot(X(end,1), X(end,2), 's', 'MarkerFaceColor', [0.85 0.2 0.2], 'MarkerEdgeColor', 'k');
    title(sprintf('Simulink MIL 轨迹 — %s', scenarioName), 'Interpreter', 'none');
    xlabel('x (m)'); ylabel('y (m)'); axis equal; xlim([0 40]); ylim([-HW-0.3 HW+0.3]);

    subplot(1,2,2); hold on; grid on; box on;
    yyaxis left;  plot(tt, X(:,4), 'Color', [0.15 0.35 0.75], 'LineWidth', 1.6);
    ylabel('v (m/s)');
    yyaxis right; plot(tt, X(:,5) * 180/pi, 'Color', [0.75 0.35 0.10], 'LineWidth', 1.4);
    ylabel('\delta (deg)');
    xlabel('t (s)');
    title('Walking speed vs steering command');

    print(fig, fullfile(outDir, 'simulink_traj.png'), '-dpng', '-r140');
    close(fig);

    fid = fopen(fullfile(outDir, 'simulink_metrics.json'), 'w');
    fprintf(fid, '%s', jsonencode(r));
    fclose(fid);

    fprintf('\n=== Simulink MIL (%s) ===\n', scenarioName);
    fprintf('  步数 %d | 里程 %.2f m | 平均速度 %.3f m/s | 刹停 %d 次 | 终点 x=%.2f | 到达=%d\n', ...
        r.n_steps, r.dist_m, r.avg_speed, r.stops, r.final_x, r.reached);
end

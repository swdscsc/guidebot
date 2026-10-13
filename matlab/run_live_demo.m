function report = run_live_demo(stage)
%RUN_LIVE_DEMO  在本机 MATLAB 里实跑 GuideBot 的 MBD 验证链路。
%
%   run_live_demo('core')      % SIL 交叉验证 + MIL 闭环 + 完整消融套件
%   run_live_demo('simulink')  % 自动建模 + Stateflow 仿真
%
%   每一步都用 try/catch 包住：任何一步挂掉不会带走后面步骤，
%   并且日记文件里会留下明确的失败原因（工程上的可诊断性）。
%   所有屏幕输出同步落盘到 results/live_run_<stage>.log。

    if nargin < 1 || isempty(stage), stage = 'core'; end
    here = fileparts(mfilename('fullpath'));
    addpath(here);
    outDir = fullfile(here, '..', 'results');
    if ~exist(outDir, 'dir'), mkdir(outDir); end

    logFile = fullfile(outDir, sprintf('live_run_%s.log', stage));
    if exist(logFile, 'file'), delete(logFile); end
    diary(logFile);

    t0 = tic;
    fprintf('\n');
    fprintf('==========================================================\n');
    fprintf('  GuideBot LIVE MATLAB RUN  —  stage: %s\n', stage);
    fprintf('  MATLAB %s | %s\n', version('-release'), datestr(now, 31));
    fprintf('  host: %s\n', computer('arch'));
    fprintf('==========================================================\n');

    report = struct('stage', stage, 'ok', struct());

    switch lower(stage)
        case 'core'
            %% ---- [1/3] SIL：同一套控制律，Python vs MATLAB 逐帧比对 ----
            fprintf('\n--- [1/3] SIL 交叉验证 (software-in-the-loop) ---\n');
            try
                verify_against_python();
                report.ok.sil = true;
                fprintf('[1/3] SIL: PASS\n');
            catch ME
                report.ok.sil = false;
                fprintf(2, '[1/3] SIL: FAIL -> %s\n', ME.message);
                fprintf(2, '%s\n', getReport(ME, 'extended'));
            end

            %% ---- [2/3] MIL full：MATLAB 独立闭环，出轨迹图 ----
            fprintf('\n--- [2/3] MIL 闭环仿真 (model-in-the-loop, full) ---\n');
            try
                r = run_mil(struct('plot', true, 'maxTime', 75.0));
                report.mil_full = struct('goal_rate', sum([r.goal])/numel(r), ...
                                         'collisions', sum([r.collisions]), ...
                                         'dropFalls', sum([r.dropFalls]), ...
                                         'dist', sum([r.dist]));
                report.ok.mil = true;
                fprintf('[2/3] MIL full: DONE\n');
            catch ME
                report.ok.mil = false;
                fprintf(2, '[2/3] MIL full: FAIL -> %s\n', ME.message);
            end

            %% ---- [3/3] MIL 消融套件 ----
            fprintf('\n--- [3/3] MIL 消融套件 (8 variants x 15 scenarios) ---\n');
            try
                run_mil_suite();
                report.ok.suite = true;
                fprintf('[3/3] ablation suite: DONE\n');
            catch ME
                report.ok.suite = false;
                fprintf(2, '[3/3] ablation suite: FAIL -> %s\n', ME.message);
            end

        case 'simulink'
            %% ---- Simulink + Stateflow 自动建模与仿真 ----
            fprintf('\n--- Simulink model build + simulation ---\n');
            scen = 'mixed_1';
            try
                fprintf('[1/2] 自动构建模型 ...\n');
                mdl = build_guidebot_model(scen);
                fprintf('      模型已构建: %s\n', mdl);
                open_system(mdl);
                fprintf('[2/2] 仿真 StopTime=60s ...\n');
                % 用 SimulationInput/SimulationOutput API：To Workspace 写的变量
                % 会被收进 SimulationOutput 里，不依赖 "仿真结束再写回 base workspace"
                % 这个隐式行为 —— 旧式 sim(mdl,'StopTime',...) 在 R2024a 已废弃，
                % 拿不到日志对象。
                in = Simulink.SimulationInput(mdl);
                in = in.setModelParameter('StopTime', '60.0');
                in = in.setModelParameter('SaveOutput', 'on');
                out = sim(in);
                names = out.who;
                fprintf('      SimulationOutput 变量: %s\n', strjoin(names, ', '));
                if any(strcmp(names, 'slxState'))
                    S = out.get('slxState');
                else
                    S = evalin('base', 'slxState');
                end
                if isstruct(S) && isfield(S, 'signals')
                    X = squeeze(S.signals.values); tt = S.time;
                else
                    X = S; tt = (0:size(S,1)-1)' * 0.05;
                end
                d = sum(sqrt(sum(diff(X(:,1:2),1,1).^2, 2)));
                % 读到终点的时间（模型不会因为到达而自动停，所以自己算）
                worlds = jsondecode(fileread(fullfile(here, 'data', 'scenarios.json')));
                goalX = worlds.(scen).goal_x;
                idx = find(X(:,1) >= goalX, 1, 'first');
                if isempty(idx)
                    tGoal = NaN;
                else
                    tGoal = (idx-1) * 0.05;
                end
                report.simulink = struct('steps', size(X,1), 'dist_m', d, ...
                                         'final_x', X(end,1), 'goal_x', goalX, ...
                                         'time_to_goal_s', tGoal, ...
                                         'avg_speed', mean(X(:,4)), ...
                                         'reached', ~isempty(idx));
                report.ok.simulink = true;
                fprintf('      Simulink OK: steps=%d dist=%.2fm avg_v=%.3f\n', ...
                    size(X,1), d, mean(X(:,4)));
                if isempty(idx), verdict = '未到达'; else, verdict = '到达'; end
                fprintf('      走到 goal_x=%.1f 用时 %.2f s（60s 仿真窗口内 %s）\n', ...
                    goalX, tGoal, verdict);
                printSimulinkPlot(X, outDir, scen, goalX, tGoal);
            catch ME
                report.ok.simulink = false;
                report.simulink_error = ME.message;
                fprintf(2, '      Simulink FAIL -> %s\n', ME.message);
                fprintf(2, '%s\n', getReport(ME, 'extended'));
            end

        otherwise
            error('unknown stage: %s', stage);
    end

    fprintf('\n==========================================================\n');
    fprintf('  stage=%s finished in %.1f s\n', stage, toc(t0));
    fn = fieldnames(report.ok);
    for i = 1:numel(fn)
        if report.ok.(fn{i}), verdict = 'PASS'; else, verdict = 'FAIL'; end
        fprintf('    %-10s : %s\n', fn{i}, verdict);
    end
    fprintf('==========================================================\n');

    fid = fopen(fullfile(outDir, sprintf('live_run_%s.json', stage)), 'w');
    fprintf(fid, '%s', jsonencode(report));
    fclose(fid);

    diary off
end

% -------------------------------------------------------------------------
function printSimulinkPlot(X, outDir, scen, goalX, tGoal)
    HW = 3.0;
    worlds = jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')), ...
        'data', 'scenarios.json')));
    try, HW = worlds.(scen).half_width; catch, end

    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1180 500]);
    subplot(1,2,1); hold on; grid on; box on;
    patch([0 42 42 0], [-HW -HW HW HW], [0.96 0.96 0.92], 'EdgeColor', 'none');
    xline(goalX, 'g-', 'LineWidth', 1.8);
    plot(X(:,1), X(:,2), 'Color', [0.10 0.55 0.25], 'LineWidth', 2.0);
    plot(X(1,1), X(1,2), 'o', 'MarkerFaceColor', [0.1 0.6 0.3], 'MarkerEdgeColor', 'k');
    plot(X(end,1), X(end,2), 's', 'MarkerFaceColor', [0.85 0.2 0.2], 'MarkerEdgeColor', 'k');
    if ~isnan(tGoal)
        k = find(X(:,1) >= goalX, 1, 'first');
        plot(X(k,1), X(k,2), 'p', 'MarkerSize', 14, 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k');
        text(X(k,1), X(k,2)+0.35, sprintf('goal @ %.1fs', tGoal), 'Color', [0 0.5 0], 'FontSize', 9);
    end
    title(sprintf('Simulink + Stateflow trajectory - %s', scen), 'Interpreter', 'none');
    xlabel('x (m)'); ylabel('y (m)'); axis equal;
    xlim([0 goalX+3]); ylim([-HW-0.4 HW+0.4]);

    subplot(1,2,2); hold on; grid on; box on;
    tt = (0:size(X,1)-1)' * 0.05;
    yyaxis left;  plot(tt, X(:,4), 'Color', [0.15 0.35 0.75], 'LineWidth', 1.6); ylabel('v (m/s)');
    yyaxis right; plot(tt, X(:,5)*180/pi, 'Color', [0.75 0.35 0.10], 'LineWidth', 1.4); ylabel('\delta (deg)');
    xlabel('t (s)'); title('speed vs steering command');

    out = fullfile(outDir, 'simulink_traj.png');
    print(fig, out, '-dpng', '-r140');
    close(fig);
    fprintf('  -> %s\n', out);
end

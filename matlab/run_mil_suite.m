function suite = run_mil_suite()
%RUN_MIL_SUITE  MATLAB 侧的完整消融评测套件（与 Python run_benchmark.py 对应）。
%
%   为什么要两边各跑一遍？
%     这不是重复劳动。同一套算法在两个独立实现里跑出同向的结论，
%     说明结论不是某个随机数序列的巧合 —— 这是"结果可信"的最低门槛。
%     （很多学生的项目跑一次就写进简历，换台机器复现不出来。）
%
%   变体：
%     full        完整栈
%     no_high     关掉悬空感知层（h_high 射线）
%     no_probe    关掉近距下视探头
%     no_safety   关掉制动不等式 + 可控性门限
%     no_pred     关掉运动障碍预测
%     stress      延迟 300ms + 丢帧 6%
%     stress_no_safety  压力 + 关掉安全层（关键对照：证明安全层在退化条件下才真正值钱）

    here = fileparts(mfilename('fullpath'));
    addpath(here);
    cfgs = {
        'full',               struct()
        'no_high',            struct('noHigh', true)
        'no_probe',           struct('noProbe', true)
        'no_safety',          struct('noSafety', true)
        'no_pred',            struct('noPred', true)
        'stress',             struct('stress', true)
        'stress_no_safety',   struct('stress', true, 'noSafety', true)
        'stress_no_pred',     struct('stress', true, 'noPred', true)
    };

    suite = struct('variant', {}, 'goal_rate', {}, 'collisions_per_km', {}, ...
                   'drop_per_km', {}, 'stops_per_km', {}, 'near_per_km', {}, ...
                   'total_km', {});

    fprintf('\n########## MATLAB MIL Ablation Suite ##########\n');
    for i = 1:size(cfgs, 1)
        name = cfgs{i, 1};
        o = cfgs{i, 2};
        o.plot = false;
        o.maxTime = 75.0;
        r = run_mil(o);
        km = sum([r.dist]) / 1000.0;
        suite(i) = struct( ...
            'variant', name, ...
            'goal_rate', sum([r.goal]) / numel(r), ...
            'collisions_per_km', sum([r.collisions]) / km, ...
            'drop_per_km', sum([r.dropFalls]) / km, ...
            'stops_per_km', sum([r.stops]) / km, ...
            'near_per_km', sum([r.nearMiss]) / km, ...
            'total_km', km); %#ok<AGROW>
        fprintf('  ==> %-18s goal=%.1f%%  col=%.2f/km  drop=%.2f/km  stop=%.0f/km\n\n', ...
            name, 100*suite(i).goal_rate, suite(i).collisions_per_km, ...
            suite(i).drop_per_km, suite(i).stops_per_km);
    end

    outDir = fullfile(here, '..', 'results');
    if ~exist(outDir, 'dir'), mkdir(outDir); end
    fid = fopen(fullfile(outDir, 'mil_suite.json'), 'w');
    fprintf(fid, '%s', jsonencode(suite));
    fclose(fid);

    % Markdown 表
    md = {'# MATLAB / Simulink MIL 评测结果', '', ...
          '由 `matlab/run_mil_suite.m` 生成，与 Python 侧 `run_benchmark.py` 互为交叉验证。', '', ...
          '| 变体 | 完成率 | 碰撞/km | 踩空/km | STOP/km | 险情/km |', ...
          '|---|---|---|---|---|---|'};
    for i = 1:numel(suite)
        md{end+1} = sprintf('| %s | %.1f%% | %.2f | %.2f | %.0f | %.2f |', ...
            suite(i).variant, 100*suite(i).goal_rate, ...
            suite(i).collisions_per_km, suite(i).drop_per_km, ...
            suite(i).stops_per_km, suite(i).near_per_km); %#ok<AGROW>
    end
    md{end+1} = '';
    md{end+1} = sprintf('_生成时间 %s，%d 个变体 x 15 个场景。_', datestr(now, 31), numel(suite));
    fid = fopen(fullfile(outDir, 'MIL_RESULTS.md'), 'w');
    fprintf(fid, '%s\n', strjoin(md, newline));
    fclose(fid);
    fprintf('  -> %s\n', fullfile(outDir, 'MIL_RESULTS.md'));
end

function diag_slx_trace()
%DIAG_SLX_TRACE  把 Simulink 模型内部信号引出来逐帧打印，定位"为什么走不动"。
%
%   模型能仿真 ≠ 模型是对的。这里不看外部轨迹，而是直接把
%   规划/安全判据/状态机决策的内部变量打出来，看是谁把人刹住了。
%   To Workspace 用 Array 格式：一列一个新端口，一帧一行。

    here = fileparts(mfilename('fullpath'));
    addpath(here);
    outDir = fullfile(here, '..', 'results');
    if ~exist(outDir, 'dir'), mkdir(outDir); end
    logFile = fullfile(outDir, 'diag_slx_trace.log');
    if exist(logFile, 'file'), delete(logFile); end
    diary(logFile);

    mdl = 'guidebot_mil';
    fprintf('===== build =====\n');
    build_guidebot_model('mixed_1');

    % ---- 挂 To Workspace：一个信号一个变量 ----
    probe = {
        'Perception_Planning',  1,  'planDelta'
        'Perception_Planning',  2,  'minClear'
        'Perception_Planning',  3,  'ttc'
        'Perception_Planning',  5,  'frontClear'
        'Perception_Planning',  6,  'brakeRule'
        'Perception_Planning',  7,  'overlap'
        'Perception_Planning',  8,  'gate'
        'Perception_Planning',  9,  'tight'
        'Perception_Planning', 10,  'overSig'
        'Perception_Planning', 11,  'dropSig'
        'SafetyFSM',            1,  'brake'
        'SafetyFSM',            2,  'cue'
    };

    fprintf('\n===== attach probes =====\n');
    for i = 1:size(probe, 1)
        blk = probe{i,1}; port = probe{i,2}; varName = probe{i,3};
        try
            ph = get_param([mdl '/' blk], 'PortHandles');
            src = ph.Outport(port);
            nm  = [mdl '/probe_' varName];
            if ~isempty(find_system(mdl, 'SearchDepth', 1, 'Name', ['probe_' varName]))
                delete_block(nm);
            end
            add_block('simulink/Sinks/To Workspace', nm, ...
                'VariableName', varName, 'SaveFormat', 'Array', ...
                'Position', [900 60+40*i 1040 90+40*i]);
            % 参数名随版本变化（旧版 LimitDataPoints / 新版 MaxDataPoints），都试一遍
            try, set_param(nm, 'LimitDataPoints', 'off'); catch, end
            try, set_param(nm, 'MaxDataPoints', 'inf');    catch, end
            ph2 = get_param(nm, 'PortHandles');
            add_line(mdl, src, ph2.Inport(1), 'autorouting', 'on');
            fprintf('  probe %-14s <- %s outport %d\n', varName, blk, port);
        catch ME
            fprintf('  [warn] probe %s failed: %s\n', varName, ME.message);
        end
    end

    fprintf('\n===== simulate =====\n');
    try
        in = Simulink.SimulationInput(mdl);
        in = in.setModelParameter('StopTime', '60.0');
        in = in.setModelParameter('SaveOutput', 'on');
        out = sim(in);
        fprintf('  output vars: %s\n', strjoin(out.who, ', '));
    catch ME
        fprintf(2, 'SIM FAIL: %s\n', getReport(ME, 'extended'));
        diary off; return;
    end

    g = @(n) pick(out, n);
    S = g('slxState');
    if ~isempty(S) && isstruct(S) && isfield(S, 'signals')
        X = squeeze(S.signals.values);
    else
        X = S;
    end
    if ~isempty(X) && size(X,1) < size(X,2), X = X'; end

    fprintf('\n===== trace (every 40 steps) =====\n');
    fprintf('%5s %7s %7s %8s %8s %6s %6s %6s %6s %6s %6s %6s %6s %7s\n', ...
        'step', 'x', 'y', 'v', 'delta', 'bRule', 'ovl', 'gate', 'tight', 'over', 'drop', 'brake', 'cue', 'minClr');

    nStep = size(X,1);
    for k = 1:40:nStep
        row = [ nan nan nan nan nan nan nan nan ];
        fields = {'planDelta','minClear','brakeRule','overlap','gate','tight','overSig','dropSig','brake','cue'};
        vals = nan(1, numel(fields));
        for f = 1:numel(fields)
            a = g(fields{f});
            if ~isempty(a)
                a = a(:);
                idx = min(k, numel(a));
                vals(f) = a(idx);
            end
        end
        fprintf('%5d %7.2f %7.2f %8.3f %8.3f %6.1f %6.1f %6.1f %6.1f %6.1f %6.1f %6.1f %6.1f %7.2f\n', ...
            k, X(k,1), X(k,2), X(k,4), X(k,5), ...
            vals(3), vals(4), vals(5), vals(6), vals(7), vals(8), vals(9), vals(10), vals(2));
    end

    fprintf('\n===== key statistics =====\n');
    for f = 1:1
        fields = {'planDelta','minClear','ttc','frontClear','brakeRule','overlap','gate','tight','overSig','dropSig','brake','cue'};
        for i = 1:numel(fields)
            a = g(fields{i});
            if ~isempty(a)
                a = a(:);
                fprintf('  %-12s n=%-6d min=%-9.3f max=%-9.3f mean=%-9.3f  (first=%g, last=%g)\n', ...
                    fields{i}, numel(a), min(a), max(a), mean(a), a(1), a(end));
            else
                fprintf('  %-12s <empty>\n', fields{i});
            end
        end
    end
    fprintf('  v: min=%.3f max=%.3f mean=%.3f  final x=%.2f\n', ...
        min(X(:,4)), max(X(:,4)), mean(X(:,4)), X(end,1));
    fprintf('  brake duty = %.1f%%\n', 100*mean(g('brake') > 0.5));

    diary off
end

function v = pick(out, name)
    v = [];
    try
        names = out.who;
        if any(strcmp(names, name))
            v = out.get(name);
        end
    catch
    end
    if isempty(v)
        try, v = evalin('base', name); catch, v = []; end
    end
end

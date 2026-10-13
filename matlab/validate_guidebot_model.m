function rep = validate_guidebot_model()
%VALIDATE_GUIDEBOT_MODEL  对已生成的 guidebot_mil.slx 做结构性验证。
%
%   为什么要单独做"结构验证"？
%   一个自动生成的模型最典型的失败不是算法错，而是**接线断了一半还看不出来**：
%   某个输入口悬空、某个信号宽度不匹配、Stateflow 状态机缺了返回边。
%   这些问题在模型小的时候能在编译期报出来，但一旦模型变大就变成
%   "跑出来结果不对但不知道哪里不对"。所以这里在仿真之前先做静态检查。
%
%   检查项：
%     1) 所有非 Source 块的输入端口都已连线
%     2) Stateflow Chart 的输入/输出数据与五态 + 转移齐全
%     3) 反馈环存在（Plant -> UnitDelay -> Demux -> 感知），即闭环而非开环
%
%   输出：results/simulink_model_report.md

    here = fileparts(mfilename('fullpath'));
    addpath(here);
    mdl = 'guidebot_mil';
    slx = fullfile(here, [mdl '.slx']);
    fprintf('cwd=%s\nslx=%s\nisfile=%d\n', pwd, slx, isfile(slx));
    assert(isfile(slx), '找不到 %s，请先运行 build_guidebot_model', slx);

    if bdIsLoaded(mdl), close_system(mdl, 0); end
    load_system(mdl);

    L = {};

    blocks = find_system(mdl, 'SearchDepth', 1, 'Type', 'block');
    names  = sort(cellfun(@(b) get_param(b, 'Name'), blocks, 'UniformOutput', false));
    L{end+1} = '# Simulink 模型结构验证报告';
    L{end+1} = '';
    L{end+1} = sprintf('- 模型文件：`%s`', slx);
    L{end+1} = sprintf('- 顶层块数：%d', numel(blocks) - 1);
    L{end+1} = sprintf('- 块清单：%s', strjoin(names, ' / '));
    L{end+1} = '';

    % ---- 1) 悬空输入端口 ----
    L{end+1} = '## 1. 输入端口连通性';
    L{end+1} = '';
    L{end+1} = '| 块 | 端口 | 已连线 |';
    L{end+1} = '|---|---|---|';
    bad = 0;
    for i = 1:numel(blocks)
        b = blocks{i};
        if strcmpi(get_param(b, 'BlockType'), 'ModelReference'), continue; end
        ph = get_param(b, 'PortHandles');
        for k = 1:numel(ph.Inport)
            lh = get_param(ph.Inport(k), 'Line');
            ok = (lh ~= -1);
            if ~ok, bad = bad + 1; end
            L{end+1} = sprintf('| %s | In%d | %s |', get_param(b, 'Name'), k, tf(ok));
        end
    end
    L{end+1} = '';
    L{end+1} = sprintf('悬空输入端口数：**%d**', bad);
    L{end+1} = '';

    % ---- 2) Stateflow Chart ----
    L{end+1} = '## 2. Stateflow 安全状态机';
    L{end+1} = '';
    rt = sfroot();
    m  = rt.find('-isa', 'Simulink.BlockDiagram', 'Name', mdl);
    ch = m.find('-isa', 'Stateflow.Chart');
    if isempty(ch)
        L{end+1} = '未找到 Stateflow Chart（构建时降级为纯 Simulink 逻辑）。';
    else
        c = ch(1);
        din  = c.find('-isa', 'Stateflow.Data', 'Scope', 'Input');
        dout = c.find('-isa', 'Stateflow.Data', 'Scope', 'Output');
        sts  = c.find('-isa', 'Stateflow.State');
        trs  = c.find('-isa', 'Stateflow.Transition');
        L{end+1} = sprintf('- 输入数据（%d）：%s', numel(din), ...
            strjoin(cellfun(@(d) d.Name, num2cell(din), 'UniformOutput', false), ', '));
        L{end+1} = sprintf('- 输出数据（%d）：%s', numel(dout), ...
            strjoin(cellfun(@(d) d.Name, num2cell(dout), 'UniformOutput', false), ', '));
        L{end+1} = sprintf('- 状态（%d）：%s', numel(sts), ...
            strjoin(cellfun(@(s) s.Name, num2cell(sts), 'UniformOutput', false), ', '));
        L{end+1} = sprintf('- 转移数：%d', numel(trs));
        L{end+1} = '';
        L{end+1} = '| 源 | 目标 | 条件 |';
        L{end+1} = '|---|---|---|';
        for i = 1:numel(trs)
            t = trs(i);
            src = tern(~isempty(t.Source),      @() t.Source.Name,      '(default)');
            dst = tern(~isempty(t.Destination), @() t.Destination.Name, '(junction)');
            L{end+1} = sprintf('| %s | %s | `%s` |', src, dst, strrep(t.LabelString, '|', '\|'));
        end
    end
    L{end+1} = '';

    % ---- 3) 闭环 ----
    L{end+1} = '## 3. 闭环检查';
    L{end+1} = '';
    ph = get_param([mdl '/Actuator_Plant'], 'PortHandles');
    lh = get_param(ph.Outport(1), 'Line');
    if lh ~= -1
        dst = get_param(lh, 'DstBlockHandle');
        nm  = cellfun(@(h) get_param(h, 'Name'), num2cell(dst), 'UniformOutput', false);
        L{end+1} = sprintf('- `Actuator_Plant` 输出 -> %s', strjoin(nm, ', '));
        L{end+1} = sprintf('- 反馈闭环：**%s**（Plant 输出必须回到 Unit Delay 才能构成递推）', ...
            tf(any(strcmp(nm, 'State'))));
    else
        L{end+1} = '- `Actuator_Plant` 输出未连线（开环）';
    end
    L{end+1} = '';
    L{end+1} = '---';
    L{end+1} = '';
    L{end+1} = '_由 `matlab/validate_guidebot_model.m` 自动生成。_';

    outDir = fullfile(fileparts(here), 'results');
    if ~exist(outDir, 'dir'), mkdir(outDir); end
    fid = fopen(fullfile(outDir, 'simulink_model_report.md'), 'w');
    for i = 1:numel(L), fprintf(fid, '%s\n', L{i}); end
    fclose(fid);

    close_system(mdl, 0);
    fprintf('报告已写出，悬空输入端口 %d 个\n', bad);
    rep = struct('blocks', {names}, 'dangling_inports', bad);
end

function s = tf(b)
    if b, s = 'YES'; else, s = '**NO**'; end
end

function v = tern(cond, f, other)
    if cond, v = f(); else, v = other; end
end

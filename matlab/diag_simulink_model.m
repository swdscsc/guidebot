function diag_simulink_model()
%DIAG_SIMULINK_MODEL  在真实 MATLAB 里诊断自动生成的 Simulink/Stateflow 模型。
%
%   为什么需要这个脚本？
%     build 阶段不报错 ≠ 能仿真。模型编译（update diagram）才会暴露
%     端口维度、Stateflow 图结构、数据类型这几类问题。
%     这个函数把编译期能拿到的全部信息打出来，用来定位而不是猜。

    here = fileparts(mfilename('fullpath'));
    addpath(here);
    outDir = fullfile(here, '..', 'results');
    if ~exist(outDir, 'dir'), mkdir(outDir); end
    logFile = fullfile(outDir, 'diag_simulink.log');
    if exist(logFile, 'file'), delete(logFile); end
    diary(logFile);

    mdl = 'guidebot_mil';
    fprintf('\n===== Step 1: build model =====\n');
    try
        build_guidebot_model('mixed_1');
    catch ME
        fprintf(2, 'build FAIL: %s\n', getReport(ME, 'extended'));
        diary off; return;
    end

    fprintf('\n===== Step 2: inspect Stateflow chart =====\n');
    try
        rt = sfroot();
        charts = rt.find('-isa', 'Stateflow.Chart');
        fprintf('charts found: %d\n', numel(charts));
        for i = 1:numel(charts)
            ch = charts(i);
            fprintf('\n-- Chart: %s (machine=%s)\n', ch.Name, ch.Machine.Name);
            fprintf('   Path               : %s\n', ch.Path);
            fprintf('   ActionLanguage     : %s\n', ch.ActionLanguage);
            fprintf('   StateMachineType   : %s\n', ch.StateMachineType);
            fprintf('   ChartUpdate        : %s\n', ch.ChartUpdate);
            fprintf('   EnableBitOp        : %s\n', mat2str(ch.EnableBitOps));

            sts = ch.find('-isa', 'Stateflow.State');
            fprintf('   states (%d):\n', numel(sts));
            for k = 1:numel(sts)
                fprintf('     [%d] name="%s"  decomposition=%s  type=%s  id=%d\n', ...
                    k, sts(k).Name, sts(k).Decomposition, sts(k).Type, sts(k).Id);
                fprintf('         label=%s\n', replace(sts(k).LabelString, newline, ' | '));
            end

            jts = ch.find('-isa', 'Stateflow.Junction');
            fprintf('   junctions (%d):\n', numel(jts));
            for k = 1:numel(jts)
                fprintf('     [%d] type=%s id=%d\n', k, jts(k).Type, jts(k).Id);
            end

            trs = ch.find('-isa', 'Stateflow.Transition');
            fprintf('   transitions (%d):\n', numel(trs));
            for k = 1:numel(trs)
                t = trs(k);
                srcN = portName(t.Source);
                dstN = portName(t.Destination);
                fprintf('     [%d] %-14s --> %-14s   label="%s"\n', k, srcN, dstN, t.LabelString);
            end

            dts = ch.find('-isa', 'Stateflow.Data');
            fprintf('   data (%d):\n', numel(dts));
            for k = 1:numel(dts)
                d = dts(k);
                sz = '?';
                try, sz = num2str(d.Props.Size); catch, end
                pr = '?';
                try, pr = num2str(d.Props.InitialValue); catch, end
                fprintf('     [%d] %-10s scope=%-12s port=%d size=%s init=%s type=%s\n', ...
                    k, d.Name, d.Scope, d.Port, sz, pr, d.DataType);
            end
        end
    catch ME
        fprintf(2, 'inspect FAIL: %s\n', getReport(ME, 'extended'));
    end

    fprintf('\n===== Step 3: try update diagram (compile) =====\n');
    try
        set_param(mdl, 'SimulationCommand', 'Update');
        fprintf('UPDATE: OK\n');
        % 编译后端口维度
        blks = {'Perception_Planning', 'SafetyFSM', 'Actuator_Plant', 'State', 'Split'};
        for i = 1:numel(blks)
            for p = {'Inport', 'Outport'}
                try
                    ph = get_param([mdl '/' blks{i}], 'PortHandles');
                    arr = ph.(p{1});
                    for k = 1:numel(arr)
                        nm = get_param(arr(k), 'Name');
                        d = get_param(arr(k), 'CompiledPortDimensions');
                        w = get_param(arr(k), 'CompiledPortWidth');
                        dt = get_param(arr(k), 'CompiledPortDataType');
                        fprintf('  %-20s %s[%d] %-16s dim=[%s] w=%d type=%s\n', ...
                            blks{i}, p{1}, k, nm, mat2str(d), w, dt);
                    end
                catch ME2
                    fprintf('  [warn] %s %s -> %s\n', blks{i}, p{1}, ME2.message);
                end
            end
        end
    catch ME
        fprintf(2, 'UPDATE FAIL: %s\n', ME.message);
        % 逐个 root cause（MATLAB 的 aggregate exception 会把所有原因堆在一起）
        ce = ME.cause;
        if ~isempty(ce)
            for i = 1:numel(ce)
                fprintf(2, '  cause[%d]: %s\n', i, ce{i}.message);
            end
        end
        fprintf(2, '%s\n', getReport(ME, 'extended'));
    end

    fprintf('\n===== Step 4: Simulink model advisor - basic checks =====\n');
    try
        load_system(mdl);
        % 悬空 / 未连线端口统计
        ph = get_param([mdl '/SafetyFSM'], 'PortHandles');
        for k = 1:numel(ph.Inport)
            fprintf('  FSM inport %d (%s): line=%s\n', k, get_param(ph.Inport(k), 'Name'), ...
                num2str(get_param(ph.Inport(k), 'Line')));
        end
        for k = 1:numel(ph.Outport)
            fprintf('  FSM outport %d (%s): line=%s\n', k, get_param(ph.Outport(k), 'Name'), ...
                num2str(get_param(ph.Outport(k), 'Line')));
        end
    catch ME
        fprintf(2, 'advisor FAIL: %s\n', ME.message);
    end

    diary off
end

function s = portName(obj)
    try
        if isempty(obj)
            s = '<none>';
        elseif isa(obj, 'Stateflow.State')
            s = ['S:' obj.Name];
        elseif isa(obj, 'Stateflow.Junction')
            s = ['J:' obj.Type];
        else
            s = class(obj);
        end
    catch
        s = '<?>';
    end
end

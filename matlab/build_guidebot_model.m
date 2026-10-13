function mdl = build_guidebot_model(scenarioName)
%BUILD_GUIDEBOT_MODEL  用脚本自动生成 GuideBot 的 Simulink 模型（MBD 工作流）。
%
%   为什么用脚本建模型而不是手动画？
%     1. 模型变成可版本管理的产物：改算法 -> 重新生成 -> 回归测试，全链路可复现。
%     2. 参数从 params.json 注入，不存在"模型里手抄的常数和代码不一致"——
%        这种漂移在"生成 C 代码烧进 MCU"的场景下是最难排查的一类 bug。
%     3. 任何人可以一行命令复现整个模型，而不是打开一个黑盒 slx。
%
%   架构（两条独立计算链，对应两种部署目标）：
%
%     [Perception_Planning]  extrinsic 的 AI 主机 + 可代码生成的安全判据
%                            -> planDelta / minClear / ttc / brakeRule / overlap / gate
%                  |
%     [SafetyFSM]  Stateflow 五态安全状态机 -> brake / cue
%                  |
%     [Actuator_Plant]  转向速率限幅 + 人/无动力轮运动学  psi_dot = v*tan(delta)/L
%
%   用法：  build_guidebot_model('mixed_1')

    if nargin < 1 || isempty(scenarioName), scenarioName = 'mixed_1'; end
    here = fileparts(mfilename('fullpath'));
    addpath(here);
    dataDir = fullfile(here, 'data');

    p  = gb_params(fullfile(dataDir, 'params.json'));
    worlds = jsondecode(fileread(fullfile(dataDir, 'scenarios.json')));
    if ~isfield(worlds, scenarioName)
        error('未知场景 %s，可选：%s', scenarioName, strjoin(fieldnames(worlds)', ', '));
    end
    wd = worlds.(scenarioName);
    c = p.control;  pc = p.planner;
    dt = c.dt;

    % 参数注入拆到独立函数，模型 InitFcn 回调也调用它：
    % 这样"只打开模型直接点运行"不会报 GB_X0 无法识别。
    gb_setup_base(scenarioName);

    mdl = 'guidebot_mil';
    step('params/world ready');
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    f = fullfile(here, [mdl '.slx']);
    if exist(f, 'file'), delete(f); end
    new_system(mdl);
    open_system(mdl);
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
                   'FixedStep', num2str(dt), 'StartTime', '0', 'StopTime', '60.0');
    % InitFcn：每次仿真初始化前自动注入参数（模型自包含的关键）。
    % 注意回调里的代码在运行时才解析，gb_setup_base 必须在 path 上
    % （guidebot_open.m 已 addpath；或手动 addpath 一次）。
    set_param(mdl, 'InitFcn', 'gb_setup_base();');

    step('add Clock');
    add_block('simulink/Sources/Clock', [mdl '/tNow'], 'Position', [60 60 110 80]);
    step('add Unit Delay');
    add_block('simulink/Discrete/Unit Delay', [mdl '/State'], ...
        'Position', [700 190 770 280], 'InitialCondition', 'GB_X0', ...
        'SampleTime', num2str(dt));
    step('add Demux');
    add_block('simulink/Signal Routing/Demux', [mdl '/Split'], ...
        'Position', [810 180 820 340], 'Outputs', '8');
    step('add To Workspace');
    add_block('simulink/Sinks/To Workspace', [mdl '/logState'], ...
        'Position', [860 360 940 400], 'VariableName', 'slxState', ...
        'SaveFormat', 'Structure With Time');

    step('add EMChart 1 (Perception_Planning)');
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [mdl '/Perception_Planning'], 'Position', [170 110 380 300]);
    step('add EMChart 2 (Actuator_Plant)');
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [mdl '/Actuator_Plant'], 'Position', [500 320 680 470]);

    sf = sfroot();

    ppScript = sprintf([ ...
        'function [planDelta,minClear,ttc,horizon,frontClear,brakeRule,overlap,gate,tight,over,drop] = pp(x,y,psi,v,prevDelta,t)\n' ...
        'coder.extrinsic(''gb_ai_host_step'');\n' ...
        'planDelta=0; minClear=1e9; ttc=1e9; horizon=1.0; frontClear=1e9; over=false; drop=false;\n' ...
        '[planDelta,minClear,ttc,horizon,frontClear,over,drop] = gb_ai_host_step(x,y,psi,v,prevDelta,t);\n' ...
        'deltaMax=%.12f; wheelbase=%.12f; mu=%.12f; g=%.12f; latency=%.12f;\n' ...
        'margin=%.12f; vMin=%.12f; bodyClear=%.12f; ttcBrakeS=%.12f; safetyClear=%.12f;\n' ...
        'brakeRule=false; overlap=false; gate=false; tight=false; dReq=0; latCan=0; latNeed=0;\n' ...
        '[b0,b1,b2,b3,dReq,latCan,latNeed] = gb_safety_compute(minClear,frontClear,ttc,v,horizon,deltaMax,wheelbase,mu,g,latency,margin,vMin,bodyClear,ttcBrakeS,safetyClear);\n' ...
        'brakeRule=logical(b0); overlap=logical(b1); gate=logical(b2); tight=logical(b3);\n' ...
        'end'], pc.delta_max_deg*pi/180.0, p.const.WHEELBASE, c.mu, c.g, c.latency_s, ...
        c.margin, c.v_min, c.body_clear, c.ttc_brake_s, c.safety_clear);

    % 关于 cue 的语义：它是枚举量（0 none / 3 slow / 4 stop / 5 overhead / 6 drop），
    % 绝对不能用区间判断。写成 "cue>3.5" 会把 cue=5/6 一起吃进来，
    % 于是"提示前方有悬空障碍"被当成"停止"执行 —— 人会被永久停死。
    % （逐帧 trace 表现为 x 冻结、v 单调衰减到 0。）必须与 Python/MIL 侧保持一致。
    apScript = sprintf([ ...
        'function st2 = ap(st,planDelta,brake,cue)\n' ...
        'dt=%.6f; WB=%.6f; RR=%.6f; HW=%.6f;\n' ...
        'pref=%.6f; slowF=%.6f; Tr=%.6f; slewDps=%.6f;\n' ...
        'x=st(1); y=st(2); psi=st(3); v=st(4); prevDelta=st(5); tgt=st(6); bk=st(7); held=st(8);\n' ...
        'maxStep=(slewDps*pi/180.0)*dt;\n' ...
        'delta=planDelta; step=delta-prevDelta;\n' ...
        'if step>maxStep, step=maxStep; end\n' ...
        'if step<-maxStep, step=-maxStep; end\n' ...
        'delta=prevDelta+step;\n' ...
        'if brake>0, delta=prevDelta*0.5; end\n' ...
        'if bk>0\n    tgt=-0.30;\n' ...
        'elseif abs(cue-4.0)<0.5\n    tgt=0.0;\n' ...
        'elseif abs(cue-3.0)<0.5\n    tgt=pref*slowF;\n' ...
        'else\n    tgt=pref;\n' ...
        'end\n' ...
        'v = v + (tgt-v)*(dt/Tr);\n' ...
        'if brake>0, v = v - 3.2*dt; end\n' ...
        'v = max(-0.45, min(2.0, v));\n' ...
        'if abs(v)>1e-4\n' ...
        '    psi = psi + v*tan(delta)/WB*dt;\n' ...
        '    x = x + v*cos(psi)*dt;\n' ...
        '    y = y + v*sin(psi)*dt;\n' ...
        '    lim = HW-RR;\n' ...
        '    if abs(y)>lim, y = sign(y)*lim; psi = 0.0; end\n' ...
        'end\n' ...
        'if brake>0 && abs(v)<0.06, held = held + dt; else, held = 0.0; end\n' ...
        'if held>5.0, bk = 1.2; held = 0.0; end\n' ...
        'bk = max(0.0, bk-dt);\n' ...
        'st2 = [x;y;psi;v;delta;tgt;bk;held];\n' ...
        'end'], dt, p.const.WHEELBASE, p.const.ROBOT_RADIUS, wd.half_width, ...
        1.15, 0.50, 0.35, c.slew_max_dps);

    step('set EMChart scripts');
    setEmChartScript(sf, [mdl '/Perception_Planning'], ppScript);
    setEmChartScript(sf, [mdl '/Actuator_Plant'],      apScript);

    step('build Stateflow FSM');
    okFSM = buildSafetyFSM(mdl);

    step('wire');
    P = @(b) get_param([mdl '/' b], 'PortHandles');
    hState = P('State'); hSplit = P('Split'); hPP = P('Perception_Planning');
    hAP = P('Actuator_Plant'); hClk = P('tNow'); hLog = P('logState');

    wire(mdl, hState.Outport(1), hSplit.Inport(1));
    wire(mdl, hClk.Outport(1),   hPP.Inport(6));
    for k = 1:5
        wire(mdl, hSplit.Outport(k), hPP.Inport(k));
    end
    wire(mdl, hState.Outport(1), hAP.Inport(1));
    wire(mdl, hPP.Outport(1),    hAP.Inport(2));    % planDelta

    if okFSM
        hFSM = P('SafetyFSM');
        for k = 6:9
            wire(mdl, hPP.Outport(k), hFSM.Inport(k-5));   % brakeRule/overlap/gate/tight
        end
        wire(mdl, hPP.Outport(10), hFSM.Inport(5));        % over
        wire(mdl, hPP.Outport(11), hFSM.Inport(6));        % drop
        wire(mdl, hFSM.Outport(1), hAP.Inport(3));         % brake
        wire(mdl, hFSM.Outport(2), hAP.Inport(4));         % cue
    else
        wire(mdl, hPP.Outport(6), hAP.Inport(3));
        wire(mdl, hPP.Outport(9), hAP.Inport(4));
    end

    wire(mdl, hAP.Outport(1),   hState.Inport(1));
    wire(mdl, hState.Outport(1), hLog.Inport(1));

    step('save');
    save_system(mdl, f);
    fprintf('[build] 模型已生成: %s   (Stateflow: %s)\n', f, onoff(okFSM));
end

% -------------------------------------------------------------------------
function step(s)
    fprintf('[%s] %s ...\n', datestr(now, 'HH:MM:SS'), s);
end

function wire(mdl, src, dst)
    try
        add_line(mdl, src, dst, 'autorouting', 'on');
    catch ME
        fprintf('  [warn] 连线失败: %s\n', ME.message);
    end
end

function setEmChartScript(sf, path, script)
    emc = sf.find('Path', path, '-isa', 'Stateflow.EMChart');
    if isempty(emc)
        emc = sf.find('-isa', 'Stateflow.EMChart', 'Path', path);
    end
    assert(~isempty(emc), '找不到 MATLAB Function 块 %s', path);
    emc.Script = char(script);
end

function ok = buildSafetyFSM(mdl)
    ok = false;
    try
        load_system('sflib');
        add_block('sflib/Chart', [mdl '/SafetyFSM'], 'Position', [170 320 600 560]);
        rt = sfroot();
        m  = rt.find('-isa', 'Simulink.BlockDiagram', 'Name', mdl);
        ch = m.find('-isa', 'Stateflow.Chart', 'Name', 'SafetyFSM');
        assert(~isempty(ch), 'Chart 未创建');

        addData(ch, 'brakeRule', 'Input');
        addData(ch, 'overlap',   'Input');
        addData(ch, 'gate',      'Input');
        addData(ch, 'tight',     'Input');
        addData(ch, 'over',      'Input');
        addData(ch, 'drop',      'Input');
        addData(ch, 'brake',     'Output');
        addData(ch, 'cue',       'Output');

        % 注意：state 之间必须留出空白间隙。Position=[left top width height]，
        % 如果一个状态的底边正好等于另一个的顶边（相切），Stateflow 编译期会报
        % "状态 A 不能与另一个状态或结点交叉"，并且会把端口维度错误一起带出来
        % （是级联反应，不是真的维度问题）——这类报错极具误导性，值得记一笔。
        sN = addState(ch, 'NOMINAL',     [ 40  40 120  56], 'brake=0; cue=0;');
        sS = addState(ch, 'SLOW',        [ 40 160 120  56], 'brake=0; cue=3;');
        sO = addState(ch, 'HAZARD_OVER', [230  40 140  56], 'brake=0; cue=5;');
        sD = addState(ch, 'HAZARD_DROP', [230 160 140  56], 'brake=0; cue=6;');
        sB = addState(ch, 'STOP',        [440 100 120  56], 'brake=1; cue=4;');

        % 顶层互斥（EXCLUSIVE_OR）状态必须有默认转移，否则初始状态未定义，
        % 编译期报错 "chart 缺少默认转移"。默认转移的实现方式是：
        % 创建一条 Transition，Destination 指向目标状态，Source 保持为空。
        dflt = Stateflow.Transition(ch);
        dflt.Destination = sN;
        dflt.LabelString = '';
        try
            dflt.Source = [];
        catch ME2
            fprintf('  [warn] 无法清空默认转移源: %s\n', ME2.message);
        end

        % 注意：chart 的动作语言是 MATLAB（不是 C），逻辑取反必须写 "~"，
        % 写成 C 风格的 "!" 会在编译期报"解析失败"，且错误信息只给转移标签、
        % 不指出错在哪一格——这是 Stateflow 脚本建模最浪费时间的一类坑。
        stopCond = '[brakeRule || overlap || gate]';
        addTrans(ch, sN, sB, stopCond);
        addTrans(ch, sS, sB, stopCond);
        addTrans(ch, sO, sB, stopCond);
        addTrans(ch, sD, sB, stopCond);
        addTrans(ch, sB, sN, '[~(brakeRule || overlap || gate)]');
        addTrans(ch, sN, sO, '[over]');
        addTrans(ch, sN, sD, '[drop]');
        addTrans(ch, sN, sS, '[tight]');
        addTrans(ch, sO, sN, '[!over]');
        addTrans(ch, sD, sN, '[!drop]');
        addTrans(ch, sS, sN, '[!tight]');

        ok = true;
    catch ME
        fprintf('  [warn] Stateflow 构建失败，降级为纯 Simulink: %s\n', ME.message);
        try, delete_block([mdl '/SafetyFSM']); catch, end
    end
end

function d = addData(ch, name, scope)
    d = Stateflow.Data(ch);
    d.Name  = name;
    d.Scope = scope;
end

function s = addState(ch, name, pos, actions)
    s = Stateflow.State(ch);
    s.Name = name;
    s.Position = pos;
    % 注意：必须用 sprintf 把 \n 变成真正的换行。
    % 用字符拼接 [name '\nentry: ' actions] 的话 \n 是字面量，
    % Stateflow 会把整串当成状态名，得到 "NOMINAL\nentry: brake=0;" 这种鬼名字。
    s.LabelString = sprintf('%s\nentry: %s', name, actions);
end

function addTrans(ch, src, dst, label)
    t = Stateflow.Transition(ch);
    t.Source = src;
    t.Destination = dst;
    t.LabelString = label;
end

function s = onoff(b)
    if b, s = 'ON'; else, s = 'OFF(降级)'; end
end

function report = verify_against_python(dataDir)
%VERIFY_AGAINST_PYTHON  逐帧交叉验证：MATLAB 版控制律 vs Python 版控制律。
%
%   这是"模型在环（MIL）"里最容易被跳过、但最重要的一步。
%   同一套公式在两种语言下必须给出**逐帧 bit-level 一致**的决策，
%   否则说明存在隐藏的类型/取整/截断差异 —— 这类差异一旦被
%   MATLAB Coder 编译成 C 再烧进 MCU，就变成了现场最难复现的 bug。
%
%   做法：Python 侧把 supervise() 每一次调用的入参+出参录成 CSV
%   （见 export_matlab.py），MATLAB 侧只做一件事 —— 用同样的输入
%   重算一遍，然后逐帧比对。
%
%   判定标准（工程上可接受）：
%     delta 最大偏差 < 1e-12 rad
%     brake/cue/reason 必须 100% 一致（离散量，不允许任何一帧不同）
%
%   用法：  verify_against_python('data')

    if nargin < 1 || isempty(dataDir)
        here = fileparts(mfilename('fullpath'));
        dataDir = fullfile(here, 'data');
    end

    p = gb_params(fullfile(dataDir, 'params.json'));
    c = p.control;

    T = readtable(fullfile(dataDir, 'trace_ref.csv'));
    n = height(T);
    fprintf('[%s] 载入 %d 帧决策日志\n', mfilename, n);

    dErr   = zeros(n, 1);
    brakeM = zeros(n, 1);
    cueM   = zeros(n, 1);
    reasM  = zeros(n, 1);
    dreqM  = zeros(n, 1);

    for i = 1:n
        [delta, brake, cue, reason, d_req] = gb_supervise( ...
            T.plan_delta(i), T.min_clear(i), T.v(i), T.prev_delta(i), c, ...
            T.horizon(i), T.delta_max(i), T.wheelbase(i), T.target_clear(i), ...
            T.overhead(i), T.drop(i), T.front_clear(i), T.ttc(i));
        dErr(i)   = abs(delta - T.delta_out(i));
        brakeM(i) = brake;
        cueM(i)   = cue;
        reasM(i)  = reason;
        dreqM(i)  = d_req;
    end

    maxDErr = max(dErr);
    nBrakeDiff = sum(brakeM ~= T.brake);
    nCueDiff   = sum(cueM   ~= T.cue);
    nReasDiff  = sum(reasM  ~= T.reason);
    maxDReqErr = max(abs(dreqM - T.d_req));

    fprintf('\n===== 交叉验证结果（Python vs MATLAB）=====\n');
    fprintf('  帧数                : %d\n', n);
    fprintf('  转向角最大偏差      : %.3e rad   (%s)\n', maxDErr, passfail(maxDErr < 1e-12));
    fprintf('  brake 不一致帧数    : %d          (%s)\n', nBrakeDiff, passfail(nBrakeDiff == 0));
    fprintf('  cue   不一致帧数    : %d          (%s)\n', nCueDiff, passfail(nCueDiff == 0));
    fprintf('  reason不一致帧数    : %d          (%s)\n', nReasDiff, passfail(nReasDiff == 0));
    fprintf('  d_req 最大偏差      : %.3e m\n', maxDReqErr);

    ok = (maxDErr < 1e-12) && (nBrakeDiff == 0) && (nCueDiff == 0) && (nReasDiff == 0);
    fprintf('\n  结论: %s\n', passfail(ok));
    if ~ok
        fprintf('  !! 存在不一致，禁止进入代码生成环节\n');
    end

    % 决策分布统计（顺便看看这台设备到底在干嘛）
    fprintf('\n  cue 分布: ');
    names = {'none','turn_left','turn_right','slow_down','stop','overhead','drop_ahead'};
    for k = 0:6
        fprintf('%s=%d  ', names{k+1}, sum(cueM == k));
    end
    fprintf('\n  reason 分布: ');
    rnames = {'nominal','brake_rule','overlap','gate','overhead_haz','neg_obstacle','tight','steering'};
    for k = 0:7
        fprintf('%s=%d  ', rnames{k+1}, sum(reasM == k));
    end
    fprintf('\n');

    report = struct('frames', n, 'maxDeltaErr', maxDErr, ...
        'brakeDiff', nBrakeDiff, 'cueDiff', nCueDiff, ...
        'reasonDiff', nReasDiff, 'maxDReqErr', maxDReqErr, 'pass', ok);

    outDir = fullfile(dataDir, '..', 'results');
    if ~exist(outDir, 'dir'), mkdir(outDir); end
    fid = fopen(fullfile(outDir, 'mil_xverify.json'), 'w');
    fprintf(fid, '%s', jsonencode(report));
    fclose(fid);
    fprintf('\n  -> %s\n', fullfile(outDir, 'mil_xverify.json'));
end

function s = passfail(ok)
    if ok, s = 'PASS'; else, s = 'FAIL'; end
end

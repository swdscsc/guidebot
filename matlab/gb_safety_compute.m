function [brakeRule, overlap, gate, tight, dReq, latCan, latNeed] = gb_safety_compute( ...
        minClear, frontClear, ttc, v, horizon, deltaMax, wheelbase, ...
        mu, g, latency, margin, vMin, bodyClear, ttcBrakeS, safetyClear)
%GB_SAFETY_COMPUTE  安全判据的**纯计算部分**（不决定动作，只给出事实）。
%
%   为什么要和"决定动作"分开？
%   因为动作逻辑（优先级、状态记忆、提示词）交给 Stateflow 用状态机表达 —— 
%   状态机可以被 Simulink Design Verifier 做形式化验证、被 Coverage 做覆盖度分析；
%   而判据是连续量的不等式，用代码表达更自然。
%   把二者拆开，是 MBD（Model-Based Design）里的标准做法。
%
%   这个拆分还有第二个好处：本函数**完全可代码生成**（全标量、无动态内存），
%   可以连同 Stateflow 一起被 Embedded Coder 编译进 MCU。
%
%   brakeRule : 正前方自由距离 < 制动距离需求  => 停不下来了
%   overlap   : 预测轨迹马上就要压到障碍（看 TTC，不是看最小净空）
%   gate      : 当前速度下横向带不开 => 诚实承认做不到
%   tight     : 净空偏紧（只提示减速，不干预）

    dReq    = abs(v)*abs(v) / (2.0*mu*g) + abs(v)*latency + margin;

    % 用 max(v, vMin) 评估横移能力：v=0 时 latCan 必然为 0，
    % 直接比较会得出"永远走不了"的荒谬结论并死锁。
    vEff    = max(abs(v), vMin);
    sArc    = vEff * horizon;
    latCan  = (sArc*sArc) * abs(tan(deltaMax)) / (2.0*wheelbase);
    latNeed = max(0.0, safetyClear - minClear);

    brakeRule = double(frontClear < dReq);
    overlap   = double((minClear < bodyClear) && (ttc < ttcBrakeS));
    gate      = double(latNeed > latCan);
    tight     = double(minClear < 0.45);
end

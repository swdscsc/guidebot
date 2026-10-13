function [delta, brake, cue, reason, d_req] = gb_supervise_entry( ...
        plan_delta, min_clear, v, prev_delta, horizon, delta_max, wheelbase, ...
        target_clear, overhead, drop, front_clear, ttc, ...
        mu, g, latency_s, margin, slew_max_dps, v_min, body_clear, ...
        ttc_brake_s, safety_clear, slow_threshold, ...
        enable_brake_rule, enable_gate, enable_slew_limit, dt)
%GB_SUPERVISE_ENTRY  安全监督层的**代码生成入口**（全标量，无结构体入参）。
%
%   为什么要有这个包装函数？
%   gb_supervise.m 接收一个参数结构体 —— 这在 MATLAB 里很方便，
%   但作为"要烧进 MCU"的入口，结构体意味着运行时字段查找、可变类型、
%   难以静态分析。MBD 的标准做法是：把会进代码生成的那一层，
%   接口压平成**纯标量**，让编译器能在编译期确定一切。
%
%   于是接口变成：
%       input  : 26 个 double 标量
%       output : delta / brake / cue / reason / d_req
%
%   生成 C 之后，MCU 侧只需要每 100Hz 调一次：
%       gb_supervise_entry(&delta, &brake, &cue, &reason, &d_req, ...);
%   没有任何动态内存分配、没有浮点异常路径、没有库依赖。
%
%   codegen 命令见 codegen_controller.m。

    c.mu = mu;
    c.g  = g;
    c.latency_s = latency_s;
    c.margin = margin;
    c.slew_max_dps = slew_max_dps;
    c.v_min = v_min;
    c.body_clear = body_clear;
    c.ttc_brake_s = ttc_brake_s;
    c.safety_clear = safety_clear;
    c.slow_threshold = slow_threshold;
    c.enable_brake_rule = enable_brake_rule;
    c.enable_gate = enable_gate;
    c.enable_slew_limit = enable_slew_limit;
    c.dt = dt;

    [delta, brake, cue, reason, d_req] = gb_supervise( ...
        plan_delta, min_clear, v, prev_delta, c, horizon, delta_max, ...
        wheelbase, target_clear, overhead, drop, front_clear, ttc);
end

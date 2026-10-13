function [planDelta, minClear, ttcMin, horizon, frontClear, overFlag, dropFlag] = ...
        gb_ai_host_step(x, y, psi, v, prevDelta, tNow)
%GB_AI_HOST_STEP  "AI 主机"侧的一次完整推理：感知 → 感知隔离 → 规划。
%
%   在 Simulink 模型里这一块被声明为 coder.extrinsic，理由是**架构性的**，
%   不是偷懒：
%
%       AI 主机（Linux / 20Hz）跑的是深度学习感知 + 采样规划，它需要
%       动态内存、随机数、变长障碍列表 —— 这类东西本来就不该进代码生成。
%       真正必须"可生成 C 代码、可形式化验证"的只有下面那一层：
%           SafetyMCU（MCU / 100Hz）—— 只有它掌握刹车。
%
%   这个划分正好对应本项目一贯的架构主张：**智能可以崩，安全不能崩**。
%   就算 AI 主机死机、掉帧、返回垃圾值，只要 SafetyMCU 还在，
%   设备就退化成一根会刹车的普通手杖，而不是把人推向危险。

    % 世界与参数从 base workspace 取（由 build_guidebot_model.m 注入）
    obs = evalin('base', 'GB_OBS');
    drp = evalin('base', 'GB_DRP');
    s   = evalin('base', 'GB_SENSOR');
    pc  = evalin('base', 'GB_PLANNER');
    WB  = evalin('base', 'GB_WHEELBASE');
    RR  = evalin('base', 'GB_ROBOT_RADIUS');
    HW  = evalin('base', 'GB_HALF_WIDTH');

    % --- 感知 ---
    low  = gb_cast_layer(x, y, psi, obs, s, s.h_low,  tNow);
    if s.enable_high > 0
        high = gb_cast_layer(x, y, psi, obs, s, s.h_high, tNow);
    else
        high = s.max_range * ones(s.n_rays, 1);
    end
    [drops, nearest] = gb_probe_drops(x, y, psi, drp, s);
    seen = gb_perceived_obstacles(x, y, psi, obs, s, tNow);

    frontClear = s.max_range;
    ang = gb_ray_angles(s);
    for i = 1:numel(ang)
        if abs(ang(i)) <= 0.20
            frontClear = min([frontClear, low(i), high(i)]);
        end
    end

    overFlag = (s.enable_high > 0) && any(high < s.max_range & low > high + 0.25);
    dropFlag = ~isempty(nearest);

    % --- 规划 ---
    if pc.unknown_is_obstacle > 0
        mux = 1e9;
    else
        mux = s.max_range * 0.9;
    end
    [planDelta, minClear, ttcMin, horizon] = gb_plan_steering( ...
        x, y, psi, v, prevDelta, seen, drops, HW, mux, pc, ...
        frontClear, tNow, WB, RR);
end

function a = gb_ray_angles(s)
    half = s.fov_deg * pi/180.0 / 2.0;
    a = -half + (2.0*half) * ((0:s.n_rays-1) ./ max(s.n_rays-1, 1));
end

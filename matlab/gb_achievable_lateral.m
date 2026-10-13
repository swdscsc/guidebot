function lat = gb_achievable_lateral(v, horizon, delta_max, wheelbase)
%GB_ACHIEVABLE_LATERAL  给定速度下"最多能把人横向带开多少米"。
%
%   恒定转角 delta 下轨迹是半径 R = L / tan(delta) 的圆弧，
%   走过弧长 s = v*T 后的横向偏移量约为
%
%        s^2 / (2R) = (v*T)^2 * tan(delta) / (2L)
%
%   注意它**正比于 v^2** —— 速度一低，横移能力平方级衰减。
%   这不是"转弯半径不够"，而是更根本的一条：
%
%        psi_dot = v * tan(delta) / L        v -> 0  ==>  psi_dot -> 0
%
%   即：**用户站住不动时，这台设备的转向权限严格为零**。
%   再强的 AI 也转不动一个静止的无动力轮。
%   正确工程做法不是硬撑，而是诚实 —— 刹停并提示用户后退重来。
%   （Glide 的"无动力轮"设计把这条约束刻进了机械结构里。）

    s = abs(v) * horizon;
    lat = (s*s) * abs(tan(delta_max)) / (2.0 * wheelbase);
end

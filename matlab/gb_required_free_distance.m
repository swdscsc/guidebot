function d = gb_required_free_distance(v, c)
%GB_REQUIRED_FREE_DISTANCE  制动距离不等式（安全走廊约束）。
%
%       d_free >= v^2 / (2*mu*g) + v*T_latency + margin
%
%   三项的物理含义：
%     1) v^2/(2*mu*g)  纯滑动制动距离（摩擦圆上限，mu 取保守值 0.6）
%     2) v*T_latency   感知-规划-通信的端到端延迟里"已经走出去"的那段距离
%     3) margin        建模误差 / 地面湿滑 / 用户反应不一致的余量
%
%   这一条是整个项目的核心公式：它把"AI 有多聪明"这个玄学问题，
%   换成了一个可验证的几何不等式。任何一条候选轨迹，只要它前方
%   的自由空间小于 d，就**不许**被下发 —— 不管规划器觉得它多优。
%
%   注：与自动驾驶 RSS 规则同源，但做了行人化改写
%   （人不具备 1g 的制动能力，速度也不是设备能命令的）。

    d = v*v / (2.0*c.mu*c.g) + v*c.latency_s + c.margin;
end

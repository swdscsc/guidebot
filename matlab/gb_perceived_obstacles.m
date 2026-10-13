function seen = gb_perceived_obstacles(x, y, psi, obs, s, t_abs)
%GB_PERCEIVED_OBSTACLES  严格的信息隔离：只把"这一帧真能被感知到"的障碍交给规划器。
%
%   这一步极其重要。早期版本直接把真值障碍全量喂给规划器（上帝视角），
%   结果关掉悬空层射线之后机器人依然完美避开看不见的横杆 ——
%   Ablation 完全失效，结论全是假的。任何带"我看得见 vs 看不见"主张的
%   机器人评测，都必须在这一层做严格信息隔离。
%
%   三重过滤：距离 -> 视野角 -> 层高度。
%
%   注意：输出存的是 t=0 基准位置 + 速度，外推由 planner 用绝对时间完成。
%   若在感知层就外推，等于让规划器在"已外推过的位置"上再外推一次，
%   会双重计时间 —— 这是把 Python 版移植到 MATLAB 时最容易踩的坑之一。

    if isempty(obs)
        seen = zeros(0, 5);
        return;
    end
    halfFov = s.fov_deg * pi/180.0 / 2.0;
    cx = obs(:,1) + obs(:,6) * t_abs;
    cy = obs(:,2) + obs(:,7) * t_abs;
    dist = hypot(cx - x, cy - y);

    keep = dist <= (s.max_range + obs(:,3));

    rel = gb_wrap_angle(atan2(cy - y, cx - x) - psi);
    angRad = atan2(obs(:,3), max(dist, 0.1));
    keep = keep & (abs(rel) <= (halfFov + angRad));

    visHigh = (s.enable_high > 0) & (obs(:,4) <= s.h_high) & (s.h_high <= obs(:,5));
    visLow  = (obs(:,4) <= s.h_low) & (s.h_low <= obs(:,5));
    keep = keep & (visHigh | visLow);

    seen = [obs(keep, 1:3), obs(keep, 6:7)];   % [x0, y0, r, vx, vy]
end

function ranges = gb_cast_layer(ox, oy, psi, obs, s, height, t_rel)
%GB_CAST_LAYER  投射一层扇形射线，返回每个角度的最近命中距离（含噪声 / 丢帧）。
%
%   两层高度：
%     h_low  = 0.95m（相机层）—— 看不到 1.35~1.85m 的悬空横杆
%     h_high = 1.50m（头部层）—— 看不到地面上的低矮石墩
%   射线与圆柱求交有闭式解，这里用隐式扩展一次性算 n_rays x N_obs 的解析解，
%   比在循环里写 for 快一个量级（Simulink 里这一步是实时性的大头）。

    half = s.fov_deg * pi/180.0 / 2.0;
    n = s.n_rays;
    th = -half + (2.0*half) * ((0:n-1) ./ max(n-1, 1));
    ang = psi + th;
    dx = cos(ang(:));                        % n x 1
    dy = sin(ang(:));

    if isempty(obs)
        ranges = s.max_range * ones(n, 1);
        return;
    end

    cx = (obs(:,1) + obs(:,6) * t_rel)';     % 1 x N（绝对时间外推）
    cy = (obs(:,2) + obs(:,7) * t_rel)';
    rr = obs(:,3)';

    fx = ox - cx;
    fy = oy - cy;

    b = 2.0 * (dx .* fx + dy .* fy);         % n x N
    c = fx.^2 + fy.^2 - rr.^2;               % 1 x N（广播）
    disc = b.^2 - 4.0 * c;

    inZ  = (obs(:,4) <= height) & (height <= obs(:,5));   % N x 1
    valid = (disc >= 0) & inZ';

    sq = sqrt(max(disc, 0));
    s1 = (-b - sq) / 2.0;
    s2 = (-b + sq) / 2.0;

    hit = inf(n, numel(rr));
    m1 = valid & (s1 > 0);
    m2 = valid & (s1 <= 0) & (s2 > 0);
    hit(m1) = s1(m1);
    hit(m2) = 0.0;

    ranges = min(hit, [], 2);
    ranges(~isfinite(ranges)) = s.max_range;
    ranges = min(ranges, s.max_range);

    % ---- 噪声与丢帧 ----
    if s.dropout_prob > 0 || s.range_noise_abs > 0
        for i = 1:n
            if rand() < s.dropout_prob
                % 丢帧：当成"未知"，下游必须按"未知即高危"处理
                ranges(i) = s.max_range;
            else
                ranges(i) = ranges(i) + randn() * (s.range_noise_abs + ...
                            s.range_noise_rel * ranges(i));
                ranges(i) = max(0.0, min(s.max_range, ranges(i)));
            end
        end
    end
end

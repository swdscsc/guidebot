function [found, nearest] = gb_probe_drops(ox, oy, psi, drp, s)
%GB_PROBE_DROPS  近距下视探头：只能看到脚前有限范围内的负障碍（坑 / 下行台阶）。
%
%   这是整个系统里"距离最短却最要命"的通道 —— 探测距离只有 1.3m，
%   所以速度越快留给制动的时间越少，直接决定了控制层的制动距离约束：
%       d_free >= v^2/(2*mu*g) + v*T_latency + margin
%   换成数字看：v=1.15 m/s、mu=0.6、T=0.1s 时需要约 0.34m 的自由空间，
%   而探头只有 1.3m —— 留给"看到坑之后还能绕开"的余量其实非常薄。

    found = zeros(0, 3);
    nearest = [];
    if s.enable_probe <= 0 || isempty(drp), return; end

    fx = cos(psi);  fy = sin(psi);
    lx = -fy;       ly = fx;
    d = 0.2;
    while d <= s.probe_reach
        lat = -s.probe_halfwidth;
        while lat <= s.probe_halfwidth + 1e-9
            px = ox + fx*d + lx*lat;
            py = oy + fy*d + ly*lat;
            idx = find((px - drp(:,1)).^2 + (py - drp(:,2)).^2 <= drp(:,3).^2);
            for j = 1:numel(idx)
                row = drp(idx(j), :);
                if isempty(found) || ~any(all(abs(found - row) < 1e-9, 2))
                    found = [found; row]; %#ok<AGROW>
                end
                if isempty(nearest) || d < nearest, nearest = d; end
            end
            lat = lat + 0.18;
        end
        d = d + s.probe_step;
    end
end

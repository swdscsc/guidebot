function cl = gb_clearance_at(px, py, obs, drp, t_rel, robot_radius)
%GB_CLEARANCE_AT  一个点到所有障碍（含运动外推）与虚拟负障碍的最小净空。
%
%   obs: Nx5 [x, y, r, vx, vy]  —— 位置按绝对时间 t_rel 做恒定速度外推
%   drp: Mx3 [x, y, r]          —— 负障碍（坑洞/下行台阶）是静止的
%
%   注意净空 = 圆心距 - 障碍半径 - 机器人半径，即"表面到表面"的距离。
%   它可以是负数（表示已经穿模），规划器的屏障函数正是指着这个负值工作的。

    cl = 1e9;

    if ~isempty(obs)
        ox = obs(:,1) + obs(:,4) * t_rel;
        oy = obs(:,2) + obs(:,5) * t_rel;
        d  = sqrt((px - ox).^2 + (py - oy).^2) - obs(:,3) - robot_radius;
        cl = min(min(d), cl);
    end

    if ~isempty(drp)
        d = sqrt((px - drp(:,1)).^2 + (py - drp(:,2)).^2) - drp(:,3) - robot_radius;
        cl = min(min(d), cl);
    end
end

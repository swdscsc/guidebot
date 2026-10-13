function obs = gb_load_world_obstacles(w)
%GB_LOAD_WORLD_OBSTACLES  把 scenarios.json 里的世界转成 Nx7 障碍矩阵。
%   列: [x, y, r, zmin, zmax, vx, vy]
%
%   zmin/zmax 是本项目建模的关键：
%     消防栓/石墩  -> z 低矮，相机层（0.95m）射线容易被糊掉
%     广告牌横杆   -> z 悬空（1.35~1.85m），相机层射线直接从下方穿过
%     行人/柱子    -> 全高度
%   因此必须有两层射线，而"关掉 h_high"就是悬空感知能力的消融开关。

    W = w.obstacles;
    n = numel(W);
    obs = zeros(n, 7);
    for i = 1:n
        obs(i, :) = [W(i).x, W(i).y, W(i).r, W(i).zmin, W(i).zmax, ...
                     W(i).vx, W(i).vy];
    end
end

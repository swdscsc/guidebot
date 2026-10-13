function drp = gb_load_world_drops(w)
%GB_LOAD_WORLD_DROPS  负障碍 -> Mx3 [x, y, r]（坑洞 / 下行台阶）。
%   只有近距下视探头能发现它们，探测距离 1.3m —— 速度越快留给制动的时间越少。
    D = w.drops;
    m = numel(D);
    drp = zeros(m, 3);
    for i = 1:m
        drp(i, :) = [D(i).x, D(i).y, D(i).r];
    end
end

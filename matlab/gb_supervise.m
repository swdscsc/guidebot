function [delta, brake, cue, reason, d_req] = gb_supervise(plan_delta, min_clear, ...
        v, prev_delta, c, horizon, delta_max, wheelbase, target_clear, ...
        overhead, drop, front_clear, ttc)
%GB_SUPERVISE  被动转向控制器 + 失效安全监督层 + 人机反馈词表。
%
%   【本函数是整个项目里唯一会被代码生成到 MCU 的部分】
%   所以约束很硬：全标量入参、无 cell / 无字符串 / 无动态内存分配，
%   cue 与 reason 全部走整数枚举（见 gb_cue_name / gb_reason_name）。
%
%   三级安全判据（顺序即优先级，短路）：
%     1. brake_rule   正前方自由距离 < 制动距离 ⇒ 停不下来了，必须停
%     2. overlap      预测轨迹很快就要压到障碍（看 TTC，不是看最小净空）⇒ 停
%     3. gate         当前速度下横向带不开 ⇒ 诚实承认做不到，停
%
%   为什么第 2 条必须用 TTC 而不是 min_clear？
%     预测窗口有 3~4 米长，一个 1.5 秒之后才会擦到的东西如果现在就刹停，
%     人一停 ⇒ 转向权限归零 ⇒ 再也规划不出绕行 ⇒ **永久死锁**。
%     正确的语义是"冲突发生在多远的未来"：远的交给转向，近的才刹车。
%     （这是我在 v0.5 上踩的第六个坑，靠 80%→87% 的完成率提升验证的。）
%
%   为什么第 1 条只用"正前方"而不是整条轨迹的最小净空？
%     后者包含"侧身擦过一个 20cm 外的柱子"这种良性情况，用它触发硬刹停
%     会让设备在密集场景寸步难行（v0.5 踩的坑：STOP/km 高达 600）。
%     真正的语义是：**我还能不能在撞上正前方的东西之前停下来**。
%
%   输入：
%     plan_delta    规划器选出的转向角 [rad]
%     min_clear     该候选轨迹的最小通行净空 [m]
%     v             用户当前步速 [m/s]（外部输入，设备无法命令）
%     prev_delta    上一帧下发的转向角 [rad]
%     c             控制参数结构体（见 gb_params）
%     horizon/delta_max/wheelbase/target_clear  规划器状态
%     overhead/drop 悬空危险 / 负障碍标志
%     front_clear   正前方锥形自由距离 [m]
%     ttc           最近冲突发生的时间 [s]
%
%   输出：
%     delta   实际下发给舵机的转向角（已做速率限幅）[rad]
%     brake   是否主动刹车（0/1）
%     cue     触觉词表枚举（0=none 1=left 2=right 3=slow 4=stop 5=overhead 6=drop）
%     reason  决策原因枚举（见 gb_reason_name）
%     d_req   当前制动距离需求 [m]（用于日志与 HIL 断言）

    d_req = gb_required_free_distance(abs(v), c);

    % 用 max(v, v_min) 评估横移能力：v=0 时 lateral_can 必然为 0，
    % 直接比较会得出"永远走不了"的荒谬结论并死锁。
    % 正确的语义是：假设用户至少会以 v_min 的行走节奏继续前进。
    lateral_can  = gb_achievable_lateral(max(abs(v), c.v_min), horizon, delta_max, wheelbase);
    lateral_need = max(0.0, c.safety_clear - min_clear);   % 安全净空，不是舒适净空

    brake  = 0.0;
    cue    = 0.0;    % none
    reason = 0.0;    % nominal

    if c.enable_brake_rule > 0 && front_clear < d_req
        % --- 1) 安全走廊（制动距离）约束 ---
        brake  = 1.0;
        cue    = 4.0;    % stop
        reason = 1.0;    % brake_rule
    elseif c.enable_brake_rule > 0 && min_clear < c.body_clear && ttc < c.ttc_brake_s
        % --- 2) 预测轨迹马上就要压到障碍 ---
        brake  = 1.0;
        cue    = 4.0;
        reason = 2.0;    % overlap
    elseif c.enable_gate > 0 && lateral_need > lateral_can
        % --- 3) 低速可控性门限 ---
        brake  = 1.0;
        cue    = 4.0;
        reason = 3.0;    % gate
    else
        % --- 信息类提示（不干预，只告知）---
        if overhead > 0 && min_clear < 1.5
            cue    = 5.0;    % overhead
            reason = 4.0;
        elseif drop > 0
            cue    = 6.0;    % drop_ahead
            reason = 5.0;
        elseif min_clear < c.slow_threshold
            cue    = 3.0;    % slow_down
            reason = 6.0;
        elseif abs(plan_delta) > 8.0*pi/180.0
            if plan_delta > 0
                cue = 1.0;   % turn_left
            else
                cue = 2.0;   % turn_right
            end
            reason = 7.0;
        end
    end

    % --- 转向速率限制（体感平滑）：舵机有转速上限，否则体感会"抽搐" ---
    delta = plan_delta;
    if c.enable_slew_limit > 0
        max_step = (c.slew_max_dps * pi/180.0) * c.dt;
        step = delta - prev_delta;
        if step >  max_step, step =  max_step; end
        if step < -max_step, step = -max_step; end
        delta = prev_delta + step;
    end
    if brake > 0
        % 刹车时不再试图转向，保持直线更安全
        delta = prev_delta * 0.5;
    end
end

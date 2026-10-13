/*
 * File: gb_supervise_entry.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-10-12 22:03:07
 */

/* Include Files */
#include "gb_supervise_entry.h"
#include "gb_supervise_entry_data.h"
#include "gb_supervise_entry_initialize.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * GB_SUPERVISE_ENTRY  安全监督层的**代码生成入口**（全标量，无结构体入参）。
 *
 *    为什么要有这个包装函数？
 *    gb_supervise.m 接收一个参数结构体 —— 这在 MATLAB 里很方便，
 *    但作为"要烧进 MCU"的入口，结构体意味着运行时字段查找、可变类型、
 *    难以静态分析。MBD 的标准做法是：把会进代码生成的那一层，
 *    接口压平成**纯标量**，让编译器能在编译期确定一切。
 *
 *    于是接口变成：
 *        input  : 26 个 double 标量
 *        output : delta / brake / cue / reason / d_req
 *
 *    生成 C 之后，MCU 侧只需要每 100Hz 调一次：
 *        gb_supervise_entry(&delta, &brake, &cue, &reason, &d_req, ...);
 *    没有任何动态内存分配、没有浮点异常路径、没有库依赖。
 *
 *    codegen 命令见 codegen_controller.m。
 *
 * Arguments    : double plan_delta
 *                double min_clear
 *                double v
 *                double prev_delta
 *                double horizon
 *                double delta_max
 *                double wheelbase
 *                double target_clear
 *                double overhead
 *                double drop
 *                double front_clear
 *                double ttc
 *                double mu
 *                double g
 *                double latency_s
 *                double margin
 *                double slew_max_dps
 *                double v_min
 *                double body_clear
 *                double ttc_brake_s
 *                double safety_clear
 *                double slow_threshold
 *                double enable_brake_rule
 *                double enable_gate
 *                double enable_slew_limit
 *                double dt
 *                double *delta
 *                double *brake
 *                double *cue
 *                double *reason
 *                double *d_req
 * Return Type  : void
 */
void gb_supervise_entry(double plan_delta, double min_clear, double v,
                        double prev_delta, double horizon, double delta_max,
                        double wheelbase, double target_clear, double overhead,
                        double drop, double front_clear, double ttc, double mu,
                        double g, double latency_s, double margin,
                        double slew_max_dps, double v_min, double body_clear,
                        double ttc_brake_s, double safety_clear,
                        double slow_threshold, double enable_brake_rule,
                        double enable_gate, double enable_slew_limit, double dt,
                        double *delta, double *brake, double *cue,
                        double *reason, double *d_req)
{
  double s;
  double step;
  int b_brake;
  boolean_T guard1;
  (void)target_clear;
  if (!isInitialized_gb_supervise_entry) {
    gb_supervise_entry_initialize();
  }
  /* GB_SUPERVISE  被动转向控制器 + 失效安全监督层 + 人机反馈词表。 */
  /*  */
  /*    【本函数是整个项目里唯一会被代码生成到 MCU 的部分】 */
  /*    所以约束很硬：全标量入参、无 cell / 无字符串 / 无动态内存分配， */
  /*    cue 与 reason 全部走整数枚举（见 gb_cue_name / gb_reason_name）。 */
  /*  */
  /*    三级安全判据（顺序即优先级，短路）： */
  /*      1. brake_rule   正前方自由距离 < 制动距离 ⇒ 停不下来了，必须停 */
  /*      2. overlap      预测轨迹很快就要压到障碍（看 TTC，不是看最小净空）⇒ 停
   */
  /*      3. gate         当前速度下横向带不开 ⇒ 诚实承认做不到，停 */
  /*  */
  /*    为什么第 2 条必须用 TTC 而不是 min_clear？ */
  /*      预测窗口有 3~4 米长，一个 1.5 秒之后才会擦到的东西如果现在就刹停， */
  /*      人一停 ⇒ 转向权限归零 ⇒ 再也规划不出绕行 ⇒ **永久死锁**。 */
  /*      正确的语义是"冲突发生在多远的未来"：远的交给转向，近的才刹车。 */
  /*      （这是我在 v0.5 上踩的第六个坑，靠 80%→87% 的完成率提升验证的。） */
  /*  */
  /*    为什么第 1 条只用"正前方"而不是整条轨迹的最小净空？ */
  /*      后者包含"侧身擦过一个 20cm 外的柱子"这种良性情况，用它触发硬刹停 */
  /*      会让设备在密集场景寸步难行（v0.5 踩的坑：STOP/km 高达 600）。 */
  /*      真正的语义是：**我还能不能在撞上正前方的东西之前停下来**。 */
  /*  */
  /*    输入： */
  /*      plan_delta    规划器选出的转向角 [rad] */
  /*      min_clear     该候选轨迹的最小通行净空 [m] */
  /*      v             用户当前步速 [m/s]（外部输入，设备无法命令） */
  /*      prev_delta    上一帧下发的转向角 [rad] */
  /*      c             控制参数结构体（见 gb_params） */
  /*      horizon/delta_max/wheelbase/target_clear  规划器状态 */
  /*      overhead/drop 悬空危险 / 负障碍标志 */
  /*      front_clear   正前方锥形自由距离 [m] */
  /*      ttc           最近冲突发生的时间 [s] */
  /*  */
  /*    输出： */
  /*      delta   实际下发给舵机的转向角（已做速率限幅）[rad] */
  /*      brake   是否主动刹车（0/1） */
  /*      cue     触觉词表枚举（0=none 1=left 2=right 3=slow 4=stop 5=overhead
   * 6=drop） */
  /*      reason  决策原因枚举（见 gb_reason_name） */
  /*      d_req   当前制动距离需求 [m]（用于日志与 HIL 断言） */
  s = fabs(v);
  /* GB_REQUIRED_FREE_DISTANCE  制动距离不等式（安全走廊约束）。 */
  /*  */
  /*        d_free >= v^2 / (2*mu*g) + v*T_latency + margin */
  /*  */
  /*    三项的物理含义： */
  /*      1) v^2/(2*mu*g)  纯滑动制动距离（摩擦圆上限，mu 取保守值 0.6） */
  /*      2) v*T_latency   感知-规划-通信的端到端延迟里"已经走出去"的那段距离 */
  /*      3) margin        建模误差 / 地面湿滑 / 用户反应不一致的余量 */
  /*  */
  /*    这一条是整个项目的核心公式：它把"AI 有多聪明"这个玄学问题， */
  /*    换成了一个可验证的几何不等式。任何一条候选轨迹，只要它前方 */
  /*    的自由空间小于 d，就**不许**被下发 —— 不管规划器觉得它多优。 */
  /*  */
  /*    注：与自动驾驶 RSS 规则同源，但做了行人化改写 */
  /*    （人不具备 1g 的制动能力，速度也不是设备能命令的）。 */
  *d_req = (s * s / (2.0 * mu * g) + s * latency_s) + margin;
  /*  用 max(v, v_min) 评估横移能力：v=0 时 lateral_can 必然为 0， */
  /*  直接比较会得出"永远走不了"的荒谬结论并死锁。 */
  /*  正确的语义是：假设用户至少会以 v_min 的行走节奏继续前进。 */
  /* GB_ACHIEVABLE_LATERAL  给定速度下"最多能把人横向带开多少米"。 */
  /*  */
  /*    恒定转角 delta 下轨迹是半径 R = L / tan(delta) 的圆弧， */
  /*    走过弧长 s = v*T 后的横向偏移量约为 */
  /*  */
  /*         s^2 / (2R) = (v*T)^2 * tan(delta) / (2L) */
  /*  */
  /*    注意它**正比于 v^2** —— 速度一低，横移能力平方级衰减。 */
  /*    这不是"转弯半径不够"，而是更根本的一条： */
  /*  */
  /*         psi_dot = v * tan(delta) / L        v -> 0  ==>  psi_dot -> 0 */
  /*  */
  /*    即：**用户站住不动时，这台设备的转向权限严格为零**。 */
  /*    再强的 AI 也转不动一个静止的无动力轮。 */
  /*    正确工程做法不是硬撑，而是诚实 —— 刹停并提示用户后退重来。 */
  /*    （Glide 的"无动力轮"设计把这条约束刻进了机械结构里。） */
  if ((!(s >= v_min)) && (!rtIsNaN(v_min))) {
    s = v_min;
  }
  s = fabs(s) * horizon;
  step = safety_clear - min_clear;
  /*  安全净空，不是舒适净空 */
  b_brake = 0;
  *cue = 0.0;
  /*  none */
  *reason = 0.0;
  /*  nominal */
  if ((enable_brake_rule > 0.0) && (front_clear < *d_req)) {
    /*  --- 1) 安全走廊（制动距离）约束 --- */
    b_brake = 1;
    *cue = 4.0;
    /*  stop */
    *reason = 1.0;
    /*  brake_rule */
  } else if ((enable_brake_rule > 0.0) && (min_clear < body_clear) &&
             (ttc < ttc_brake_s)) {
    /*  --- 2) 预测轨迹马上就要压到障碍 --- */
    b_brake = 1;
    *cue = 4.0;
    *reason = 2.0;
    /*  overlap */
  } else {
    guard1 = false;
    if (enable_gate > 0.0) {
      if ((step <= 0.0) || rtIsNaN(step)) {
        step = 0.0;
      }
      if (step > s * s * fabs(tan(delta_max)) / (2.0 * wheelbase)) {
        /*  --- 3) 低速可控性门限 --- */
        b_brake = 1;
        *cue = 4.0;
        *reason = 3.0;
        /*  gate */
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      /*  --- 信息类提示（不干预，只告知）--- */
      if ((overhead > 0.0) && (min_clear < 1.5)) {
        *cue = 5.0;
        /*  overhead */
        *reason = 4.0;
      } else if (drop > 0.0) {
        *cue = 6.0;
        /*  drop_ahead */
        *reason = 5.0;
      } else if (min_clear < slow_threshold) {
        *cue = 3.0;
        /*  slow_down */
        *reason = 6.0;
      } else if (fabs(plan_delta) > 0.13962634015954636) {
        if (plan_delta > 0.0) {
          *cue = 1.0;
          /*  turn_left */
        } else {
          *cue = 2.0;
          /*  turn_right */
        }
        *reason = 7.0;
      }
    }
  }
  /*  --- 转向速率限制（体感平滑）：舵机有转速上限，否则体感会"抽搐" --- */
  *delta = plan_delta;
  if (enable_slew_limit > 0.0) {
    s = slew_max_dps * 3.1415926535897931 / 180.0 * dt;
    step = plan_delta - prev_delta;
    if (step > s) {
      step = s;
    }
    if (step < -s) {
      step = -s;
    }
    *delta = prev_delta + step;
  }
  if (b_brake > 0) {
    /*  刹车时不再试图转向，保持直线更安全 */
    *delta = prev_delta * 0.5;
  }
  *brake = b_brake;
}

/*
 * File trailer for gb_supervise_entry.c
 *
 * [EOF]
 */

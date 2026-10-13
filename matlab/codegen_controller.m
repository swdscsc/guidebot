function codegen_controller()
%CODEGEN_CONTROLLER  用 MATLAB Coder 把安全监督层生成 ANSI C。
%
%   这一步是整个 MATLAB 链路的落点，也是这个项目**区别于普通课程设计**的地方：
%
%       Python 原型  ->  SIL 交叉验证（逐帧 bit-level 一致）
%                    ->  MATLAB MIL（独立实现，统计结论一致）
%                    ->  **C 代码生成**（可编译进 MCU，无需手写一行 C）
%
%   为什么只生成这一层？
%   因为架构上只有"安全 MCU"这一层需要可验证、可固化、可形式化分析：
%     - 它全标量、无动态内存、无随机数、无变长数组
%     - 它掌握刹车，失效后果最严重
%     - 它必须能在 AI 主机死机时独立工作
%   而感知+规划那一层要跑深度学习、要动态内存，本来就不该进 MCU。
%
%   产出：matlab/codegen/lib/gb_supervise_entry.c / .h
%
%   用法：  codegen_controller()

    here = fileparts(mfilename('fullpath'));
    addpath(here);
    outDir = fullfile(here, 'codegen');

    fprintf('\n===== MATLAB Coder: 生成安全层 C 代码 =====\n');

    cfg = coder.config('lib');            % 静态库（也可换 'dll' / 'exe'）
    cfg.GenerateReport = true;
    cfg.ReportPotentialDifferences = false;
    cfg.TargetLang = 'C';
    % 只生成源码、不调用本机编译器编译。
    %   这样做有两个好处：
    %     (a) 不依赖机器上是否装了 MinGW / MSVC —— 换台电脑也能复现；
    %     (b) 目标本来就是交叉编译到 MCU，在 PC 上编译出来也没意义。
    %   真正部署时把 .c/.h 丢进厂商工具链即可。
    cfg.GenCodeOnly = true;
    % 不指定具体芯片：目标是"可移植的 ANSI C"，让 MCU 厂商的工具链去接。
    % （若指定了具体器件反而会引入不必要的对齐/字长假设。）

    % 26 个 double 标量入参
    A = repmat({coder.typeof(double(0))}, 1, 26);

    try
        codegen('-config', cfg, 'gb_supervise_entry', '-args', A, ...
                '-d', outDir, '-report');
        fprintf('\n  [OK] C 代码已生成 -> %s\n', outDir);
        d = dir(fullfile(outDir, '*.c'));
        for i = 1:numel(d)
            fprintf('       %s  (%d bytes)\n', d(i).name, d(i).bytes);
        end
        h = dir(fullfile(outDir, '*.h'));
        for i = 1:numel(h)
            fprintf('       %s  (%d bytes)\n', h(i).name, h(i).bytes);
        end
    catch ME
        fprintf('\n  [WARN] codegen 失败: %s\n', ME.message);
        fprintf('         常见原因：未安装 MATLAB Coder 许可证。\n');
        fprintf('         此时 MATLAB/Simulink 侧的 MIL 与交叉验证结论依然成立。\n');
    end
end

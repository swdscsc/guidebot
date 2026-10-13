%GUIDEBOT_OPEN  MATLAB 桌面启动入口：打开 GuideBot 模型 + 展示实跑结果。
%
%   用法（在任意终端）：
%     "E:\rj666\MATLAB R2024a(64bit)\bin\win64\matlab.exe" -sd <matlab目录> -r guidebot_open
%
%   打开后可以：
%     1. 双击 SafetyFSM 块查看五态安全状态机（NOMINAL/SLOW/HAZARD_*/STOP）；
%     2. 在命令行运行  run_live_demo('simulink')  重新仿真并刷新 results/；
%     3. 运行  run_live_demo('core')  重跑 SIL + MIL 消融套件。

here = fileparts(mfilename('fullpath'));
addpath(here);
res = fullfile(here, '..', 'results');

fprintf('== GuideBot MIL: 注入参数 + 打开模型 ==\n');
try
    gb_setup_base('mixed_1');   % 先注入 GB_* 变量，保证点"运行"就能跑
    open_system(fullfile(here, 'guidebot_mil.slx'));
catch ME
    fprintf(2, '打开模型失败: %s\n', ME.message);
end

if exist(fullfile(res, 'simulink_traj.png'), 'file')
    figure('Name', 'Simulink + Stateflow 轨迹 (mixed_1)', 'NumberTitle', 'off');
    imshow(fullfile(res, 'simulink_traj.png'));
end
if exist(fullfile(res, 'mil_traj.png'), 'file')
    figure('Name', 'MIL 消融轨迹（红点 = 主动刹车）', 'NumberTitle', 'off');
    imshow(fullfile(res, 'mil_traj.png'));
end

fprintf(['\nGuideBot 就绪。\n' ...
    '  - 点开 SafetyFSM 可以看到五态状态机\n' ...
    '  - 命令行运行 run_live_demo(''simulink'') 可重新仿真（约 1 分钟）\n' ...
    '  - 命令行运行 run_live_demo(''core'')    可重跑 SIL+MIL（约 1 分钟）\n']);

function gb_setup_base(scenarioName)
%GB_SETUP_BASE  把世界/传感器/控制参数注入 base workspace，供 guidebot_mil.slx 使用。
%
%   为什么存在这个函数？
%     Unit Delay 的初始条件 'GB_X0'、世界障碍 'GB_OBS' 等都住在 base workspace。
%     以前只有 build_guidebot_model 会注入它们 —— 意味着"只打开模型直接点运行"
%     必然报 "函数或变量 'GB_X0' 无法识别"。
%     现在把它拆出来，并挂到模型的 InitFcn 回调上：模型自包含，打开即跑。
%
%   用法：
%     gb_setup_base()              % 默认 mixed_1
%     gb_setup_base('corridor_1')  % 指定场景

    if nargin < 1 || isempty(scenarioName), scenarioName = 'mixed_1'; end
    here = fileparts(mfilename('fullpath'));
    addpath(here);
    dataDir = fullfile(here, 'data');

    p = gb_params(fullfile(dataDir, 'params.json'));
    worlds = jsondecode(fileread(fullfile(dataDir, 'scenarios.json')));
    assert(isfield(worlds, scenarioName), '未知场景 %s，可选：%s', ...
        scenarioName, strjoin(fieldnames(worlds)', ', '));
    wd = worlds.(scenarioName);

    assignin('base', 'GB_OBS',          gb_load_world_obstacles(wd));
    assignin('base', 'GB_DRP',          gb_load_world_drops(wd));
    assignin('base', 'GB_SENSOR',       p.sensor);
    assignin('base', 'GB_PLANNER',      p.planner);
    assignin('base', 'GB_WHEELBASE',    p.const.WHEELBASE);
    assignin('base', 'GB_ROBOT_RADIUS', p.const.ROBOT_RADIUS);
    assignin('base', 'GB_HALF_WIDTH',   wd.half_width);
    assignin('base', 'GB_X0', [1.0; 0.0; 0.0; 1.15 * 0.5; 0.0; 1.15; 0; 0]);
    fprintf('[gb_setup_base] 场景 %s 参数已注入 base workspace。\n', scenarioName);
end

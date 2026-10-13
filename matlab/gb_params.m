function p = gb_params(jsonPath)
%GB_PARAMS  从 Python 导出的 params.json 读入全部参数（单一真源）。
%
%   为什么不直接在 MATLAB 里再抄一遍常数？
%   因为只要存在两份，它们一定会漂移。而漂移在"代码生成到 MCU"的场景下
%   是最难排查的一类 bug：Simulink 里跑得好好的，烧进去行为就变了。
%   所以这里强制从 Python 侧导出的 JSON 读取，并在 load 时做字段校验。
%
%   用法：  p = gb_params('data/params.json');
%          p.control.mu, p.planner.w_barrier, p.const.WHEELBASE ...

    if nargin < 1 || isempty(jsonPath)
        here = fileparts(mfilename('fullpath'));
        jsonPath = fullfile(here, 'data', 'params.json');
    end
    raw = fileread(jsonPath);
    p = jsondecode(raw);

    % jsondecode 会把 true/false 变成 logical，这里统一成 double，
    % 因为 MATLAB Coder 对 logical 与 double 混用比较敏感。
    p = logical2double(p);

    need = {'control', 'planner', 'sensor', 'human', 'const'};
    for i = 1:numel(need)
        assert(isfield(p, need{i}), 'gb_params: params.json 缺少字段 %s', need{i});
    end
    assert(isfield(p.control, 'mu') && isfield(p.control, 'ttc_brake_s'), ...
        'gb_params: control 参数不完整');
end

function s = logical2double(s)
    if isstruct(s)
        f = fieldnames(s);
        for i = 1:numel(f)
            s.(f{i}) = logical2double(s.(f{i}));
        end
    elseif islogical(s)
        s = double(s);
    end
end

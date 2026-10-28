function result = run_online(scene_id, params_override, opts)
%RUN_ONLINE 【已并入主链路】转调 run.m
%
%   result = run_online(scene_id, params_override, opts)
%
%   本文件原是独立的"在线配对融合"链路，2026-10-28 起其全部能力已并入
%   唯一主链路 run.m（九阶段逐帧推进，见 docs/主链路在线化重构契约_20261028.md）。
%   保留本文件只为兼容既有调用脚本，功能上完全等价于：
%
%       result = run(scene_id, params_override, opts);
%
%   新代码请直接调用 run.m。

    if nargin < 1 || isempty(scene_id), scene_id = 'adsb_ns'; end
    if nargin < 2, params_override = []; end
    if nargin < 3 || isempty(opts), opts = struct(); end

    warning('run_online:deprecated', ...
        'run_online 已并入主链路 run.m，转发调用中；新代码请直接使用 run.m。');

    result = run(scene_id, params_override, opts);
end

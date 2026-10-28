function probe_merge()
%PROBE_MERGE 追踪重复系统航迹合并过程
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'tracker'), fullfile(root,'fusion'), fullfile(root,'config'), fullfile(root,'utils'));
    fid = fopen('results/_probe_merge.txt','w');
    p = struct('dt_sec',30,'online_gate_chi2',5.99,'online_M',3,'online_N',4, ...
        'online_K_loss',8,'online_min_life',1,'online_merge_confirm',2, ...
        'online_fusion_method','SCC');
    state = central_track_manager('init', p);
    for k = 1:8
        if k <= 3
            L1 = mk(11, 107.0);  L2 = [];
        elseif k <= 5
            L1 = mk(11, 107.0);  L2 = mk(22, 108.0);
        else
            L1 = mk(11, 107.0);  L2 = mk(22, 107.0);
        end
        s1 = snap(k, L1);  s2 = snap(k, L2);
        [state, log] = central_track_manager('step', state, s1, s2, k, p);
        fprintf(fid,'k=%d nCT=%d merge_cnt=%s :', k, numel(state.CT), mat2str([state.CT.merge_cnt]));
        for i = 1:numel(state.CT)
            fprintf(fid,' [id=%d s1=%d s2=%d mc=%d x=%.2f]', ...
                state.CT(i).id, state.CT(i).src1_id, state.CT(i).src2_id, ...
                state.CT(i).merge_cnt, state.CT(i).x(1));
        end
        fprintf(fid,'\n');
    end
    fclose(fid);
end

function L = mk(id, lon)
    L = struct('id', id, 'x', [lon;0;31.0;0], 'P', diag([1e-4,1e-8,1e-4,1e-8]), ...
        'life', 10, 'truth', 1);
end

function s = snap(k, L)
    if isempty(L)
        s = struct('frameID', k, 'trackList', {{}});
        return;
    end
    trk = struct('id', L.id, 'type', 1, 'lat', L.x(3), 'lon', L.x(1), ...
        'life', L.life, 'truth_idx', L.truth, 'ukf', struct('x', L.x, 'P', L.P));
    s = struct('frameID', k, 'trackList', {{trk}});
end

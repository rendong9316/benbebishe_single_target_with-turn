% 检查单站航迹结构里是否保留模型概率 mu / NIS 等诊断量
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root);
S = load(fullfile(root,'_audit','mc_current_geom.mat'));

fprintf('\n### 单站快照 trackList 字段 ###\n');
sn = S.trackSnapshots_R1{25}.trackList;
for t = 1:numel(sn)
    fprintf('--- track %d ---\n', t);
    disp(fieldnames(sn{t})');
    break;
end

fprintf('\n### 是否存在 mu / model_prob ###\n');
f = fieldnames(sn{1});
for i = 1:numel(f)
    if ~isempty(strfind(lower(f{i}), 'mu')) || ~isempty(strfind(lower(f{i}), 'prob')) || ...
       ~isempty(strfind(lower(f{i}), 'nis')) || ~isempty(strfind(lower(f{i}), 'model'))
        fprintf('  HIT: %s\n', f{i});
    end
end

fprintf('\n### 单站快照 trackList 第25帧内容摘要 ###\n');
for t = 1:numel(sn)
    s = sn{t};
    fprintf('id=%d type=%d truth_idx=%d lon=%.3f lat=%.3f', s.id, s.type, ...
        getfield_def(s,'truth_idx',-1), s.lon, s.lat);
    if isfield(s,'ukf') && isfield(s.ukf,'x')
        fprintf('  x=[%.3f %.5f %.3f %.5f]', s.ukf.x(1), s.ukf.x(2), s.ukf.x(3), s.ukf.x(4));
    end
    if isfield(s,'ukf') && isfield(s.ukf,'mu')
        fprintf('  mu=%s', mat2str(round(s.ukf.mu',3)));
    end
    fprintf('\n');
end

function v = getfield_def(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end

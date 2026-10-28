function probe_target_timeline(matfile, truth_idx, outfile)
%PROBE_TARGET_TIMELINE 打印某条真值在两站与系统航迹上的逐帧状态
    if nargin < 1 || isempty(matfile), matfile = 'results/online_fr24_opt/online_fr24_result.mat'; end
    if nargin < 3 || isempty(outfile), outfile = 'results/_probe_target_timeline.txt'; end
    S = load(matfile);
    r = S.r;
    fid = fopen(outfile, 'w');
    fprintf(fid, '目标真值编号 %d%s', truth_idx, newline);
    fprintf(fid, 'frame | R1(id,life) | R2(id,life) | CT(id,src1,src2,nsrc,cov)%s', newline);
    for k = 1:r.n_frames
        s1 = r.snap1s{k};  s2 = r.snap2s{k};  CTk = r.ctSnap{k};
        l1 = [];  l2 = [];
        if ~isempty(s1)
            for i = 1:numel(s1.trackList)
                t = s1.trackList{i};
                if isfield(t,'truth_idx') && isfinite(double(t.truth_idx)) && double(t.truth_idx) == truth_idx
                    l1 = [l1; double(t.id), double(t.life)]; %#ok<AGROW>
                end
            end
        end
        if ~isempty(s2)
            for i = 1:numel(s2.trackList)
                t = s2.trackList{i};
                if isfield(t,'truth_idx') && isfinite(double(t.truth_idx)) && double(t.truth_idx) == truth_idx
                    l2 = [l2; double(t.id), double(t.life)]; %#ok<AGROW>
                end
            end
        end
        cts = '';
        if ~isempty(CTk)
            for i = 1:numel(CTk)
                c = CTk(i);
                if isfinite(c.truth_idx) && c.truth_idx == truth_idx
                    cts = [cts sprintf(' CT(%d,%d,%d,%d)', c.id, c.src1_id, c.src2_id, c.n_src)]; %#ok<AGROW>
                end
            end
        end
        if isempty(l1) && isempty(l2) && isempty(cts), continue; end
        fprintf(fid, '%5d |', k);
        for i = 1:size(l1,1), fprintf(fid, ' R1(%d,life=%d)', l1(i,1), l1(i,2)); end
        for i = 1:size(l2,1), fprintf(fid, ' R2(%d,life=%d)', l2(i,1), l2(i,2)); end
        fprintf(fid, ' |%s%s', cts, newline);
    end
    fclose(fid);
end

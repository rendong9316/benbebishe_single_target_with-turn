function report = check_coverage_layout(tilt_list, verbose)
%CHECK_COVERAGE_LAYOUT 覆盖互补布设的几何核算（不做跟踪，只算覆盖）
%
%   report = check_coverage_layout(tilt_list, verbose)
%
%   对每条真值航迹逐帧判定两站是否覆盖，统计三段帧数：
%     仅南站 R2 / 两站共视 / 仅北站 R1
%   用于在跑完整在线流程之前，先把波束偏转角调到位。
%
%   输入：
%     tilt_list — 待试的偏转角（度）数组，缺省 [1.5 1.8 2.05 2.3 2.6]
%     verbose   — 是否打印每条航迹的明细，缺省 false
%
%   输出：
%     report — 结构体数组，每元素对应一个 tilt，含各段帧数的均值与分布

    if nargin < 1 || isempty(tilt_list), tilt_list = [1.5 1.8 2.05 2.3 2.6]; end
    if nargin < 2, verbose = false; end

    params = params_coverage_online();
    scenario = load_adsb_scenario('adsb_ns', params);
    truth_all = scenario.truth_all;
    t1 = scenario.t1_grid;
    n_frames = scenario.n_frames;

    fprintf('场景 %s：%d 条航迹，%d 帧%s', scenario.name, scenario.n_targets, n_frames, newline);

    report = struct('tilt', {}, 'n_covered', {}, 'r2only', {}, 'both', {}, ...
        'r1only', {}, 'single', {}, 'union', {}, 'ratio', {});

    for ti = 1:numel(tilt_list)
        tilt = tilt_list(ti);
        p = params;
        p.coverage_tilt_deg = tilt;
        [p.radar1_beam_center_deg, p.radar2_beam_center_deg] = centers(p);

        r2only = zeros(scenario.n_targets, 1);
        both = zeros(scenario.n_targets, 1);
        r1only = zeros(scenario.n_targets, 1);

        for ac = 1:scenario.n_targets
            tt = truth_all{ac};
            for k = 1:n_frames
                t = t1(k);
                if t < tt(1, 5) || t > tt(end, 5), continue; end
                lon = interp1(tt(:, 5), tt(:, 1), t);
                lat = interp1(tt(:, 5), tt(:, 2), t);
                in1 = radar_coverage_check(p.radar1_lon, p.radar1_lat, lon, lat, ...
                    p.radar1_beam_center_deg, p);
                in2 = radar_coverage_check(p.radar2_lon, p.radar2_lat, lon, lat, ...
                    p.radar2_beam_center_deg, p);
                if in1 && in2
                    both(ac) = both(ac) + 1;
                elseif in2
                    r2only(ac) = r2only(ac) + 1;
                elseif in1
                    r1only(ac) = r1only(ac) + 1;
                end
            end
        end

        single = max(r2only + both, r1only + both);
        union = r2only + both + r1only;
        covered = union > 0;

        report(ti).tilt = tilt;
        report(ti).n_covered = sum(covered);
        report(ti).r2only = mean(r2only(covered));
        report(ti).both = mean(both(covered));
        report(ti).r1only = mean(r1only(covered));
        report(ti).single = mean(single(covered));
        report(ti).union = mean(union(covered));
        report(ti).ratio = mean(union(covered)) / max(mean(single(covered)), eps);

        fprintf(['[tilt %.2f] 覆盖 %d/%d 条 | 仅R2 %.1f 帧, 共视 %.1f 帧, ' ...
            '仅R1 %.1f 帧 | 单站 %.1f, 并集 %.1f, 比值 %.2f%s'], ...
            tilt, report(ti).n_covered, scenario.n_targets, ...
            report(ti).r2only, report(ti).both, report(ti).r1only, ...
            report(ti).single, report(ti).union, report(ti).ratio, newline);

        if verbose
            for ac = 1:scenario.n_targets
                if ~covered(ac), continue; end
                fprintf('    %-12s R2only=%3d both=%3d R1only=%3d%s', ...
                    scenario.labels{ac}, r2only(ac), both(ac), r1only(ac), newline);
            end
        end
    end
end

function [c1, c2] = centers(params)
    az1 = skywave_geometry('azimuth', params.radar1_lon, params.radar1_lat, ...
        params.coverage_ref_lon, params.coverage_ref_lat);
    az2 = skywave_geometry('azimuth', params.radar2_lon, params.radar2_lat, ...
        params.coverage_ref_lon, params.coverage_ref_lat);
    c1 = az1 - params.coverage_tilt_deg;
    c2 = az2 + params.coverage_tilt_deg;
end

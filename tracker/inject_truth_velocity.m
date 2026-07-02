function inject_truth_velocity(ukf, tt_ac, t_grid, frame_id)
    % 从真值注入位置估计到 UKF 状态
    if isempty(t_grid) || frame_id >= length(t_grid), return; end
    tl = interp1(tt_ac(:,5), tt_ac(:,1), t_grid(frame_id), 'linear', 'extrap');
    tb = interp1(tt_ac(:,5), tt_ac(:,2), t_grid(frame_id), 'linear', 'extrap');
    % 注入位置
    ukf.x(3) = tb;  ukf.x(4) = tl;
    if isfield(ukf, 'ukf_cv')
        ukf.ukf_cv.x(3) = tb; ukf.ukf_cv.x(4) = tl;
        ukf.ukf_ct.x(3) = tb; ukf.ukf_ct.x(4) = tl;
    end
end

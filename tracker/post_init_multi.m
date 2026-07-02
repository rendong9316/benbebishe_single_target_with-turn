function ukf = post_init_multi(ukf, params)
    ukf.dt = params.dt_sec; ukf.initialized = true;
    if isfield(ukf, 'ukf_cv')
        ukf.ukf_cv.dt = params.dt_sec; ukf.ukf_cv.initialized = true;
        ukf.ukf_ct.dt = params.dt_sec; ukf.ukf_ct.initialized = true;
    end
    ukf.nis_history = [];
    if ~isfield(ukf, 'Q_base') || isempty(ukf.Q_base)
        if isfield(ukf, 'Q'), ukf.Q_base = ukf.Q; end
    end
    if ~isfield(ukf, 'Q_ema') || isempty(ukf.Q_ema), ukf.Q_ema = 1.0; end
end

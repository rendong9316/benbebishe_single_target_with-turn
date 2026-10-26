function smoke_pairing()
% 冒烟测试：单场景单种子运行 loader + 四法配对对比，验证链路无误
    repo_root = fileparts(fileparts(mfilename('fullpath')));
    addpath(repo_root);
    code_dirs = {'config','simulation','ukf','tracker','fusion','evaluation', ...
                 'utils','visualization','validation','experiments','io'};
    for cd_i = 1:numel(code_dirs)
        d_i = fullfile(repo_root, code_dirs{cd_i});
        if exist(d_i, 'dir'), addpath(d_i); end
    end
    params = simulation_params_oracle();
    params.random_seed = 101;
    params.imm_ct_omega_slow_deg = 0.05;
    params.imm_ct_omega_fast_deg = 0.2;
    fprintf('== smoke: adsb_0002 seed 101 (optimized model set) ==\n');
    S = load_aligned_snapshots('adsb_0002', params);
    cmp = compare_pairing_methods(S, true);
    fprintf('== smoke done ==\n');
end

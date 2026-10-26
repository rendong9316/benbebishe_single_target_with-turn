function v = get_param_local(params, name, default)
% 安全读取 params 字段，缺失时返回默认值
    v = default;
    if isfield(params, name)
        v = params.(name);
    end
end

function v = getp(s, name, default)
%GETP 安全读取参数字段
    v = default;
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    end
end

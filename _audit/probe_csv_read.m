% 诊断 readtable 读取 adsb_0608 的问题
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root);
p = fullfile(root,'data','adsb','adsb_0608.csv');
T = readtable(p, 'TextType', 'string');
fprintf('行数=%d\n', height(T));
disp(T.Properties.VariableNames);
fprintf('icao class=%s  seg class=%s  frame class=%s\n', class(T.icao), class(T.seg), class(T.frame));
T.icao = string(T.icao); T.seg = string(T.seg);
key = T.icao + "|" + T.seg;
fprintf('唯一 key=%d\n', numel(unique(key)));
fprintf('icao 唯一=%d\n', numel(unique(T.icao)));
% 检查是否有些 key 只有很少行
uk = unique(key);
cnt = zeros(numel(uk),1);
for i=1:numel(uk), cnt(i) = sum(key==uk(i)); end
fprintf('每组行数: 中位=%.0f min=%d max=%d  <10帧的组数=%d\n', median(cnt), min(cnt), max(cnt), sum(cnt<10));

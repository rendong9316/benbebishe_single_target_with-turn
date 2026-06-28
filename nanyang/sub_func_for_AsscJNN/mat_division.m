% ���ܣ���־���
% ԭ����֪�д�����⣬�д��Ŀ�꣬��һ�д���Ӳ���
% 1.ÿ�����ҽ���һ��1
% 2.���һ����ÿ��������1��1
% ���룺ȷ������
% ���������������*��*����
% Author: Peng Lei
% Date: 2023-01-05
function result_mat  = mat_division(input_mat)
output_index = zeros(1,size(input_mat,1)); 
ouput_vector = [];
for row = 1:1:size(input_mat,1) %�������������
    index = 0;%��¼ÿ�����������
    for column = 1:1:size(input_mat,2)
        if input_mat(row,column) == 0
            continue;
        end
        index = index + 1;
        vector_index = zeros(1,size(input_mat,2));
        vector_index(column) = 1;
        ouput_vector = [ouput_vector;vector_index];
    end
    output_index(row) = index; 
end

%������ɾ���
num = prod(output_index);
result_mat = zeros(size(input_mat,1),size(input_mat,2),num);
a = 1;
index = 0;
while(a <= size(output_index,2))
    if a< size(output_index,2)
   re_test = prod(output_index(a+1:size(output_index,2)));
    else
        re_test  =  1;
    end
    num_test = num/re_test;
    index_test = output_index(a);
    for nn = 1:num_test
            for ii = 1:re_test
                tt = mod(nn,index_test);
                if tt == 0
                    tt = index_test;
                end
                result_mat(a,:,(nn-1)*re_test+ii) = ouput_vector(index+tt,:);
            end
    end
    a = a+1;
    index = index +index_test;
end

%���ԭ��2ɸѡ����õ���������
delete = [];
rr = 1;
while(rr <= num)
    test = sum(result_mat(:,:,rr),1);
    test(1) = [];
    if(~isempty(find(test>1)))
        delete = [delete,rr];
    end
    rr = rr+1;
end
result_mat(:,:,delete) = [];
end
function out_str = num2str_norm( num, normSize )
% This function is to normalize a number to be a normed string
% with fixed length and with sign

% normSize is the length of digits in num.
% sign of num is not counted as a digit

sign = '';
abs_num = num;
if num < 0
    sign = '-';
    abs_num = -num;
else
    sign = '+';
end

num_str = num2str(num);
num_len = length(num_str);
out_str = num_str;
if num_len < normSize
    gap = normSize - num_len;
    for i=1:1:gap
        out_str = strcat('0', out_str);
    end
end

out_str = strcat(sign, out_str);

end


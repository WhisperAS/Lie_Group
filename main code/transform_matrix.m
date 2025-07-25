function T = transform_matrix(R, p)
% 构造齐次变换矩阵
% 输入: R - 3x3旋转矩阵, p - 3x1位置向量
% 输出: T - 4x4齐次变换矩阵

if nargin < 2
    p = [0; 0; 0];
end

T = [R, p;
     0, 0, 0, 1];
end

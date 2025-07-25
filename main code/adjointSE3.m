function Ad = adjointSE3(T)
% adjointSE3 计算SE(3)元素a的伴随矩阵
% 对应2010年Ruibo He论文式（49）
% 输入：
%   T - 4x4齐次变换矩阵，T = [R, b; 0 0 0 1]
%
% 输出：
%   Ad - 6x6伴随矩阵

    R = T(1:3,1:3);
    b = T(1:3,4);
    b_hat = hat_so3(b);

    Ad = [R, zeros(3,3);
          b_hat*R, R];
end
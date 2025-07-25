function Ad_inv = adjointSE3_inv(T)
% adjointSE3_inv 计算SE(3)元素a的伴随矩阵的逆
% 对应2010年Ruibo He论文式（50）
% 输入：
%   T - 4x4齐次变换矩阵，T = [R, b; 0 0 0 1]
%
% 输出：
%   Ad_inv - 6x6伴随矩阵的逆

    R = T(1:3,1:3);
    b = T(1:3,4);
    b_hat = hat_so3(b);

    RT = R';
    Ad_inv = [RT, zeros(3,3);
             -RT*b_hat, RT];
end
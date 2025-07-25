function ad_xi = ad_se3(xi)
% ad_se3 计算se(3)元素的伴随表示（小ad算子）
% 对应对应2010年Ruibo He论文式（51）
% 输入：
%   xi - 6x1运动旋量，格式为 [omega; v]
%        omega - 3x1旋转分量
%        v     - 3x1平移分量
%
% 输出：
%   ad_xi - 6x6伴随表示矩阵

    omega = xi(1:3);
    v = xi(4:6);

    omega_hat = hat_so3(omega);
    v_hat = hat_so3(v);

    ad_xi = [omega_hat, zeros(3,3);
             v_hat,    omega_hat];
end
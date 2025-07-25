function xi_hat = hat_se3(xi)
% hat_se3 将6x1运动旋量xi转换为4x4李代数矩阵（se(3)元素）
%
% 输入：
%   xi - 6x1运动旋量，格式为 [omega; v]
%        omega - 3x1旋转分量
%        v     - 3x1平移分量
%
% 输出：
%   xi_hat - 4x4李代数矩阵
%
% 公式参考论文式(14),(24),(25)

    omega = xi(1:3);
    v = xi(4:6);
    omega_hat = hat_so3(omega);
    xi_hat = [omega_hat, v;
              0, 0, 0, 0];
end

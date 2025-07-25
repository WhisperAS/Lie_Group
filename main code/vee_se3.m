function xi = vee_se3(xi_hat)
% vee_se3 将4x4李代数矩阵（se(3)元素）转换为6x1运动旋量向量
% hat的反操作
% 输入：
%   xi_hat - 4x4李代数矩阵，形式为
%            [ hat(omega)  v;
%              0           0 ]
%
% 输出：
%   xi - 6x1运动旋量向量 [omega; v]
%        omega为3x1旋转分量，v为3x1平移分量

    % 提取旋转部分的反对称矩阵
    omega_hat = xi_hat(1:3,1:3);
    % 提取平移部分向量
    v = xi_hat(1:3,4);

    % 反对称矩阵转旋转向量（vee操作）
    omega = [omega_hat(3,2); omega_hat(1,3); omega_hat(2,1)];

    % 组合成6x1运动旋量
    xi = [omega; v];
end

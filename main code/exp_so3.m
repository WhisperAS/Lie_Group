function R = exp_so3(omega, q)
% exp_so3_axis_angle 计算SO(3)的指数映射，基于旋转轴和角度
% 对应论文2010 Ruibo He中的式（44）
% 输入：
%   omega - 3x1旋转轴向量（不要求归一化）
%   q     - 旋转角度或关节变量（标量）
%
% 输出：
%   R - 3x3旋转矩阵

    omega_norm = norm(omega);
    if omega_norm < 1e-12
        R = eye(3);
        return;
    end
    theta = omega_norm * q;
    omega_hat = hat_so3(omega);

    R = eye(3) + (sin(theta)/omega_norm) * omega_hat + ...
        ((1 - cos(theta))/omega_norm^2) * (omega_hat * omega_hat);
end
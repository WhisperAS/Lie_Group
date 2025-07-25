function T = exp_se3_2(S)
% exp_se3_2 计算SE(3)李代数4x4矩阵的指数映射
% 对应知乎文章：《基于指数积方法的串联机器人标定数学模型》中的式（16）
% 输入：
%   S - 4x4李代数矩阵（se(3)元素）
%       S = [hat(omega), v; 0 0 0 0]
%       其中 hat(omega)为3x3反对称矩阵，v为3x1平移分量
%
% 输出：
%   T - 4x4齐次变换矩阵（SE(3)群元素）
%
% 公式参考：e^S = I_4 + S + 1/theta^2*(1-cos(theta))*S^2 + 1/theta^3*(theta-sin(theta))*S^3
% 其中 theta^2 = omega_x^2 + omega_y^2 + omega_z^2

    % 提取旋转分量
    omega_hat = S(1:3,1:3);
    omega = [omega_hat(3,2); omega_hat(1,3); omega_hat(2,1)];
    theta = norm(omega);

    if theta < 1e-12
        % 小角度近似
        T = eye(4) + S;
    else
        S2 = S * S;
        S3 = S2 * S;
        T = eye(4) + S + (1/theta^2)*(1 - cos(theta))*S2 + (1/theta^3)*(theta - sin(theta))*S3;
    end
end

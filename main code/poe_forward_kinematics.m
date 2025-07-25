function g = poe_forward_kinematics(xis, qs, xi_st)
% poe_forward_kinematics 基于POE公式计算串联机器人末端位姿
%
% 输入：
%   xis   - 6xn矩阵，每列为第i个关节的运动旋量xi_i = [v; omega]
%   qs    - nx1关节变量向量（转角或位移）
%   xi_st - 6x1末端初始旋量
%
% 输出：
%   g - 4x4末端齐次变换矩阵

    n = length(qs);
    g = eye(4);
    for i = 1:n
        xi_hat = hat_se3(xis(:,i));
        g = g * expm(xi_hat * qs(i));
    end
    g = g * exp_se3(xi_st);
end

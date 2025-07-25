function xi = log_se3(T)
% log_se3_matrix 计算SE(3)齐次变换矩阵的对数映射，返回6x1运动旋量 [omega; v]
% 对应2010年Ruibo He论文式（46）-（48）
% 输入：
%   T - 4x4齐次变换矩阵 [R, b; 0, 1]
%
% 输出：
%   xi - 6x1运动旋量，格式为 [omega; v]
%        omega - 3x1旋转分量
%        v     - 3x1平移分量

    R = T(1:3,1:3);
    b = T(1:3,4);

    % 旋转部分对数映射
    theta = acos( min(max((trace(R)-1)/2, -1), 1) );
    if abs(theta) < 1e-12
        omega_hat = zeros(3,3);
        omega = zeros(3,1);
    else
        omega_hat = (theta/(2*sin(theta))) * (R - R');
        omega = [omega_hat(3,2); omega_hat(1,3); omega_hat(2,1)];
    end

    % M^{-1}计算
    if abs(theta) < 1e-12
        Minv = eye(3);
    else
        omega_hat_sq = omega_hat * omega_hat;
        omega_norm = theta;
        c1 = (2*sin(omega_norm) - omega_norm*(1+cos(omega_norm))) / (2*omega_norm^2*sin(omega_norm));
        Minv = eye(3) - 0.5*omega_hat + c1*omega_hat_sq;
    end

    % 平移部分对数映射
    v = Minv * b;

    % 输出6x1运动旋量 [omega; v]
    xi = [omega; v];
end
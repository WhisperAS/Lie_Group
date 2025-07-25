function omega_hat = log_so3(R)
% log_so3 计算SO(3)旋转矩阵的对数映射，返回so(3)的3x3反对称矩阵
% 对应知乎文章：《基于指数积方法的串联机器人标定数学模型》中的式（12），并补充了theta为0的情况
% 输入：
%   R - 3x3旋转矩阵
%
% 输出：
%   omega_hat - 3x3反对称矩阵（so(3)元素）
%
% 公式参考你提供的图片

    theta = acos( min(max((trace(R) - 1)/2, -1), 1) ); % 防止数值误差
    if abs(theta) < 1e-12
        omega_hat = zeros(3,3);
    else
        omega_hat = (theta / (2*sin(theta))) * (R - R');
    end
end

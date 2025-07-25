function omega = vee_so3(omega_hat)
% vee_so3 SO(3)的vee操作，将3x3反对称矩阵映射为3x1旋转向量
%
% 输入：
%   omega_hat - 3x3反对称矩阵，满足 omega_hat' = -omega_hat
%
% 输出：
%   omega - 3x1旋转向量

    % 验证输入是否为3x3矩阵
    assert(all(size(omega_hat) == [3, 3]), 'Input must be a 3x3 matrix');
    % 验证是否反对称
    assert(norm(omega_hat + omega_hat', 'fro') < 1e-10, 'Input matrix must be skew-symmetric');

    omega = [omega_hat(3,2); omega_hat(1,3); omega_hat(2,1)];
end

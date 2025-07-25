% 验证脚本
% 生成随机旋转轴（单位向量）
omega = randn(3,1);
omega = omega / norm(omega);

% 旋转角度
q = pi/4; % 45度

% 计算旋转矩阵
R = exp_so3(omega, q);

% 计算对数映射（反对称矩阵）
omega_hat_rec = log_so3(R);

% 将反对称矩阵映射回旋转向量
omega_rec = vee_so3(omega_hat_rec);

% 原始旋转向量（旋转轴乘以角度）
omega_true = omega * q;

% 显示结果
disp('原始旋转向量 omega_true:');
disp(omega_true);

disp('恢复旋转向量 omega_rec:');
disp(omega_rec);

% 计算误差
error = norm(omega_true - omega_rec);
fprintf('恢复误差: %.3e\n', error);

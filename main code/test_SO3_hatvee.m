%% 验证脚本
% 随机生成旋转向量
omega = randn(3,1);

% 计算hat映射
omega_hat = hat_so3(omega);

% 计算vee映射
omega_recovered = vee_so3(omega_hat);

% 显示结果
disp('原始旋转向量 omega:');
disp(omega);
disp('hat(omega) 反对称矩阵 omega_hat:');
disp(omega_hat);
disp('vee(omega_hat) 恢复的旋转向量 omega_recovered:');
disp(omega_recovered);

% 验证误差
error = norm(omega - omega_recovered);
fprintf('恢复误差: %.3e\n', error);
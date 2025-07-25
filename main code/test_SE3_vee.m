% 测试脚本 test_vee_se3.m

% 定义旋转分量 omega 和平移分量 v
omega = [0.1; -0.2; 0.3];
v = [1.0; 2.0; 3.0];

% 组合成6x1运动旋量 xi
xi = [omega; v];

% 调用hat_se3函数，得到4x4李代数矩阵
xi_hat = hat_se3(xi);

% 调用vee_se3函数，将4x4矩阵映射回6x1运动旋量
xi_recovered = vee_se3(xi_hat);

% 显示结果
disp('原始运动旋量 xi = ');
disp(xi);

disp('hat_se3计算结果 xi_hat = ');
disp(xi_hat);

disp('vee_se3恢复的运动旋量 xi_recovered = ');
disp(xi_recovered);

% 计算误差
error = norm(xi - xi_recovered);
fprintf('恢复误差: %.3e\n', error);
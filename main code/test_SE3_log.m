% 定义旋转轴角和位移
axis = [0 0 1];
angle = pi/4; % 45度
R = axang2rotm([axis angle]);
t = [1; 2; 3];

% 构造齐次变换矩阵
T = [R t; 0 0 0 1];

% 使用MATLAB内置logm计算对数（4x4李代数矩阵）
S_logm = logm(T);

% 使用自定义对数映射函数，返回6x1旋量
xi_custom = log_se3(T);

% 从S_logm提取旋转部分和位移部分
omega_hat_logm = S_logm(1:3,1:3);
v_logm = S_logm(1:3,4);

% 将旋转部分反对称矩阵转换为旋转向量
omega_logm = [omega_hat_logm(3,2); omega_hat_logm(1,3); omega_hat_logm(2,1)];

% 显示结果对比
disp('MATLAB logm旋转向量 omega_logm:');
disp(omega_logm);
disp('MATLAB logm平移向量 v_logm:');
disp(v_logm);

disp('自定义函数旋量 xi_custom:');
disp(xi_custom);

% 计算旋转和平移误差
rot_error = norm(omega_logm - xi_custom(1:3));
trans_error = norm(v_logm - xi_custom(4:6));

fprintf('旋转向量误差: %.3e\n', rot_error);
fprintf('平移向量误差: %.3e\n', trans_error);
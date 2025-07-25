% 测试脚本：验证自定义SE(3)指数映射与expm函数结果一致性

% 定义旋转向量omega和位移向量v
omega = [0.1; 0.2; 0.3];
v = [1; 2; 3];

% 计算omega的反对称矩阵
omega_hat = [  0       -omega(3)  omega(2);
             omega(3)    0      -omega(1);
            -omega(2)  omega(1)    0    ];

% 构造4x4李代数矩阵S
S = [omega_hat, v; 0 0 0 0];

% 调用自定义的指数映射函数
T_custom = exp_se3_matrix(S);

% 调用MATLAB Robotics System Toolbox的expm函数
T_expm = expm(S);

% 显示两个结果
disp('自定义指数映射结果 T_custom = ');
disp(T_custom);

disp('MATLAB expm函数结果 T_expm = ');
disp(T_expm);

% 计算两个矩阵的差异范数
diff_norm = norm(T_custom - T_expm, 'fro');
fprintf('两个矩阵差异的Frobenius范数: %.3e\n', diff_norm);

% 判断是否足够接近
tolerance = 1e-12;
if diff_norm < tolerance
    disp('测试通过：自定义指数映射与expm函数结果高度一致。');
else
    disp('测试未通过：两者结果存在较大差异。');
end

% --- 自定义指数映射函数 ---
function T = exp_se3_matrix(S)
    omega_hat = S(1:3,1:3);
    omega = [omega_hat(3,2); omega_hat(1,3); omega_hat(2,1)];
    theta = norm(omega);

    if theta < 1e-12
        T = eye(4) + S;
    else
        S2 = S * S;
        S3 = S2 * S;
        T = eye(4) + S + (1/theta^2)*(1 - cos(theta))*S2 + (1/theta^3)*(theta - sin(theta))*S3;
    end
end

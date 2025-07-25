% 测试脚本
% 定义旋转分量 omega 和平移分量 v
omega = [0.1; -0.2; 0.3];
v = [1.0; 2.0; 3.0];

% 组合成6x1运动旋量 xi
xi = [omega; v];

% 调用hat_se3函数
xi_hat = hat_se3(xi);

% 手动计算期望的xi_hat
omega_hat = hat_so3(omega);
xi_hat_expected = [omega_hat, v; 0 0 0 0];

% 显示结果
disp('hat_se3计算结果 xi_hat = ');
disp(xi_hat);

disp('期望结果 xi_hat_expected = ');
disp(xi_hat_expected);

% 验证两者是否相等（允许一定数值误差）
tolerance = 1e-12;
if norm(xi_hat - xi_hat_expected, 'fro') < tolerance
    disp('测试通过：hat_se3函数输出正确。');
else
    disp('测试失败：hat_se3函数输出与期望不符。');
end


% --- 需要你已定义的hat_so3函数 ---
function omega_hat = hat_so3(omega)
    omega_hat = [  0       -omega(3)  omega(2);
                 omega(3)    0      -omega(1);
                -omega(2)  omega(1)    0    ];
end

% --- 你的hat_se3函数 ---
function xi_hat = hat_se3(xi)
    omega = xi(1:3);
    v = xi(4:6);
    omega_hat = hat_so3(omega);
    xi_hat = [omega_hat, v;
              0, 0, 0, 0];
end

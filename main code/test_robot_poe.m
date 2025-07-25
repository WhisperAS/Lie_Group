% test_robot_poe.m
% 串联机器人指数积公式和雅可比矩阵测试脚本

clear; clc; close all;

fprintf('===== 串联机器人指数积公式测试 =====\n\n');

%% 1. 定义测试机器人参数 (3关节旋转机器人)
% 机器人DH参数或几何参数
L1 = 1.0;  % 连杆1长度
L2 = 0.8;  % 连杆2长度  
L3 = 0.6;  % 连杆3长度

% 定义固定变换链 g_chain
% g_chain{i} 表示从坐标系{i-1}到{i}的变换
g_chain = cell(1, 4);

% 基座到关节1
g_chain{1} = [1 0 0 0;
              0 1 0 0;
              0 0 1 L1;
              0 0 0 1];

% 关节1到关节2              
g_chain{2} = [1 0 0 L2;
              0 1 0 0;
              0 0 1 0;
              0 0 0 1];

% 关节2到关节3
g_chain{3} = [1 0 0 L3;
              0 1 0 0;
              0 0 1 0;
              0 0 0 1];

% 关节3到末端执行器
g_chain{4} = [1 0 0 0.3;
              0 1 0 0;
              0 0 1 0;
              0 0 0 1];

%% 2. 定义局部运动旋量 xi_local
% 假设所有关节都是绕z轴旋转
xi_local = [0 0 0;    % wx
            0 0 0;    % wy  
            1 1 1;    % wz
            0 0 0;    % vx
            0 0 0;    % vy
            0 0 0];   % vz

%% 3. 定义测试关节角度
theta_test = [pi/6; pi/4; pi/3];  % 30°, 45°, 60°
n_joints = length(theta_test);

fprintf('测试机器人参数:\n');
fprintf('关节数量: %d\n', n_joints);
fprintf('连杆长度: L1=%.1f, L2=%.1f, L3=%.1f\n', L1, L2, L3);
fprintf('测试关节角度: [%.2f, %.2f, %.2f] (弧度)\n\n', theta_test);

%% 4. 测试局部指数积正运动学
fprintf('=== 测试1: 局部指数积正运动学 ===\n');
try
    T_local = FK_local_poe(g_chain, xi_local, theta_test);
    fprintf('✓ FK_local_poe 执行成功\n');
    fprintf('末端执行器位置: [%.3f, %.3f, %.3f]\n', T_local(1:3,4));
    
    % 提取姿态角
    R_local = T_local(1:3, 1:3);
    fprintf('旋转矩阵行列式: %.6f (应接近1)\n', det(R_local));
    fprintf('旋转矩阵正交性检查: %.2e (应接近0)\n', norm(R_local'*R_local - eye(3)));
    
catch ME
    fprintf('✗ FK_local_poe 执行失败: %s\n', ME.message);
end

%% 5. 计算全局运动旋量并测试全局指数积
fprintf('\n=== 测试2: 局部到全局旋量转换和雅可比矩阵 ===\n');
try
    % 重要修改：使用零位形计算全局旋量（固定不变）
    theta_zero = zeros(n_joints, 1);
    [Js_zero, xi_global_fixed] = local_to_global_jacobian(g_chain, xi_local, theta_zero);
    
    fprintf('✓ local_to_global_jacobian 执行成功\n');
    fprintf('零位形空间雅可比矩阵维度: %dx%d\n', size(Js_zero));
    fprintf('固定全局旋量列表维度: %dx%d\n', size(xi_global_fixed));
    
    % 显示零位形下的全局旋量（这些在整个运动中保持不变）
    fprintf('\n零位形下的全局运动旋量（固定）:\n');
    for i = 1:n_joints
        fprintf('关节%d: [%.3f %.3f %.3f %.3f %.3f %.3f]ᵀ\n', i, xi_global_fixed(:,i));
    end
    
    % 同时计算当前角度下的雅可比矩阵（用于比较）
    [Js_current, xi_global_current] = local_to_global_jacobian(g_chain, xi_local, theta_test);
    fprintf('\n当前角度下的空间雅可比矩阵维度: %dx%d\n', size(Js_current));
    
catch ME
    fprintf('✗ local_to_global_jacobian 执行失败: %s\n', ME.message);
    return;
end

%% 6. 计算零位形变换矩阵M (用于全局指数积)
fprintf('\n=== 测试3: 计算零位形变换矩阵 ===\n');
% M是所有关节角度为零时的末端执行器位姿
theta_zero = zeros(n_joints, 1);
M = FK_local_poe(g_chain, xi_local, theta_zero);
fprintf('零位形末端执行器位置: [%.3f, %.3f, %.3f]\n', M(1:3,4));

%% 7. 测试全局指数积正运动学
fprintf('\n=== 测试4: 全局指数积正运动学 ===\n');
try
    % 修改：使用固定的全局旋量（零位形下计算的）
    T_global = FK_global_poe(xi_global_fixed, theta_test, M);
    fprintf('✓ FK_global_poe 执行成功\n');
    fprintf('末端执行器位置: [%.3f, %.3f, %.3f]\n', T_global(1:3,4));
    
catch ME
    fprintf('✗ FK_global_poe 执行失败: %s\n', ME.message);
end

%% 8. 验证局部与全局指数积结果的一致性
fprintf('\n=== 测试5: 局部与全局指数积一致性验证 ===\n');
if exist('T_local', 'var') && exist('T_global', 'var')
    position_error = norm(T_local(1:3,4) - T_global(1:3,4));
    rotation_error = norm(T_local(1:3,1:3) - T_global(1:3,1:3), 'fro');
    
    fprintf('位置误差: %.2e\n', position_error);
    fprintf('旋转矩阵误差: %.2e\n', rotation_error);
    
    if position_error < 1e-10 && rotation_error < 1e-10
        fprintf('✓ 局部与全局指数积结果一致\n');
    else
        fprintf('✗ 局部与全局指数积结果不一致\n');
    end
else
    fprintf('无法进行一致性验证（前面的计算失败）\n');
end

%% 9. 多组关节角度测试
fprintf('\n=== 测试6: 多组关节角度测试 ===\n');
test_angles = [
    0,    0,    0;     % 零位形
    pi/2, 0,    0;     % 关节1转90度
    0,    pi/2, 0;     % 关节2转90度  
    0,    0,    pi/2;  % 关节3转90度
    pi/4, pi/4, pi/4   % 均匀分布
];

fprintf('测试不同关节角度组合的一致性:\n');
for i = 1:size(test_angles, 1)
    theta_i = test_angles(i, :)';
    
    try
        % 局部指数积
        T_local_i = FK_local_poe(g_chain, xi_local, theta_i);
        
        % 修改：始终使用固定的全局旋量（零位形下的）
        T_global_i = FK_global_poe(xi_global_fixed, theta_i, M);
        
        % 比较结果
        pos_err = norm(T_local_i(1:3,4) - T_global_i(1:3,4));
        rot_err = norm(T_local_i(1:3,1:3) - T_global_i(1:3,1:3), 'fro');
        
        fprintf('测试%d: θ=[%.2f,%.2f,%.2f], 位置误差=%.2e, 旋转误差=%.2e', ...
                i, theta_i(1), theta_i(2), theta_i(3), pos_err, rot_err);
        
        if pos_err < 1e-10 && rot_err < 1e-10
            fprintf(' ✓\n');
        else
            fprintf(' ✗\n');
        end
        
    catch ME
        fprintf('测试%d失败: %s\n', i, ME.message);
    end
end

%% 10. 雅可比矩阵性质检验
fprintf('\n=== 测试7: 雅可比矩阵性质检验 ===\n');
if exist('Js', 'var')
    fprintf('雅可比矩阵条件数: %.2f\n', cond(Js));
    
    % 检查雅可比矩阵的奇异性
    singular_values = svd(Js);
    fprintf('奇异值: [');
    fprintf('%.4f ', singular_values);
    fprintf(']\n');
    
    min_sv = min(singular_values);
    if min_sv < 1e-6
        fprintf('⚠ 警告: 机器人可能处于奇异位形 (最小奇异值=%.2e)\n', min_sv);
    else
        fprintf('✓ 机器人处于非奇异位形\n');
    end
end

%% 新增测试：验证全局旋量的固定性概念
fprintf('\n=== 测试8: 验证全局旋量固定性概念 ===\n');
fprintf('全局指数积中的旋量应该在零位形下计算，并在整个运动中保持不变\n');

% 比较零位形和当前角度下计算的全局旋量
diff_xi = norm(xi_global_fixed - xi_global_current, 'fro');
fprintf('零位形与当前角度全局旋量差异: %.4f\n', diff_xi);

if diff_xi > 1e-10
    fprintf('✓ 正确：零位形和当前角度的全局旋量不同\n');
    fprintf('  这证明了应该使用零位形的固定全局旋量\n');
else
    fprintf('⚠ 注意：零位形和当前角度的全局旋量相同\n');
    fprintf('  这可能表明当前测试角度较小或机器人结构特殊\n');
end

fprintf('\n零位形全局旋量（正确用于全局指数积）:\n');
for i = 1:n_joints
    fprintf('ξ%d = [%.4f %.4f %.4f %.4f %.4f %.4f]ᵀ\n', i, xi_global_fixed(:,i));
end

fprintf('\n当前角度全局旋量（不应用于全局指数积）:\n');
for i = 1:n_joints
    fprintf('ξ%d = [%.4f %.4f %.4f %.4f %.4f %.4f]ᵀ\n', i, xi_global_current(:,i));
end

%% 11. 可视化结果（可选）
fprintf('\n=== 测试完成 ===\n');
if exist('T_local', 'var')
    fprintf('\n最终结果总结:\n');
    fprintf('末端执行器位置: [%.3f, %.3f, %.3f]\n', T_local(1:3,4));
    fprintf('末端执行器姿态矩阵:\n');
    disp(T_local(1:3,1:3));
end

fprintf('\n所有测试完成！\n');

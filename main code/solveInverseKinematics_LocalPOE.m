function [theta_solution, converged, info] = solveInverseKinematics_LocalPOE(target_pose, initial_guess, robot_params, solver_params)
% 基于局部POE方法的逆运动学求解器
%
% 输入:
%   target_pose: 4x4 目标齐次变换矩阵
%   initial_guess: 6x1 初始关节角度猜测 [theta1, theta2, d3, theta4, theta5, theta6]
%   robot_params: 机器人参数结构体
%   solver_params: 求解器参数结构体
%
% 输出:
%   theta_solution: 6x1 求解得到的关节角度
%   converged: 逻辑值，是否收敛
%   info: 求解信息结构体

    % 参数检查和默认值设置
    if nargin < 4
        solver_params = getDefaultSolverParams();
    end
    if nargin < 3
        robot_params = getDefaultRobotParams_Local();
    end
    
    % 提取求解器参数
    max_iterations = solver_params.max_iterations;
    tolerance = solver_params.tolerance;
    lambda_init = solver_params.damping_factor;
    step_size = solver_params.step_size;
    
    % 初始化
    theta = initial_guess;
    converged = false;
    lambda = lambda_init;
    prev_error = inf;
    stagnation_count = 0;
    
    % 存储迭代信息
    error_history = zeros(max_iterations, 1);
    
    for iter = 1:max_iterations
        % 使用局部POE计算当前正运动学和雅可比矩阵
        [current_T, J_spatial] = computeForwardKinematicsWithJacobian_LocalPOE(theta, robot_params);
        
        % 计算位姿误差
        pose_error = computePoseError(target_pose, current_T);
        current_error = norm(pose_error);
        error_history(iter) = current_error;
        
        % 检查收敛
        if current_error < tolerance
            converged = true;
            fprintf('收敛成功！迭代次数: %d, 最终误差: %.8f\n', iter, current_error);
            break;
        end
        
        % 检查停滞
        if abs(prev_error - current_error) < tolerance * 0.01
            stagnation_count = stagnation_count + 1;
        else
            stagnation_count = 0;
        end
        
        % 自适应阻尼调整
        cond_J = cond(J_spatial);
        if cond_J > 1e8 || stagnation_count > 3
            lambda = lambda * 2;
        elseif stagnation_count == 0 && lambda > lambda_init
            lambda = lambda * 0.5;
        end
        
        % 求解更新步长
        try
            if cond_J > 1e6
                % 使用阻尼最小二乘法
                H = J_spatial' * J_spatial + lambda * eye(size(J_spatial, 2));
                delta_theta = H \ (J_spatial' * pose_error);
            else
                % 标准最小二乘法
                delta_theta = J_spatial \ pose_error;
            end
        catch
            % 备用方案：伪逆
            delta_theta = pinv(J_spatial) * pose_error;
        end
        
        % 自适应步长控制
        step_norm = norm(delta_theta);
        if step_norm > solver_params.max_step_size
            delta_theta = delta_theta * (solver_params.max_step_size / step_norm);
        end
        
        % 更新关节角度
        theta_new = theta + step_size * delta_theta;
        theta_new = constrainJointLimits(theta_new, robot_params);
        
        % 线搜索：检查更新是否改善误差
        [test_T, ~] = computeForwardKinematicsWithJacobian_LocalPOE(theta_new, robot_params);
        test_error_vec = computePoseError(target_pose, test_T);
        test_error = norm(test_error_vec);
        
        if test_error < current_error * 1.01  % 允许轻微增加
            theta = theta_new;
            step_size = min(step_size * 1.05, 1.0);  % 增加步长
        else
            step_size = step_size * 0.7;  % 减小步长
            if step_size < 1e-6
                warning('步长过小，算法可能陷入局部最小值');
                break;
            end
        end
        
        prev_error = current_error;
        
        % 输出迭代信息
        if mod(iter, 20) == 0 || iter <= 5
            fprintf('迭代 %3d: 误差=%.6e, 条件数=%.2e, 阻尼=%.2e, 步长=%.4f\n', ...
                iter, current_error, cond_J, lambda, step_size);
        end
    end
    
    if ~converged
        warning('未收敛！达到最大迭代次数，最终误差: %.6e', current_error);
    end
    
    theta_solution = theta;
    
    % 返回求解信息
    info.iterations = iter;
    info.final_error = current_error;
    info.error_history = error_history(1:iter);
    info.jacobian_condition = cond(J_spatial);
    info.final_step_size = step_size;
end

function [T, J_spatial] = computeForwardKinematicsWithJacobian_LocalPOE(theta, params)
% 使用局部POE方法计算正运动学和空间雅可比矩阵
%
% 输入:
%   theta: 6x1 关节角度向量
%   params: 机器人参数结构体
%
% 输出:
%   T: 4x4 末端执行器齐次变换矩阵
%   J_spatial: 6x6 空间雅可比矩阵

    % 提取机器人参数
    xi_local = params.xi_local;  % 6x6 矩阵，每列是一个关节的局部旋量
    M = params.M;  % 4x4 末端执行器在零位时的变换矩阵
    
    % 使用您的FK_local_poe函数计算正运动学
    T = FK_local_poe(xi_local, theta, M);
    
    % 计算变换链（从基座到每个关节）
    g_chain = computeTransformChain(xi_local, theta, M);
    
    % 使用您的local_to_global_jacobian函数计算雅可比矩阵
    J_spatial = local_to_global_jacobian(g_chain, xi_local, theta);
end

function g_chain = computeTransformChain(xi_local, theta, M)
% 计算从基座到每个关节和末端执行器的变换链
%
% 输入:
%   xi_local: 6x6 局部旋量矩阵
%   theta: 6x1 关节角度向量
%   M: 4x4 末端执行器零位变换矩阵
%
% 输出:
%   g_chain: (4x4)x(n+1) 变换链，包含从基座到每个关节的变换

    n_joints = length(theta);
    g_chain = zeros(4, 4, n_joints + 1);
    
    % 基座变换（单位矩阵）
    g_chain(:,:,1) = eye(4);
    
    % 累积计算每个关节的变换
    current_T = eye(4);
    for i = 1:n_joints
        % 使用您的exp_se3函数计算关节变换
        joint_twist = xi_local(:, i);
        joint_transform = exp_se3(joint_twist * theta(i));
        
        % 累积变换
        current_T = current_T * joint_transform;
        g_chain(:,:,i+1) = current_T;
    end
    
    % 最后乘以末端执行器变换
    g_chain(:,:,end) = current_T * M;
end

function pose_error = computePoseError(T_target, T_current)
% 计算位姿误差（以旋量形式表示）
%
% 输入:
%   T_target: 4x4 目标齐次变换矩阵
%   T_current: 4x4 当前齐次变换矩阵
%
% 输出:
%   pose_error: 6x1 位姿误差向量 [旋转误差; 平移误差]

    % 计算相对变换矩阵
    T_error = inv(T_current) * T_target;
    
    % 使用您的log_se3函数直接获得位姿误差向量
    pose_error = log_se3(T_error);
end

function theta_constrained = constrainJointLimits(theta, params)
% 关节限制约束
%
% 输入:
%   theta: 6x1 关节角度向量
%   params: 机器人参数结构体
%
% 输出:
%   theta_constrained: 6x1 约束后的关节角度向量

    if isfield(params, 'joint_limits')
        joint_limits = params.joint_limits;
        theta_constrained = max(min(theta, joint_limits(:,2)), joint_limits(:,1));
    else
        theta_constrained = theta;
    end
end

function params = getDefaultRobotParams_Local()
% 默认机器人参数（局部POE版本）
    % 机器人几何参数
    l0 = 500e-3;  % 500mm
    l1 = 200e-3;  % 200mm
    
    % 局部关节旋量（在各自关节坐标系中表示）
    % 对于旋转关节，局部旋量通常是 [0;0;1;0;0;0] 或其变形
    % 对于移动关节，局部旋量通常是 [0;0;0;0;0;1] 或其变形
    params.xi_local = [
        0, 0, 0, 0, 0, 0;     % wx
        0, 0, 0, 0, 0, 0;     % wy  
        1, 1, 0, 1, 1, 1;     % wz (绕z轴旋转)
        0, 0, 0, 0, 0, 0;     % vx
        0, 0, 1, 0, 0, 0;     % vy (第3关节为移动关节，沿y轴)
        0, 0, 0, 0, 0, 0      % vz
    ];
    
    % 末端执行器在零位时的变换矩阵
    params.M = [eye(3), [0; 0; l0 + l1]; 0, 0, 0, 1];
    
    % 关节限制
    params.joint_limits = [
        -pi, pi;           % theta1
        -pi, pi;           % theta2  
        -0.1, 0.1;         % d3 (移动关节，米)
        -pi, pi;           % theta4
        -pi, pi;           % theta5
        -pi, pi            % theta6
    ];
end

function params = getDefaultSolverParams()
% 默认求解器参数
    params.max_iterations = 100;
    params.tolerance = 1e-6;
    params.damping_factor = 1e-6;
    params.step_size = 0.5;
    params.max_step_size = 0.2;
end

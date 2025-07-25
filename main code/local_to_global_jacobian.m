function [Js, xi_global_list] = local_to_global_jacobian(g_chain, xi_local, theta)
% 基于局部指数积描述的旋量构建全局雅可比矩阵
%
% 输入:
%   g_chain  - 1×(n+1) cell，固定变换矩阵链
%              g_chain{i}为从坐标系{i-1}到{i}的零位形变换
%   xi_local - 6×n 矩阵，各关节在自身坐标系中的单位运动旋量
%   theta    - n×1 向量，当前关节角度/位移
%
% 输出:
%   Js           - 6×n 空间雅可比矩阵（全局描述）
%   xi_global_list - 6×n 矩阵，转换后的全局运动旋量
%
% 理论基础:
%   利用伴随变换将局部旋量转换为全局坐标系表示
%   xi_global_i = Ad(T_0_to_i) * xi_local_i
%   其中 T_0_to_i 为从基座到关节i坐标系的累积变换

    % 维度验证
    [~, n] = size(xi_local);
    if length(theta) ~= n
        error('theta长度与关节数量不匹配');
    end
    if length(g_chain) ~= n + 1
        error('g_chain长度应为n+1');
    end
    
    % 初始化输出
    Js = zeros(6, n);
    xi_global_list = zeros(6, n);
    
    % 累积变换矩阵
    T_accum = eye(4);
    
    % 逐个关节计算
    for i = 1:n
        % 更新累积变换到关节i的坐标系
        T_accum = T_accum * g_chain{i};
        
        % 计算伴随变换矩阵
        Ad_T = adjointSE3(T_accum);
        
        % 将局部旋量转换为全局表示
        xi_global_i = Ad_T * xi_local(:, i);
        xi_global_list(:, i) = xi_global_i;
        
        % 雅可比矩阵的第i列就是全局旋量
        Js(:, i) = xi_global_i;
        
        % 为下一次迭代准备：应用当前关节的运动
        if i < n
            T_joint = exp_se3(xi_local(:, i), theta(i));
            T_accum = T_accum * T_joint;
        end
    end
end

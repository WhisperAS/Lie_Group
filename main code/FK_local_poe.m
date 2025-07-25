function T = FK_local_poe(g_chain, xi_local, theta)
% 串联机器人 "局部指数积公式" 正运动学 
%
% T = FK_local_poe(g_chain, xi_local, theta)
%
% 输入
%   g_chain  - 1×n cell，每个元素为 4×4 齐次矩阵  
%              g_chain{i} = g_{i+1}_i，表示初始位形下坐标系{i+1}在{i}中的表示
%   xi_local - 6×n 矩阵，第 i 列为关节 i 在**自身坐标系**中的单位运动旋量  
%   theta    - n×1 向量，关节变量（角度或位移）
%
% 输出
%   T        - 4×4 齐次矩阵，末端执行器在基座坐标系中的位姿
%
% 公式（以4关节为例）：
%   T = g_1_0 · exp([xi₁]× θ₁) · g_2_1 · exp([xi₂]× θ₂) · g_3_2 · exp([xi₃]× θ₃) · g_4_3 · exp([xi₄]× θ₄) · g_ee_4
%
% 注意：关节i的运动发生在从坐标系{i-1}到{i}的变换过程中

    % --- 维度校验 -------------------------------------------------
    [~, n] = size(xi_local);
    if length(theta) ~= n
        error('theta 的长度应与 xi_local 的列数一致');
    end
    if length(g_chain) ~= n + 1
        error('g_chain 的元素个数必须为 n+1 (包含末端变换)');
    end

    % --- 累积计算 -------------------------------------------------
    T = eye(4);                           % 从基座坐标系开始
    
    for i = 1:n
        % 先应用从坐标系{i-1}到{i}的固定变换
        T = T * g_chain{i};               % g_{i}_{i-1}，即坐标系{i}在{i-1}中的表示
        
        % 然后在坐标系{i}中应用关节i的运动
        T = T * exp_se3(xi_local(:,i), theta(i));
    end
    
    % 最后乘以末端执行器的固定变换
    T = T * g_chain{n+1};                 % g_ee_n
end

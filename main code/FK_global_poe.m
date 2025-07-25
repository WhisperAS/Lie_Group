function T = FK_global_poe(xi_list, theta, M)
    % 串联机器人全局指数积公式正运动学
    % 输入:
    %   xi_list - 6×n矩阵，每列表达第i个关节在初始位形下相对惯性坐标系的单位运动旋量坐标 [w; v]
    %   theta   - n×1向量，关节角度/位移
    %   M       - 4×4齐次变换矩阵，零位形时末端执行器相对基座的位姿
    % 输出:
    %   T       - 4×4齐次变换矩阵，末端执行器当前位姿
    
    % 验证输入维度
    [~, n] = size(xi_list);
    if length(theta) ~= n
        error('关节数量与关节角度向量长度不匹配');
    end
    
    % 初始化变换矩阵为单位矩阵
    T = eye(4);
    
    % 按顺序计算每个关节的指数积
    for i = 1:n
        % 提取第i个关节的运动旋量
        xi_i = xi_list(:, i);
        
        % 计算该关节的变换矩阵
        T_i = exp_se3(xi_i, theta(i));
        
        % 累积变换
        T = T * T_i;
    end
    
    % 乘以零位形变换矩阵
    T = T * M;
end

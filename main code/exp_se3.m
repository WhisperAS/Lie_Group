function T = exp_se3(xi, theta)
    % SE(3)群的指数映射 - 结合旋量归一化处理
    % 将se(3)李代数元素映射为SE(3)李群元素
    % 
    % 输入:
    %   xi    - 6×1运动旋量向量 [w; v]，其中:
    %           w (3×1) - 角速度向量/旋转轴
    %           v (3×1) - 线速度向量/移动方向
    %   theta - 标量，运动幅度（可选，默认为1）
    %
    % 输出:
    %   T     - 4×4齐次变换矩阵，属于SE(3)群
    
    % 处理可选参数
    if nargin < 2
        theta = 1;
    end
    
    % 输入验证
    if length(xi) ~= 6
        error('exp_se3: 输入必须是6×1向量');
    end
    
    % 将输入转换为列向量
    xi = xi(:);
    
    % 清理微小数值，避免数值误差
    xi(abs(xi) <= 1e-10) = 0;
    
    % 提取旋转和平移部分
    w = xi(1:3);
    v = xi(4:6);
    
    % 计算向量范数
    w_norm = norm(w);
    v_norm = norm(v);
    
    % 处理不同的运动类型
    if w_norm == 0 && v_norm == 0
        % 零运动情况
        T = eye(4);
        return;
    elseif w_norm == 0
        % 纯平移运动（移动关节）
        T = [eye(3), v * theta;
             0, 0, 0, 1];
        return;
    else
        % 一般螺旋运动或纯旋转运动
        % 使用您的归一化策略
        if v_norm == 0
            % 纯旋转情况，保持原有逻辑
            w_unit = w / w_norm;
            xi_normalized = [w_unit; zeros(3,1)];
            magnitude = w_norm;
        else
            % 螺旋运动，按旋转分量归一化
            w_unit = w / w_norm;
            v_normalized = v / w_norm;
            xi_normalized = [w_unit; v_normalized];
            magnitude = w_norm;
        end
    end
    
    % 计算实际的运动幅度
    total_magnitude = magnitude * theta;
    
    % 小角度情况使用线性近似
    if total_magnitude < 1e-12
        xi_hat = hat_se3(xi_normalized * total_magnitude);
        T = eye(4) + xi_hat;
    else
        % 使用expm计算
        xi_hat = hat_se3(xi_normalized * total_magnitude);
        T = expm(xi_hat);
    end
end

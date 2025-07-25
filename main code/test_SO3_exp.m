% 验证脚本：使用Robotics System Toolbox的axang2rotm验证自定义指数映射

% 定义绕x,y,z轴的旋转轴（单位向量）
omega_x = [1 0 0];
omega_y = [0 1 0];
omega_z = [0 0 1];

% 定义旋转角度（弧度）
q_x = pi/6; % 30度
q_y = pi/4; % 45度
q_z = pi/3; % 60度

% 机器人系统工具箱轴角格式：[axis_x axis_y axis_z angle]
axang_x = [omega_x, q_x];
axang_y = [omega_y, q_y];
axang_z = [omega_z, q_z];

% 使用Robotics System Toolbox函数生成旋转矩阵
R_x_toolbox = axang2rotm(axang_x);
R_y_toolbox = axang2rotm(axang_y);
R_z_toolbox = axang2rotm(axang_z);

% 使用自定义指数映射函数计算旋转矩阵
R_x_custom = exp_so3(omega_x', q_x);
R_y_custom = exp_so3(omega_y', q_y);
R_z_custom = exp_so3(omega_z', q_z);

% 计算并显示差异范数，越接近0表示越一致
fprintf('绕X轴旋转矩阵差异范数: %.3e\n', norm(R_x_toolbox - R_x_custom));
fprintf('绕Y轴旋转矩阵差异范数: %.3e\n', norm(R_y_toolbox - R_y_custom));
fprintf('绕Z轴旋转矩阵差异范数: %.3e\n', norm(R_z_toolbox - R_z_custom));

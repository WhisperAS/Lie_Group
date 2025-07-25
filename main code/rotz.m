function Rz = rotz(theta)
% 绕Z轴旋转的旋转矩阵
% 输入: theta - 旋转角度(弧度)
% 输出: Rz - 3x3旋转矩阵

c = cos(theta);
s = sin(theta);

Rz = [c, -s,  0;
      s,  c,  0;
      0,  0,  1];
end

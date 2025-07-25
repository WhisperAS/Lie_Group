function Rx = rotx(theta)
% 绕X轴旋转的旋转矩阵
% 输入: theta - 旋转角度(弧度)
% 输出: Rx - 3x3旋转矩阵

c = cos(theta);
s = sin(theta);

Rx = [1,  0,  0;
      0,  c, -s;
      0,  s,  c];
end

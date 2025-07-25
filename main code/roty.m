function Ry = roty(theta)
% 绕Y轴旋转的旋转矩阵
% 输入: theta - 旋转角度(弧度)  
% 输出: Ry - 3x3旋转矩阵

c = cos(theta);
s = sin(theta);

Ry = [ c,  0,  s;
       0,  1,  0;
      -s,  0,  c];
end

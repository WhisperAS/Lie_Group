function omega_hat = hat_so3(omega)
% hat_so3 将3x1旋转向量omega转换为3x3反对称矩阵。
% 对应知乎文章：《基于指数积方法的串联机器人标定数学模型》中的式（1）
%
% 输入:
%   omega - 3x1的旋转向量 [omega_x; omega_y; omega_z]
%
% 输出:
%   omega_hat - 3x3的反对称矩阵
%               [ 0        -omega_z   omega_y;
%                 omega_z   0        -omega_x;
%                -omega_y   omega_x   0      ]

    omega_hat = [  0       -omega(3)  omega(2);
                 omega(3)    0      -omega(1);
                -omega(2)  omega(1)    0    ];
end

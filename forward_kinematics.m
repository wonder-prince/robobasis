% 正运动学解算函数
function T = forward_kinematics(theta, d, a, alpha)
    % 计算正向运动学变换矩阵
    T = [
        cos(theta), -sin(theta), 0, a;
        cos(alpha)*sin(theta), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha);
        sin(alpha)*sin(theta), sin(alpha)*cos(theta), cos(alpha), d*cos(alpha);
        0, 0, 0, 1
    ];
end
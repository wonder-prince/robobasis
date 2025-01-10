clear all;
clc;
base = [0 ;0 ; 0; 1];
% 赋予输入值
L1 = 149; % 第一个连杆长度
L2 = 100; % 第二个连杆长度
L3 = 98;  % 第三个连杆长度
L4 = 176; % 第四个连杆长度
IN_theta_deg = [0, 0, 0, 0, 0]; % 初始关节角度（度）
IN_theta = deg2rad(IN_theta_deg); % 将角度转换为弧度
% 建立D-H参数表
C_a = [0, 0, L2, L3, 0, 0]; % 连杆长度
C_d = [L1, 0, 0, 0, L4, 0]; % 连杆偏移
C_alpha = [0, -pi/2, 0, 0, -pi/2, 0]; % 连杆扭转角，-90度转换为 -pi/2 弧度
C_theta = [IN_theta(1), IN_theta(2), IN_theta(3), IN_theta(4)-pi/2, IN_theta(5), 0]; % 关节角，第四个关节角需要减去pi/2

% 计算正向运动学
T_total = eye(4); % 初始化变换矩阵为单位矩阵

% 用于保存每个关节的空间位置
joint_positions = zeros(6, 4);  % 6个关节的空间位置（X, Y, Z）
% 设置第一个关节位置为基坐标
joint_positions(1, :) = base(1:4)';  % 将基坐标作为第一个关节的位置

% 计算正向运动学并输出每个关节的位置
for i = 1:6
    T = forward_kinematics(C_theta(i), C_d(i), C_a(i), C_alpha(i)); % 计算每个关节的变换矩阵
    T_total = T_total * T; % 累积变换矩阵
    if i>1
    % 计算当前关节的位置（去掉齐次坐标的最后一维）
    joint_pos = T_total * base;  
    joint_positions(i, :) = joint_pos(1:4);  % 保存每个关节的位置
    end
end

% 输出每个关节的位置
% disp('每个关节的空间位置：');
% disp(joint_positions);

% 输出最终的变换矩阵
disp(T_total);
% 输出最后的位置（通过变换矩阵 T_total 计算末端执行器的位置）
aim = T_total * base; % 末端执行器的位置
disp('末端执行器的位置 (齐次坐标):');
disp(aim);
% 可视化关节位置
% figure;
% plot3(joint_positions(:,1), joint_positions(:,2), joint_positions(:,3), '-o', 'MarkerSize', 6, 'LineWidth', 2);  % 绘制关节位置
% hold on;
% grid on;
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('机械臂关节位置');
% legend('关节位置');


%% 直线插补
% 定义起点、终点和插补点数
num = 11; % 插补点数
p_1 = [336 0 212]; % 直线插补起点
p_3 = [145 0 394]; % 直线插补终点
% 计算插补点
t = linspace(0, 1, num); % 创建0到1之间的均匀分布点
points = (1 - t)' * p_1 + t' * p_3; % 直线插补公式
% 绘制插补直线
figure;
plot3(points(:,1), points(:,2), points(:,3), '-o', 'MarkerSize', 6, 'LineWidth', 2); % 绘制直线和插补点
hold on;
plot3([p_1(1), p_3(1)], [p_1(2), p_3(2)], [p_1(3), p_3(3)], 'r--', 'LineWidth', 1); % 绘制起点到终点的直线
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('插补直线及插补点');
legend('插补点', '起点到终点的直线');
% % 求解theta1
% % 计算方向向量 (在XY平面上)
% v = p_3(1:2) - p_1(1:2); % 方向向量 [dx, dy]
% % 计算方向向量的模长
% v_norm = norm(v); % ||v|| = sqrt(v_x^2 + v_y^2)
% % 计算与X轴的夹角
% theta11 = acos(v(1) / v_norm); % 夹角公式，返回的是弧度值
% % 将弧度转换为角度
% theta1_deg = rad2deg(theta11);
% % 输出夹角
% disp(['theta1 :', num2str(theta1_deg), ' 度']);
%% 逆运动学解算验证
% 初始化标志变量
solution_found = false;
% % 多组XYZ坐标，每行代表一组
% XYZ_all1 = T_total(1:3,4);
% %进行转置
% XYZ_all = XYZ_all1';
%将计算的XYZ填入到下面，也可以直接输出
XYZ_all = [  
   275 23 149
];
% 初始化结果矩阵
results = [];
% 对每一组XYZ坐标进行计算
for j = 1:size(XYZ_all, 1)
x = XYZ_all(j, 1);
y = XYZ_all(j, 2);
z = XYZ_all(j, 3);
theta1 = rad2deg(atan2(y, x));  % 使用 atan2 代替 atan
solution_found = false; % 重置标志变量
% 遍历 alpha 的值，从 -45 到 90
for angle_change = -45:90
%三维转二维
t = sqrt(x^2 + y^2);
m = z - L1 - L4 * sind(angle_change);
n = t - L4 * cosd(angle_change);
% 计算theta3，余弦取倒数
theta3 = acosd((m^2 + n^2 - L2^2 - L3^2) / (2 * L2 * L3));

A = L3 * sind(theta3);
B = L2 + L3 * cosd(theta3);
% 计算theta2
theta2 = asind(m / sqrt(A^2 + B^2)) + acosd(B / sqrt(A^2 + B^2));
if theta2 < 0
theta2 = 180 - asind(m / sqrt(A^2 + B^2)) + acosd(B / sqrt(A^2 + B^2));
end
tolerance = 1e-6;  % 设置容差范围
if abs(theta2 - 180) < tolerance
    theta2 = 0;
end
% 最终得到theta4
theta4 = theta2 - theta3 - angle_change;
% 检验是否是复数解，是则计算出错
if isreal(theta2) && isreal(theta3) && isreal(theta4)
    % 检查是否找到解
    if L4 * sind(angle_change) <= z && L3 * cosd(theta2 - theta3) >= 0 && theta2 >= 0 && theta2 <= 135 && theta3 >= 0 && theta3 <= 90 && theta4 >= -3 && theta4 <= 90 
    % 记录找到解的标志
    solution_found = true;
    break;
    end
end
end
if solution_found
% 将结果添加到结果矩阵中
results = [results; theta1, -theta2, theta3, theta4];
else
% 如果没有找到解，将 NaN 添加到结果矩阵中
results = [results; NaN, NaN, NaN, NaN, NaN];
end
end
% 显示结果
disp('Results:');
disp('Theta1 Theta2 Theta3 Theta4');
disp(results);
results_1 =results/1.5;
%% 机械臂输入值
%误差
error1 = 10.4;
error2 = 0;
error3 = -4;
erroryaw = 8;
%results_1(:,1) = results_1(:,1) -erroryaw;
results_1(:,2) = 120+results_1(:,2) - error1;
results_1(:,3) = 90+results_1(:,3) - error2;
results_1(:,4) = 90+results_1(:,4) - error3;
disp('调整以后的机械臂参数')
disp(results_1);
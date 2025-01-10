import sensor, image, time, math,pyb

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # 使用RGB模式
sensor.set_framesize(sensor.QVGA)    # 设置分辨率
sensor.skip_frames(time=2000)        # 等待摄像头初始化完成
sensor.set_auto_gain(False)          # 关闭自动增益，避免颜色受光线影响
sensor.set_auto_whitebal(False)      # 关闭白平衡
sensor.set_vflip(True)              # 垂直翻转
#设置串口通信
uart = pyb.UART(3, 115200)  # 选择UART3，波特率115200

# 设置阈值，调整为适合黑色检测的灰度范围
threshold = [ [17, 100] ] * 6  # 设置 6 个区域的阈值

# 设置中值
mid_line = 160
# 设置最大速度
max_speed = 400
#设置前后系数
ka = 1
# 设置距离系数
kb = 1
# 设置旋转系数
kc = 1
# 红色、绿色、蓝色阈值
red_threshold = (37, 94, 6, 95, -20, 55)
green_threshold = (22, 81, -58, -22, -2, 53)
blue_threshold = (17, 72, -16, 26, -47, -9)

# 初始化标志
red_flag = 0
green_flag = 0
blue_flag = 0
clock = time.clock()
# 定义函数：通用目标识别
def detect_color(img, color_threshold, color_name):
    global red_flag, green_flag, blue_flag

    if color_name == "red":
        flag = red_flag
    elif color_name == "green":
        flag = green_flag
    else:
        flag = blue_flag

    flag = 0
    blobs = img.find_blobs([color_threshold], x_stride=50, y_stride=50, merge=True)
    if blobs:
        largest_blob = max(blobs, key=lambda b: b.pixels())
        img.draw_rectangle(largest_blob.rect())  # 绘制目标区域
        img.draw_cross(largest_blob.cx(), largest_blob.cy())  # 绘制中心十字
        flag = 1

    # 根据目标颜色设置不同的标志量
    if color_name == "red":
        red_flag = flag
    elif color_name == "green":
        green_flag = flag
    else:
        blue_flag = flag

    return flag

# 定义函数：识别红色目标
def detect_red(img):
    return detect_color(img, red_threshold, "red")

# 定义函数：识别绿色目标
def detect_green(img):
    return detect_color(img, green_threshold, "green")

# 定义函数：识别蓝色目标
def detect_blue(img):
    return detect_color(img, blue_threshold, "blue")

# 定义函数：判断哪个标志量为1
def check_flag():
    # 判断三个标志量中是否只有一个为1，并返回相应的编号
    if red_flag == 1 and green_flag == 0 and blue_flag == 0:
        return 1  # 红色标志量为1
    elif red_flag == 0 and green_flag == 1 and blue_flag == 0:
        return 2  # 绿色标志量为2
    elif red_flag == 0 and green_flag == 0 and blue_flag == 1:
        return 3  # 蓝色标志量为3
    else:
        return 0  # 如果三个标志量不止一个为1，则返回无效状态
# 定义函数：计算黑线区域的中心点和角度
def calculate_line_center_and_angle(img, threshold, midpoint):
    img.to_grayscale()
    #img.binary([threshold[0], threshold[1]])  # 二值化，使用给定的阈值范围
    line_point = [0, 0, 0, 0, 0, 0]  # 存储六个区域的中心点
    line_area = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]  # 存储六个区域的矩形框
    for n in range(0, 6):  # 循环6次，分别对应画面被分割的6个区域
        threshold[n] = [17, 90]  # 默认阈值为0~80，黑白画面
        while True:  # 自动收束阈值的循环
            blobs = img.find_blobs([threshold[n]], roi=(0, n * 40, 320, 40), x_stride=5, y_stride=5, pixels_threshold=50)
            if blobs:  # 如果有结果
                if len(blobs) > 1:  # 多个结果
                    if threshold[n][1] > 20:  # 阈值大于20之前
                        threshold[n][1] -= 5  # 每次减5（收束阈值）
                    else:  # 阈值等于20了还有多个结果
                        break  # 跳出
                else:  # 只有一个结果
                    break  # 跳出
            else:  # 没有结果
                break  # 跳出
        if blobs:  # 如果有识别结果
            blob = max(blobs, key=lambda b: b.pixels())  # 找到面积最大的色块
            line_area[n] = blob.rect()  # 记录当前色块所在区域
            line_point[n] = blob.cx() - midpoint  # 记录色块的中心点
            if midpoint * 0.75 < blob.w() < midpoint * 1.75:
                line_point[n] = line_point[n] * 2
        else:
            line_area[n] = -1  # 没有识别到结果装填一个负值
            line_point[n] = -500  # 没有找到黑线的标志值
    return line_point, line_area

# 定义函数：绘制区域和中心点
def draw_line_areas_and_centers(img, line_area, line_point):
    for n in range(0, 6):
        if line_area[n] != -1:  # 确保有有效区域
            img.draw_rectangle(line_area[n])
# 定义函数：计算距离和斜率
def calculate_line_distance_and_slope(line_point, midpoint):
    line_none = 0  # 记录没有发现黑线段的次数
    move_x = 0  # 初始平移值
    move_turn = 0  # 初始旋转值
    first_valid_point = None  # 第一个有效点的索引
    last_valid_point = None   # 最后一个有效点的索引

    # 倒叙循环6次，从下到上查找第一个有效的黑线中心点
    for n in range(5, -1, -1):
        if line_point[n] > -500:  # 如果有有效数值
            move_x = line_point[n]  # 记录平移位置
            first_valid_point = n  # 记录第一个有效点的索引
            break  # 跳出循环，只处理第一个有效点

    # 找到最后一个有效点
    for n in range(5, -1, -1):
        if line_point[n] > -500:
            last_valid_point = n  # 记录最后一个有效点的索引

    if first_valid_point is not None and last_valid_point is not None and first_valid_point != last_valid_point:
        # 计算斜率：使用第一个有效点和最后一个有效点来计算斜率
        delta_y = line_point[last_valid_point] - line_point[first_valid_point]  # y方向的变化
        delta_x = last_valid_point - first_valid_point  # x方向的变化
        move_turn = delta_y / delta_x  # 计算斜率

    # 返回平移距离和旋转角度（黑线到中点的斜率）
    return move_x, move_turn
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp  # 比例增益
        self.Ki = Ki  # 积分增益
        self.Kd = Kd  # 微分增益
        self.previous_error = 0  # 上一次误差
        self.integral = 0  # 积分值
        self.output = 0  # 控制器输出

    def update(self, error):
        # 计算误差的积分和微分
        self.integral += error
        derivative = error - self.previous_error

        # PID控制器公式
        self.output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # 保存当前误差，以便下次计算微分
        self.previous_error = error

        return self.output

def control_diff_wheels(control_move_x, control_move_turn, max_speed):
    """
    计算二轮差速机器人的左轮和右轮速度，根据PID控制器的输出 (control_move_x 和 control_move_turn)。
    :param control_move_x: 平移控制量（前进/后退）
    :param control_move_turn: 旋转控制量（左右转向）
    :param max_speed: 轮子的最大速度（用于限制速度）
    :return: 返回左轮和右轮的速度 [left_speed, right_speed]
    """
    # 转换旋转角度到弧度
    move_turn_rad = math.radians(control_move_turn)

    # 计算平移速度 (V_x) 和旋转角速度 (V_omega)
    V_x = -control_move_x  # 前进/后退速度
    V_omega = -control_move_turn*5  # 旋转角速度

    # 计算左右轮的速度（V_left = 左轮，V_right = 右轮）
    V_left = V_x - V_omega  # 左轮速度
    V_right = -V_x + V_omega  # 右轮速度

    # 对速度进行归一化，确保最大速度不超过 max_speed
    max_wheel_speed = max(abs(V_left), abs(V_right))

    if max_wheel_speed > max_speed:
        # 如果某个轮子的速度超过了最大速度，将所有轮子的速度缩放
        V_left = V_left / max_wheel_speed * max_speed
        V_right = V_right / max_wheel_speed * max_speed

    # 将速度转换为整数（假设你需要整数值）
    left_speed = int(V_left)
    right_speed = int(V_right)

    return left_speed, right_speed

# 初始化 PID 控制器
pid_move_x = PID(Kp=6.5, Ki=0, Kd=0)  # 用于控制平移
pid_move_turn = PID(Kp=4 , Ki=0, Kd=0)  # 用于控制旋转

# 目标值（可以根据需要进行调整）
target_move_x = 0  # 目标平移值，例如，目标是直行则设为0
target_move_turn = 0  # 目标旋转角度，设为0表示目标朝向
# 主循环
while True:
    clock.tick()
    img = sensor.snapshot()  # 拍一张照片并返回图像
    detect_red(img)
    detect_green(img)
    detect_blue(img)
    color = check_flag()
    #img.flip(True)  # 水平翻转
    line_point, line_area = calculate_line_center_and_angle(img, threshold, mid_line)
    move_x, move_turn = calculate_line_distance_and_slope(line_point, mid_line)  # 计算平移和旋转
    draw_line_areas_and_centers(img, line_area, line_point)
    # 计算 `move_x` 和 `move_turn` 与目标值之间的误差
    error_move_x = target_move_x - move_x  # 平移误差
    error_move_turn = target_move_turn - move_turn  # 旋转误差
    # 假设 PID 控制器的输出
    control_move_x = pid_move_x.update(error_move_x)
    control_move_turn = pid_move_turn.update(error_move_turn)
    left_speed, right_speed = control_diff_wheels(move_x, move_turn, max_speed)
    #print(move_x,move_turn)
    speed_str = f"{left_speed} {right_speed} {left_speed} {right_speed}"
    info = f"{error_move_x} {error_move_turn} {color}"
    print(info + "%")
    #uart.write(speed_str + "%")  # 如果检测到目标，发8送信号
    uart.write(info + "%")

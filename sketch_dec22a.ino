#include <Servo.h>
//定义超声波引脚
const int TRIG_PIN = A1;  // 超声波模块 Trig 引脚
const int ECHO_PIN = A2;  // 超声波模块 Echo 引脚
// 定义距离阈值
const float BACK_DISTANCE_THRESHOLD = 60 ;   //后退阈值
const float DISTANCE_THRESHOLD = 45.0; // 距离阈值，单位：厘米
const float WRITE_DISTANCE_THRESHOLD =20;
unsigned long startTime;  // 记录开始时间
unsigned long lasting_time = 1000;  // 持续读取的时间（毫秒）
unsigned long lasting_time_1 = 10000; //初次循迹关闭超声波时间
unsigned long lasting_time_red = 28000;//红色路径关闭超声波时间
unsigned long lasting_time_green = 50000;
unsigned long lasting_time_blue = 25000;
int ra_counter = 0;
int color_counter = 0;//用于计算物块抓取数量
//定义写字初始化参数
const int number1 = 10;
const int number2 = 10;
float angles_write1[number1+2][3];
float angles_write2[number2+1][3];
//十字
//第一笔
float angles1[number1+2][3] = {
      {90,0,180},//归位

    {90.00, 31.39, 166.48},
    {87.18, 31.48, 166.42},
    {84.38, 31.76, 166.27},
    {81.65, 32.22, 166.01},
    {79.00, 32.86, 165.64},
    {76.45, 33.66, 165.18},
    {74.03, 34.60, 164.61},
    {71.73, 35.69, 163.95},
    {69.57, 36.90, 163.19},
    {67.54, 38.23, 162.33},

    {90,0,180},//归位
};
//第二笔
float angles2[number2+1][3] = {
      {77.71, 30.48, 173.92},
    {77.71, 30.96, 172.13},
    {77.71, 31.52, 170.29},
    {77.71, 32.15, 168.38},
    {77.71, 32.86, 166.42},
    {77.71, 33.63, 164.41},
    {77.71, 34.47, 162.33},
    {77.71, 35.36, 160.19},
    {77.71, 36.32, 157.99},
    {77.71, 37.34, 155.72},

    {90,0,180},//归位
};

// 二字
// 第一笔
float angles3[number1+2][3] = {
    {90,0,180},//归位

    {90.00, 31.39, 166.48},
    {87.18, 31.48, 166.42},
    {84.38, 31.76, 166.27},
    {81.65, 32.22, 166.01},
    {79.00, 32.86, 165.64},
    {76.45, 33.66, 165.18},
    {74.03, 34.60, 164.61},
    {71.73, 35.69, 163.95},
    {69.57, 36.90, 163.19},
    {67.54, 38.23, 162.33},

    {90,0,180},//归位
};
//第二笔
float angles4[number2+1][3] = {
    {74.03, 38.55, 154.96},
    {74.82, 38.26, 155.15},
    {75.63, 37.98, 155.33},
    {76.45, 37.71, 155.49},
    {77.29, 37.46, 155.65},
    {78.14, 37.23, 155.79},
    {79.00, 37.01, 155.92},
    {79.87, 36.81, 156.05},
    {80.75, 36.62, 156.16},
    {81.65, 36.46, 156.26},

    {90,0,180},//归位
};

// T
// 第一笔
float angles5[number1+2][3] = {
      {90,0,180},//归位

    {90.00, 31.39, 166.48},
    {87.18, 31.48, 166.42},
    {84.38, 31.76, 166.27},
    {81.65, 32.22, 166.01},
    {79.00, 32.86, 165.64},
    {76.45, 33.66, 165.18},
    {74.03, 34.60, 164.61},
    {71.73, 35.69, 163.95},
    {69.57, 36.90, 163.19},
    {67.54, 38.23, 162.33},

    {90,0,180},//归位
};
//第二笔
float angles6[number2+1][3] = {
    {77.71, 30.48, 173.92},
    {77.71, 30.71, 173.03},
    {77.71, 30.96, 172.13},
    {77.71, 31.23, 171.22},
    {77.71, 31.52, 170.29},
    {77.71, 31.83, 169.34},
    {77.71, 32.15, 168.38},
    {77.71, 32.50, 167.41},
    {77.71, 32.86, 166.42},
    {77.71, 33.24, 165.42},

    {90,0,180},//归位
};
// 1. 定义舵机
Servo servo_7;
Servo servo_3;
Servo servo_5;
Servo servo_6;
Servo servo_9;
Servo servo_8;
char cmd_return_tmp[64];
// 定义四个电机的速度
float motor_left_front_speed = 0;   // 左前电机速度
float motor_right_front_speed = 0;  // 右前电机速度
float motor_left_back_speed = 0;    // 左后电机速度
float motor_right_back_speed = 0;   // 右后电机速度
float pid_x_value = 0;    
float pid_turn_value = 0;
int color = 3;
String data = "";  
//声明函数
void redLogic();
void greenLogic();
void blueLogic();
// 2. 定义 Motion 类
class Motor {
public:
  Motor() {}
  Motor(int id) : id(id) {}
  
  ~Motor() {}

  int id;

private:
  int pwm;
  int time;

public:
  void reversal(int pwm) { 
    if (pwm < 0) {
      pwm = 0;
    }
    if (pwm > 1000) {
      pwm = 1000;
    }
    if (this->id == 6 || this->id == 8) {
      sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", this->id, 1500 - pwm, 0);
      Serial.println(cmd_return_tmp);
      delay(10);
    } else {
      sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", this->id, 1500 + pwm, 0);
      Serial.println(cmd_return_tmp);
      delay(10);
    }
  }

  void foreward(int pwm) { 
    if (pwm < 0) {
      pwm = 0;
    }
    if (pwm > 1000) {
      pwm = 1000;
    }
    if (this->id == 6 || this->id == 8) {
      sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", this->id, 1500 + pwm+20, 0);
      Serial.println(cmd_return_tmp);
      delay(10);
    } else {
      sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", this->id, 1500 - pwm-20, 0);
      Serial.println(cmd_return_tmp);
      delay(10);
    }
  }

  void start(int pwm) { 
    if (pwm <= 0) {
      if (pwm < -1000) {
        pwm = -1000;
      }
      pwm = 0 - pwm;
      this->reversal(pwm);
    } else if (pwm >= 0) {
      if (pwm > 1000) {
        pwm = 1000;
      }
      this->foreward(pwm);
    }
  }

  void stop() { 
    sprintf(cmd_return_tmp, "#%03dP%04dT%04d!", this->id, 1500, 0);
    Serial.println(cmd_return_tmp);
    delay(10);
  }
};

class Motion {
public:
  Motion(int id1, int id2, int id3, int id4) {
    this->motor1.id = id1;
    this->motor2.id = id2;
    this->motor3.id = id3;
    this->motor4.id = id4;
  }

  ~Motion() {}

private:
  Motor motor1;
  Motor motor2;
  Motor motor3;
  Motor motor4;

public:
  void moveForeward(int speed) { 
    if (speed < 0) {
      speed = 0;
    }
    if (speed > 1000) {
      speed = 1000;
    }
    motor1.foreward(speed);
    motor2.foreward(speed);
    motor3.foreward(speed);
    motor4.foreward(speed);
  }
  void time_moveForeward(int speed,int time) { 
    if (speed < 0) {
      speed = 0;
    }
    if (speed > 1000) {
      speed = 1000;
    }
    motor1.foreward(speed);
    motor2.foreward(speed);
    motor3.foreward(speed);
    motor4.foreward(speed);
    delay(time);
  }
  void rotate_right(int speed,int time)
  {
    if(speed < 0) 
    {
      speed = 0; // 限制最小速度为0
    }
    if(speed > 1000) 
    {
      speed = 1000; // 限制最大速度为1000
    }
    
    // 左前电机顺时针，右后电机顺时针
    motor1.start(speed-100);
    motor3.start(speed-100);

    // 右前电机反时针，左后电机反时针
    motor2.start(-speed);
    motor4.start(-speed);
    delay(time);
  }
  void rotate_left(int speed,int time)
  {
        if(speed < 0) 
    {
      speed = 0; // 限制最小速度为0
    }
    if(speed > 1000) 
    {
      speed = 1000; // 限制最大速度为1000
    }

    
    motor1.start(-speed);
    motor3.start(-speed);

    // 右前电机顺时针，左后电机顺时针
    motor2.start(speed);
    motor4.start(speed);
    delay(time);
  }
  void moveBack(int speed,int time) { 
    if (speed < 0) {
      speed = 0;
    }
    if (speed > 1000) {
      speed = 1000;
    }
    motor1.reversal(speed-50);
    motor2.reversal(speed);
    motor3.reversal(speed-50);
    motor4.reversal(speed);
    delay(time);
  }

  void moveStop() { 
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
  }

  void setDifferential(int rspeed, int lspeed) { 
    motor1.start(rspeed);
    motor3.start(rspeed);
    motor2.start(lspeed);
    motor4.start(lspeed);
  }

  void moveRight(int speed) { 
    motor1.start(speed);
    motor4.start(speed);
    motor2.start(0 - speed);
    motor3.start(0 - speed);
    delay(350);
  }

  void moveLeft(int speed) { 
    motor1.start(0 - speed);
    motor4.start(0 - speed);
    motor2.start(speed);
    motor3.start(speed);
    delay(200);
  }

  void kongzhi_test() {
    motor_left_front_speed = 250 + pid_x_value*2;   
    motor_right_front_speed = 250 - pid_x_value*2;  
    motor_left_back_speed = 250 + pid_x_value*2;    
    motor_right_back_speed = 250 - pid_x_value*2;   
    setDifferential(motor_right_front_speed, motor_left_front_speed); 
    delay(10);
  }

  void moveTest() {
    if (pid_turn_value < -95) {
      moveRight(400); 
    } else if (pid_turn_value > 95) {
      moveLeft(400);  
      //moveForeward(300); 
    }
  }
};

// 3. 创建 Motion 实例
Motion motion(6, 7, 8, 9);

void setup() {
  Serial.begin(115200); 
  delay(400);           
  initializeServo();    
  pinMode(TRIG_PIN, OUTPUT);  // 设置 Trig 引脚为输出
  pinMode(ECHO_PIN, INPUT);   // 设置 Echo 引脚为输入
}

void initializeServo() {
  servo_7.attach(7);
  servo_3.attach(3);
  servo_5.attach(5);
  servo_6.attach(6);
  servo_9.attach(9);
  servo_8.attach(8);
  setServoPositions();
}

void setServoPositions() {
  servo_7.write(88);
  servo_3.write(35);
  servo_5.write(145);
  servo_6.write(150);
  servo_9.write(90);
  servo_8.write(200);
}

void handleOpenMVData(String data) {
  int firstSpaceIdx = data.indexOf(' ');  // 查找第一个空格的位置
  int secondSpaceIdx = data.indexOf(' ', firstSpaceIdx + 1);  // 查找第二个空格的位置
  int percentIdx = data.indexOf('%');  // 查找百分号的位置

  // 提取第一个部分（pid_x_value）
  String firstPart = data.substring(0, firstSpaceIdx);
  pid_x_value = -firstPart.toFloat();  

  // 提取第二个部分（pid_turn_value）
  String secondPart = data.substring(firstSpaceIdx + 1, secondSpaceIdx);
  pid_turn_value = -secondPart.toFloat();
  pid_turn_value = pid_turn_value * 2.5;

  // 提取第三个部分，去掉 '%'
  String thirdPart = data.substring(secondSpaceIdx + 1, percentIdx);
  color = thirdPart.toInt();  // 将第三部分转为整数

  // 输出调试信息
  Serial.print("First value (pid_x_value): ");
  Serial.println(pid_x_value);
  Serial.print("Second value (pid_turn_value): ");
  Serial.println(pid_turn_value);
  Serial.print("Third value (thirdValue): ");
  Serial.println(color);
}


void receiveOpenMVData() {
  while (Serial.available() > 0) {
    data += char(Serial.read());
  }

  if (data.length() > 1 && data.endsWith("%")) {
    handleOpenMVData(data);  
    data = "";  
  }
}
//接受超声波数据
float checkdistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  float duration = pulseIn(ECHO_PIN, HIGH);
  if (duration == 0) {
    return -1;  // 未检测到目标物
  }

  float distance = duration / 58.0;
  Serial.println(distance);
  return distance;
}
// 返回 1 或 0，根据距离与阈值的比较
int checkDistanceAndReturnValue() {
  // 获取当前距离
  float distance = checkdistance();
  
  // 如果没有检测到目标物，返回 0
  if (distance == -1) {
    return 1;  // 无效距离时返回 0
  }

  // 判断距离是否大于阈值，返回对应的值
  if (distance > DISTANCE_THRESHOLD) {
    return 1;  // 距离大于阈值时返回 1
  } else {
    return 0;  // 距离小于或等于阈值时返回 0
  }
}
int writeDistanceAndReturnValue(){
  // 获取当前距离
  float distance = checkdistance();
  
  // 如果没有检测到目标物，返回 0
  if (distance == -1) {
    return 1;  // 无效距离时返回 0
  }

  // 判断距离是否大于阈值，返回对应的值
  if (distance > WRITE_DISTANCE_THRESHOLD) {
    return 1;  // 距离大于阈值时返回 1
  } else {
    return 0;  // 距离小于或等于阈值时返回 0
  }
}
int backDistanceAndReturnValue() {
  // 获取当前距离
  float distance = checkdistance();
  
  // 如果没有检测到目标物，返回 0
  if (distance == -1) {
    return 0;  // 无效距离时返回 0
  }

  // 判断距离是否大于阈值，返回对应的值
  if (distance < BACK_DISTANCE_THRESHOLD) {
    return 1;  // 距离大于阈值时返回 1
  } else {
    return 0;  // 距离小于或等于阈值时返回 0
  }
}
// 通用舵机控制函数
void moveServo(Servo& servo, int angle, int delayTime = 600) {
  servo.write(angle);    // 设置舵机的目标角度
  delay(delayTime);      // 延迟一段时间，让舵机转动完成
}

// 封装的动作函数，只输入六个舵机的角度
void Servoreach(int angle_7, int angle_3, int angle_5, int angle_6, int angle_9 ,int angle_8) {
  moveServo(servo_7, angle_7);  // 控制舵机7转动到输入的角度
  moveServo(servo_3, angle_3);  // 控制舵机3转动到输入的角度
  moveServo(servo_5, angle_5);  // 控制舵机5转动到输入的角度
  moveServo(servo_6, angle_6);  // 控制舵机6转动到输入的角度
  moveServo(servo_9, angle_9);  // 控制舵机6转动到输入的角度
  moveServo(servo_8, angle_8);  // 控制舵机8转动到输入的角度
}

void readSerialDataForDuration(unsigned long startTime) {
  startTime = millis();  // 记录开始时间
  // 在持续时间内持续读取串口数据
  while (millis() - startTime < lasting_time) {
    receiveOpenMVData();
  }
}
void limitrunmotor(unsigned long startTime,unsigned long motor_lasting_time) {
  startTime = millis();  // 记录开始时间
  // 在持续时间内持续循迹
  while (millis() - startTime < motor_lasting_time) {
    receiveOpenMVData();
    motion.kongzhi_test();
  }
}
void backline(){
  int flag = backDistanceAndReturnValue();
    // 根据判断值控制电机
  if (flag == 1) {
    motion.moveBack(300,100);  // 距离大于阈值，启动电机
    backline();
  } else {
    motion.moveStop();  // 距离小于或等于阈值，停止电机
    delay(1000);
  }
}
void detect_color(){
  //执行识别姿态
  moveServo(servo_7, 23);  // 控制舵机7转动到输入的角度
  moveServo(servo_6, 155);  // 控制舵机6转动到输入的角度
  moveServo(servo_5, 110);  // 控制舵机5转动到输入的角度
  moveServo(servo_3, 85);  // 控制舵机3转动到输入的角度
  moveServo(servo_8, 100);  // 控制舵机8转动到输入的角度
  delay(500);
  backline();
  readSerialDataForDuration(startTime);
  //执行识别
  if (color == 1) {
    // 红色逻辑
    color_counter++;   
    redLogic();
  }
  else if (color == 2) {
    // 绿色逻辑
    color_counter++;
    greenLogic();   
  }
  else if (color == 3) {
    // 蓝色逻辑
    color_counter++;
    blueLogic();
  }
  else if(color == 0){
    Serial.println("Invalid color input!");    
    detect_color();    
  }
}


void grabbox(){
  moveServo(servo_8, 100);  // 控制舵机8转动到输入的角度
  moveServo(servo_7, 20);  // 控制舵机7转动到输入的角度
  moveServo(servo_6, 45);  // 控制舵机6转动到输入的角度
  moveServo(servo_3, 95);  // 控制舵机3转动到输入的角度
  moveServo(servo_5, 160);  // 控制舵机5转动到输入的角度
  moveServo(servo_8, 200);  // 控制舵机8转动到输入的角度
}
void loosbox_left(){  
  moveServo(servo_7, 150);  // 控制舵机7转动到输入的角度
  moveServo(servo_6, 50);  // 控制舵机6转动到输入的角度
  moveServo(servo_3, 100);  // 控制舵机3转动到输入的角度
  moveServo(servo_5, 147);  // 控制舵机5转动到输入的角度
  moveServo(servo_8, 100);  // 控制舵机8转动到输入的角度
}
void loosbox_right(){  
  moveServo(servo_7, 25);  // 控制舵机7转动到输入的角度
  moveServo(servo_6, 50);  // 控制舵机6转动到输入的角度
  moveServo(servo_3, 100);  // 控制舵机3转动到输入的角度  
  moveServo(servo_5, 147);  // 控制舵机5转动到输入的角度
  moveServo(servo_8, 100);  // 控制舵机8转动到输入的角度
}
void circleline(){
  receiveOpenMVData();
  int flag = checkDistanceAndReturnValue();
    // 根据判断值控制电机
  if (flag == 1) {
    motion.kongzhi_test();  // 距离大于阈值，启动电机
    circleline();
  } else {
    motion.moveStop();  // 距离小于或等于阈值，停止电机
    delay(1000);
  }
}
void writeline(){
  int flag = writeDistanceAndReturnValue();
    // 根据判断值控制电机
  if (flag == 1) {
    motion.kongzhi_test();  // 距离大于阈值，启动电机
    circleline();
  } else {
    motion.moveStop();  // 距离小于或等于阈值，停止电机
    delay(1000);
  }
}
//红色逻辑
void redLogic(){
  grabbox();//执行抓取
  delay(1000);
  setServoPositions();//回到巡线姿态
  delay(500);
  motion.rotate_right(300,2700);//(速度，时间)
  delay(500);
  motion.moveBack(300,500);
  limitrunmotor(startTime,lasting_time_red);//无超声波循迹

  //开启超声波循迹
    circleline();
    motion.moveBack(300,1000);
    motion.moveStop();  // 距离小于或等于阈值，停止电机
    loosbox_left();
    delay(500);
    setServoPositions();//回到巡线姿态
    delay(500);
    motion.rotate_right(300,2700);
    delay(200);
    motion.moveBack(300,500);
    limitrunmotor(startTime,lasting_time_red);//无超声波循迹  
    //超声波循迹
    circleline();
    motion.moveBack(300,1000);
    motion.moveStop();
    if(color_counter >=3){
      motion.rotate_left(300,1250);//左转九十度
      motion.moveStop();
      delay(500);
      write();
    }
    else{
      // 距离小于或等于阈值，停止电机
    detect_color();
    }   
};
void greenLogic(){
  grabbox();//执行抓取
  delay(1000);
  setServoPositions();//回到巡线姿态
  delay(500);
  motion.moveBack(300,500);
  limitrunmotor(startTime,lasting_time_green);//无超声波循迹;
  //开启超声波循迹
    circleline();
    motion.moveStop();  // 距离小于或等于阈值，停止电机
    motion.moveBack(300,1100);
    motion.moveStop();  // 距离小于或等于阈值，停止电机
    loosbox_right();
    delay(500);
    setServoPositions();//回到巡线姿态
    delay(500);
    motion.moveBack(300,500);
    limitrunmotor(startTime,lasting_time_green);//无超声波循迹;//无超声波循迹
    //超声波循迹
    circleline();
    motion.moveStop();  // 距离小于或等于阈值，停止电机
    motion.moveBack(300,1000);
    motion.moveStop();
    if(color_counter >= 3){
      motion.rotate_left(300,1250);//左转九十度
      motion.moveStop();
      delay(500);
      write();
    }
    else{
      // 距离小于或等于阈值，停止电机
    detect_color();
    }
};
void blueLogic(){
  grabbox();//执行抓取
  delay(1000);
  setServoPositions();//回到巡线姿态
  delay(500);
  motion.moveBack(300,500);
  delay(500);
  limitrunmotor(startTime,lasting_time_blue);//无超声波循迹;//无超声波循迹
  //开启超声波循迹
  circleline();
    motion.moveBack(300,1000);
    motion.moveStop();  // 距离小于或等于阈值，停止电机
    loosbox_right();
    delay(500);
    motion.moveBack(300,500);
    delay(500);
    setServoPositions();//回到巡线姿态
    delay(500);
    motion.rotate_right(300,2700);
     limitrunmotor(startTime,lasting_time_blue);//无超声波循迹;
    //超声波循迹
    circleline();
    motion.moveBack(300,1000);
    motion.moveStop();
    if(color_counter >= 3){
      motion.rotate_right(300,1250);//右转九十度
      motion.moveStop();
      delay(500);
      write();
    }
    else{
      // 距离小于或等于阈值，停止电机
    motion.rotate_right(300,2700);
    motion.moveStop();
    detect_color();
    }      
};
void write(){
  fixedEnd();
  delay(500);
  motion.time_moveForeward(300,4000);//走去写字台
  motion.moveStop();
  delay(500);
  setAngles(color);
  // 绘制字形
  drawStroke(angles_write1, number1+2);  // 第一笔
  delay(1000);
  drawStroke(angles_write2, number2+1);  // 第二笔
  delay(2000);
}
// 设置角度数据
void setAngles(int color) {
  switch (color) {
    case 1:
      memcpy(angles_write1, angles1, sizeof(angles1));
      memcpy(angles_write2, angles2, sizeof(angles2));
      break;
    case 2:
      memcpy(angles_write1, angles3, sizeof(angles3));
      memcpy(angles_write2, angles4, sizeof(angles4));
      break;
    case 3:
      memcpy(angles_write1, angles5, sizeof(angles5));
      memcpy(angles_write2, angles6, sizeof(angles6));
      break;
    default:
      memcpy(angles_write1, angles1, sizeof(angles1));
      memcpy(angles_write2, angles2, sizeof(angles2));
      break;
  }
}
void drawStroke(float angles[][3], int numAngles) {
  for (int i = 0; i < numAngles; i++) {
    servo_7.write(angles[i][0]);
    delay(10);
    servo_3.write(angles[i][1]);
    delay(10);
    servo_5.write(angles[i][2]);
    delay(10);
    if (i == 0) {
      delay(1500);
    }
    delay(300);
  }
}

// 固定末端
void fixedEnd() {
  servo_6.write(20);   // 固定为 20 度
  delay(500);
  servo_8.write(180);  // 固定为 180 度
  delay(500);  
  delay(100);  
}
void loop() {
  receiveOpenMVData();
  if (ra_counter == 0){
    limitrunmotor(startTime,lasting_time_1);//无超声波循迹
    ra_counter++;
  }    
  int flag = checkDistanceAndReturnValue();
    // 根据判断值控制电机
  if (flag == 1) {
    receiveOpenMVData();
    motion.kongzhi_test();  // 距离大于阈值，启动电机
  } else {
    motion.moveStop();  // 距离小于或等于阈值，停止电机
    delay(1000);
    detect_color();
    write();
    delay(10000);
  }
}

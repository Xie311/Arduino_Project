#include <MsTimer2.h>

// 电机引脚定义
#define ENCODER_A_L  2   // 左电机编码器A相
#define ENCODER_B_L  4
#define ENCODER_A_R  3   // 右电机编码器A相
#define ENCODER_B_R  5
#define PWML        11   // 左电机PWM
#define PWMR        12   // 右电机PWM
#define DIRL        6    // 左电机方向
#define DIRR        7    // 右电机方向

// 系统参数
#define PERIOD        20    // 控制周期（ms）
#define WHEEL_RADIUS  5.0f  // 轮子半径（cm）
#define WHEEL_BASE    2.0f  // 两轮间距（cm）
#define BLACK_THRESHOLD 400  // 红外黑线阈值

// 红外传感器引脚定义（假设使用8个红外传感器）
const byte IR_PINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// 全局变量
volatile float TARGET_L = 20;      // 左电机目标速度
volatile float TARGET_R = 20;      // 右电机目标速度
volatile float encoderVal_L = 0;   // 左编码器计数
volatile float encoderVal_R = 0;   // 右编码器计数
volatile float velocity_L = 0;     // 左轮速度
volatile float velocity_R = 0;     // 右轮速度
byte irValues[8] = {0};            // 红外传感器读数（二值化后）

// ================== 电机控制核心函数 ==================
void getEncoder_L() {
  encoderVal_L += (digitalRead(ENCODER_A_L) == digitalRead(ENCODER_B_L)) ? -1 : 1;
}

void getEncoder_R() {
  encoderVal_R += (digitalRead(ENCODER_A_R) == digitalRead(ENCODER_B_R)) ? -1 : 1;
}

void setMotorSpeed(int pinPWM, int pinDIR, float speed) {
  digitalWrite(pinDIR, (speed > 0) ? HIGH : LOW);
  analogWrite(pinPWM, constrain(abs(speed), 0, 255));
}

void updateMotors() {
  // 计算轮速（弧度/秒）
  velocity_L = (encoderVal_L/780) * TWO_PI * (1000.0/PERIOD);
  velocity_R = (encoderVal_R/780) * TWO_PI * (1000.0/PERIOD);
  
  // PID控制（简化版）
  static float lastError_L = 0, integral_L = 0;
  static float lastError_R = 0, integral_R = 0;
  
  float error_L = TARGET_L - velocity_L;
  float error_R = TARGET_R - velocity_R;
  
  integral_L = constrain(integral_L + error_L, -255, 255);
  integral_R = constrain(integral_R + error_R, -255, 255);
  
  float output_L = 8*error_L + 0.1*integral_L + 15*(error_L - lastError_L);
  float output_R = 8*error_R + 0.1*integral_R + 15*(error_R - lastError_R);
  
  lastError_L = error_L;
  lastError_R = error_R;
  
  // 设置电机
  setMotorSpeed(PWML, DIRL, output_L);
  setMotorSpeed(PWMR, DIRR, -output_R); // 右侧电机反向
  
  // 重置编码器计数
  encoderVal_L = encoderVal_R = 0;
}

// ================== 循迹控制逻辑 ==================
void setDifferentialDrive(float linear_vel, float angular_vel) {
  TARGET_L = (linear_vel - angular_vel * WHEEL_BASE/2) / WHEEL_RADIUS;
  TARGET_R = -(linear_vel + angular_vel * WHEEL_BASE/2) / WHEEL_RADIUS; // 右侧电机反向
}

void readIRSensors() {
  for (byte i = 0; i < 8; i++) {
    irValues[i] = (analogRead(IR_PINS[i]) > BLACK_THRESHOLD) ? 1 : 0;
  }
}

byte countBlackLines() {
  byte count = 0;
  for (byte i = 0; i < 8; i++) count += irValues[i];
  return count;
}

void lineFollowing() {
  readIRSensors();
  byte blackCount = countBlackLines();
  
  // 根据传感器状态调整运动
  if (blackCount <= 2) {
    // 正常循迹状态
    if (irValues[0])       setDifferentialDrive(10, -16);  // 极右偏转
    else if (irValues[1] || irValues[2]) setDifferentialDrive(12, -10);  // 右转
    else if (irValues[3] || irValues[4]) setDifferentialDrive(16, 0);    // 直行
    else if (irValues[7])       setDifferentialDrive(10, 16);   // 极左偏转
    else if (irValues[5] || irValues[6]) setDifferentialDrive(12, 10);   // 左转
    else setDifferentialDrive(10, 0);  // 默认直行
  } 
  else if (blackCount <= 4) {
    // 中等偏离状态
    if (irValues[0] || irValues[1] || irValues[2] || irValues[3]) {
      setDifferentialDrive(10, -16);  // 强制右转
    } 
    else if (irValues[4] || irValues[5] || irValues[6] || irValues[7]) {
      setDifferentialDrive(10, 16);   // 强制左转
    } 
    else {
      setDifferentialDrive(10, 0);    // 直行
    }
  }
  else if (blackCount <= 6) {
    // 严重偏离状态
    if (!irValues[6] && !irValues[7]) setDifferentialDrive(6, -16);  // 急右转
    else if (!irValues[0] && !irValues[1]) setDifferentialDrive(6, 16);   // 急左转
  }
  else if (blackCount == 8) {
    // 检测到终点
    setDifferentialDrive(0, 0);
  }
}

// ================== Arduino标准函数 ==================
void setup() {
  // 初始化电机控制引脚
  pinMode(PWML, OUTPUT); pinMode(DIRL, OUTPUT);
  pinMode(PWMR, OUTPUT); pinMode(DIRR, OUTPUT);
  
  // 初始化编码器引脚
  pinMode(ENCODER_A_L, INPUT); pinMode(ENCODER_B_L, INPUT);
  pinMode(ENCODER_A_R, INPUT); pinMode(ENCODER_B_R, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_L), getEncoder_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_R), getEncoder_R, CHANGE);
  
  // 初始化红外传感器引脚
  for (byte i = 0; i < 8; i++) pinMode(IR_PINS[i], INPUT);
  
  // 设置定时中断
  MsTimer2::set(PERIOD, updateMotors);
  MsTimer2::start();
  
  // 初始速度
  TARGET_L = TARGET_R = 20;
}

void loop() {
  lineFollowing();
  delay(5);  // 主循环延迟
}

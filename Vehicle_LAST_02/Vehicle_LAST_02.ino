#include <MsTimer2.h>

// 硬件引脚定义
#define ENCODER_A_L  2   // 左编码器A相
#define ENCODER_B_L  4   // 左编码器B相
#define ENCODER_A_R  3   // 右编码器A相
#define ENCODER_B_R  5   // 右编码器B相
#define PWML        11   // 左电机PWM
#define PWMR        12   // 右电机PWM
#define DIRL        6    // 左电机方向
#define DIRR        7    // 右电机方向

// 系统参数
#define PERIOD          20      // 控制周期(ms)
#define WHEEL_RADIUS    5.0f    // 轮半径(cm)
#define WHEEL_BASE      2.0f    // 轮距(cm)
#define BLACK_THRESHOLD 400     // 黑线阈值
#define MAX_SPEED       16      // 最大线速度(cm/s)
#define TURN_SPEED      12      // 转向速度(cm/s)
#define SHARP_TURN_SPEED 6      // 急转速度(cm/s)

// 红外传感器引脚
const byte IR_PINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// 全局变量
volatile float TARGET_L = 0;      // 左轮目标速度(rad/s)
volatile float TARGET_R = 0;      // 右轮目标速度(rad/s)
volatile long encoderVal_L = 0;   // 左编码器计数
volatile long encoderVal_R = 0;   // 右编码器计数
volatile float velocity_L = 0;    // 左轮实际速度(rad/s)
volatile float velocity_R = 0;    // 右轮实际速度(rad/s)
byte irValues[8] = {0};           // 红外传感器状态(0/1)

// ================ 编码器中断处理 ================
void getEncoder_L() {
  encoderVal_L += (digitalRead(ENCODER_A_L) == digitalRead(ENCODER_B_L)) ? -1 : 1;
}

void getEncoder_R() {
  encoderVal_R += (digitalRead(ENCODER_A_R) == digitalRead(ENCODER_B_R)) ? -1 : 1;
}

// ================ 电机控制 ================
void setMotor(int pwmPin, int dirPin, float speed) {
  digitalWrite(dirPin, (speed > 0) ? HIGH : LOW);
  analogWrite(pwmPin, constrain(abs(speed), 0, 255));
}

void updateMotors() {
  // 计算实际速度(rad/s)
  static const float countsPerRev = 780.0f;
  static const float radPerCount = TWO_PI / countsPerRev;
  static const float timeFactor = 1000.0f / PERIOD;
  
  velocity_L = encoderVal_L * radPerCount * timeFactor;
  velocity_R = encoderVal_R * radPerCount * timeFactor;
  
  // 简单PID控制
  static float lastErr_L = 0, integral_L = 0;
  static float lastErr_R = 0, integral_R = 0;
  
  float err_L = TARGET_L - velocity_L;
  float err_R = TARGET_R - velocity_R;
  
  integral_L = constrain(integral_L + err_L, -255, 255);
  integral_R = constrain(integral_R + err_R, -255, 255);
  
  float output_L = 8*err_L + 0.1*integral_L + 15*(err_L - lastErr_L);
  float output_R = 8*err_R + 0.1*integral_R + 15*(err_R - lastErr_R);
  
  lastErr_L = err_L;
  lastErr_R = err_R;
  
  // 驱动电机
  setMotor(PWML, DIRL, output_L);
  setMotor(PWMR, DIRR, -output_R); // 右侧电机反向
  
  // 重置编码器
  encoderVal_L = encoderVal_R = 0;
}

// ================ 运动控制 ================
void setDifferentialDrive(float linear_vel, float angular_vel) {
  // 计算左右轮速度(rad/s)
  TARGET_L = (linear_vel - angular_vel * WHEEL_BASE/2) / WHEEL_RADIUS;
  TARGET_R = -(linear_vel + angular_vel * WHEEL_BASE/2) / WHEEL_RADIUS; // 右侧反向
}

// ================ 循迹逻辑 ================
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
  
  // 状态机处理不同情况
  switch(blackCount) {
    case 0: // 完全脱线(保持上次动作)
      break;
      
    case 1: // 单侧轻微偏离
      if (irValues[0]) setDifferentialDrive(10, -14); // 极右偏
      else if (irValues[7]) setDifferentialDrive(10, 14); // 极左偏
      break;
      
    case 2: // 正常循迹状态
      if (irValues[0] || irValues[1]) setDifferentialDrive(TURN_SPEED, -6); // 右转
      else if (irValues[6] || irValues[7]) setDifferentialDrive(TURN_SPEED, 6); // 左转
      else if (irValues[3] || irValues[4]) setDifferentialDrive(MAX_SPEED, 0); // 直行
      else setDifferentialDrive(10, 0); // 默认直行
      break;
      
    case 3: case 4: // 中等偏离
      if (irValues[0] || irValues[1] || irValues[2]) 
        setDifferentialDrive(TURN_SPEED, -12); // 强制右转
      else if (irValues[5] || irValues[6] || irValues[7]) 
        setDifferentialDrive(TURN_SPEED, 12); // 强制左转
      else 
        setDifferentialDrive(10, 0); // 直行
      break;
      
    case 5: case 6: // 严重偏离
      if (!irValues[6] && !irValues[7]) 
        setDifferentialDrive(SHARP_TURN_SPEED, -12); // 急右转
      else if (!irValues[0] && !irValues[1]) 
        setDifferentialDrive(SHARP_TURN_SPEED, 12); // 急左转
      break;
      
    case 7: // 即将完全脱线
      if (!irValues[7]) setDifferentialDrive(6, -12); // 急右转
      else if (!irValues[0]) setDifferentialDrive(6, 12); // 急左转
      break;
      
    case 8: // 检测到终点
      setDifferentialDrive(0, 0);
      break;
  }
}

// ================ 主程序 ================
void setup() {
  // 初始化电机控制引脚
  pinMode(PWML, OUTPUT); pinMode(DIRL, OUTPUT);
  pinMode(PWMR, OUTPUT); pinMode(DIRR, OUTPUT);
  
  // 初始化编码器引脚
  pinMode(ENCODER_A_L, INPUT); pinMode(ENCODER_B_L, INPUT);
  pinMode(ENCODER_A_R, INPUT); pinMode(ENCODER_B_R, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_L), getEncoder_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_R), getEncoder_R, CHANGE);
  
  // 初始化红外传感器
  for (byte i = 0; i < 8; i++) pinMode(IR_PINS[i], INPUT);
  
  // 设置定时中断
  MsTimer2::set(PERIOD, updateMotors);
  MsTimer2::start();
  
  // 初始速度
  setDifferentialDrive(10, 0);
}

void loop() {
  lineFollowing();
  
  // 调试输出
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    Serial.print("L:"); Serial.print(velocity_L);
    Serial.print(" R:"); Serial.println(velocity_R);
    lastPrint = millis();
  }
  
  delay(5);
}

#include <PID_v1_bc.h>
#include <MsTimer2.h>

// 编码器引脚定义
#define ENCODER_A_L 2
#define ENCODER_B_L 4
#define ENCODER_A_R 3
#define ENCODER_B_R 5

// 电机驱动引脚
#define PWML 11
#define DIR_L 6
#define PWMR 12
#define DIR_R 7

// 红外传感器引脚定义（数字输入）
const int irSensorPins[8] = { A0, A1, A2, A3, A4, A5, A6, A7 };  // 从左到右

// 控制参数
#define PERIOD 20  // 控制周期(ms)
volatile int encoderVal_L = 0;
volatile int encoderVal_R = 0;
volatile float velocity_L = 0;
volatile float velocity_R = 0;
double Setpoint = 0;
volatile bool inSharpTurn = false;
volatile unsigned long sharpTurnStartTime = 0;

// 速度PID参数
float L_kp = 12.0, L_ki = 0.8, L_kd = 0.2;
float R_kp = 12.0, R_ki = 0.8, R_kd = 0.2;
volatile float L_integral = 0, L_lastError = 0;
volatile float R_integral = 0, R_lastError = 0;

// 循线PID参数
double lineKp = 2.0, lineKi = 0.03, lineKd = 0.25;
double lineInput, lineOutput;
PID linePID(&lineInput, &lineOutput, &Setpoint, lineKp, lineKi, lineKd, DIRECT);

// 速度参数
int baseSpeed = 1.8;  // 基础速度(0-255)
int maxSpeed = 3.0;   // 最大速度
int turnSpeed = 1.2;  // 转弯时基础速度
int sharpTurnSpeed = 1.0; // 直角转弯速度

// 传感器状态
volatile int sensorValues[8];
volatile int lastValidValues[8] = { 0 };
volatile bool allSensorsZero = false;
volatile int lastLineError = 0;

void setup() {
  // 初始化电机引脚
  pinMode(DIR_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(PWML, OUTPUT);
  pinMode(PWMR, OUTPUT);

  // 初始化编码器引脚
  pinMode(ENCODER_A_L, INPUT_PULLUP);
  pinMode(ENCODER_B_L, INPUT_PULLUP);
  pinMode(ENCODER_A_R, INPUT_PULLUP);
  pinMode(ENCODER_B_R, INPUT_PULLUP);

  // 初始化红外传感器引脚
  for (int i = 0; i < 8; i++) {
    pinMode(irSensorPins[i], INPUT_PULLUP);
  }
  
  inSharpTurn = false;
  
  // 设置PWM频率（减少电机噪声）
  TCCR1B = TCCR1B & 0b11111000 | 0x01;  // 设置Timer1 PWM频率为31.25kHz
  
  // 初始化中断
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_L), encoderL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_R), encoderR_ISR, CHANGE);

  // 初始化PID
  linePID.SetMode(AUTOMATIC);
  linePID.SetOutputLimits(-200, 200);
  linePID.SetSampleTime(PERIOD);

  // 初始化定时器中断
  MsTimer2::set(PERIOD, controlLoop);
  MsTimer2::start();
  delay(500);
  readSensors();
  Serial.begin(115200);
}

void loop() {
  // 主循环不执行具体控制，仅用于调试显示
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 200) {
    debugPrint();
    lastPrintTime = millis();
  }
}

// 编码器中断服务程序
void encoderL_ISR() {
  if (digitalRead(ENCODER_A_L) == digitalRead(ENCODER_B_L)) {
    encoderVal_L--;
  } else {
    encoderVal_L++;
  }
}

void encoderR_ISR() {
  if (digitalRead(ENCODER_A_R) == digitalRead(ENCODER_B_R)) {
    encoderVal_R++;
  } else {
    encoderVal_R--;
  }
}

// 定时器中断服务程序（控制核心）
void controlLoop() {
  // 1. 读取传感器
  readSensors();
  
  // 2. 检测直角转弯
  bool newSharpTurn = detectSharpTurn();
  if(newSharpTurn && !inSharpTurn) {
    sharpTurnStartTime = millis();
  }
  inSharpTurn = newSharpTurn && (millis() - sharpTurnStartTime < 1000); // 直角转弯状态最多持续1秒
  
  // 3. 计算循线误差
  calculateLineError();

  // 4. 动态速度调整
  int speedAdjustment = adjustSpeed();

  // 5. 计算目标速度（考虑转向）
  float target_L = speedAdjustment - lineOutput;
  float target_R = speedAdjustment + lineOutput;

  // 6. 速度PID控制
  velocityControl(target_L, target_R);
}

void readSensors() {
  // 读取所有传感器值（检测到线=1）
  allSensorsZero = true;
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = !digitalRead(irSensorPins[i]);  // 反转逻辑，检测到线=1
    if (sensorValues[i]) {
      allSensorsZero = false;
      lastValidValues[i] = sensorValues[i];
    }
  }
}

bool detectSharpTurn() {
  // 改进的直角转弯检测逻辑
  const int threshold = 3; // 至少3个连续传感器触发
  
  // 检测左侧直角
  int leftCount = 0;
  for(int i = 0; i < 4; i++) {
    if(sensorValues[i]) leftCount++;
    else leftCount = 0;
    if(leftCount >= threshold) return true;
  }
  
  // 检测右侧直角
  int rightCount = 0;
  for(int i = 7; i >= 4; i--) {
    if(sensorValues[i]) rightCount++;
    else rightCount = 0;
    if(rightCount >= threshold) return true;
  }
  
  // 检测T型交叉
  bool leftHalf = sensorValues[0] && sensorValues[1] && sensorValues[2] && sensorValues[3];
  bool rightHalf = sensorValues[4] && sensorValues[5] && sensorValues[6] && sensorValues[7];
  
  return leftHalf || rightHalf;
}

void calculateLineError() {
  // 改进的加权误差计算
  const int weights[8] = {-70, -40, -20, -5, 5, 20, 40, 70};
  const int sharpWeights[8] = {-100, -60, -30, -5, 5, 30, 60, 100};
  
  double error = 0;
  int activeSensors = 0;
  
  if(inSharpTurn) {
    // 直角转弯模式：强化半区权重
    for(int i = 0; i < 8; i++) {
      if(sensorValues[i]) {
        error += sharpWeights[i];
        activeSensors++;
      }
    }
  } else {
    // 正常模式
    for(int i = 0; i < 8; i++) {
      if(sensorValues[i]) {
        error += weights[i];
        activeSensors++;
      }
    }
  }

  if(activeSensors == 0) {
    // 全零时使用最后有效误差方向
    lineInput = (lastLineError > 0) ? 8.0 : -8.0;
  } else {
    lineInput = error / (activeSensors * 10.0);  // 归一化误差
    lastLineError = lineInput;
  }

  // 应用低通滤波减少噪声
  static double filteredError = 0;
  filteredError = 0.7 * filteredError + 0.3 * lineInput;
  lineInput = filteredError;
  
  linePID.Compute();
}

int adjustSpeed() {
  // 改进的动态速度调整
  int detected = 0;
  for(int i = 0; i < 8; i++) {
    if(sensorValues[i]) detected++;
  }

  if(inSharpTurn) {
    return sharpTurnSpeed;
  } else if(detected >= 6) {
    return min(baseSpeed + 30, maxSpeed);  // 直线加速
  } else if(detected >= 3) {
    return baseSpeed;                     // 正常速度
  } else {
    return turnSpeed;                     // 弯道减速
  }
}

void velocityControl(float target_L, float target_R) {
  // 计算实际速度（脉冲/周期 -> rad/s）
  static int lastEncoder_L = 0, lastEncoder_R = 0;
  velocity_L = (encoderVal_L - lastEncoder_L) * (2 * 3.1416 / 780.0) * (1000.0 / PERIOD);
  velocity_R = (encoderVal_R - lastEncoder_R) * (2 * 3.1416 / 780.0) * (1000.0 / PERIOD);
  lastEncoder_L = encoderVal_L;
  lastEncoder_R = encoderVal_R;

  // 改进的PID控制算法
  float L_error = target_L - velocity_L;
  L_integral = constrain(L_integral + L_error, -100, 100); // 积分限幅
  float L_derivative = L_error - L_lastError;
  int L_pwm = L_kp * L_error + L_ki * L_integral + L_kd * L_derivative;
  L_pwm = constrain(L_pwm, -255, 255);

  float R_error = target_R - velocity_R;
  R_integral = constrain(R_integral + R_error, -100, 100); // 积分限幅
  float R_derivative = R_error - R_lastError;
  int R_pwm = R_kp * R_error + R_ki * R_integral + R_kd * R_derivative;
  R_pwm = constrain(R_pwm, -255, 255);

  // 设置电机输出
  setMotor(2*L_pwm, 2*R_pwm);

  // 更新误差记录
  L_lastError = L_error;
  R_lastError = R_error;
}

void setMotor(int L_pwm, int R_pwm) {
  // 改进的电机控制，增加死区处理
  const int deadZone = 15;
  
  // 左电机控制
  if(abs(L_pwm) > deadZone) {
    digitalWrite(DIR_L, L_pwm > 0 ? HIGH : LOW);
    analogWrite(PWML, constrain(abs(L_pwm), deadZone, 255));
  } else {
    analogWrite(PWML, 0);
  }

  // 右电机控制
  if(abs(R_pwm) > deadZone) {
    digitalWrite(DIR_R, R_pwm > 0 ? LOW : HIGH);  // 注意电机方向
    analogWrite(PWMR, constrain(abs(R_pwm), deadZone, 255));
  } else {
    analogWrite(PWMR, 0);
  }
}

void debugPrint() {
  // 改进的调试输出
  Serial.print("Sensors: ");
  for(int i = 0; i < 8; i++) {
    Serial.print(sensorValues[i] ? "■" : "□");
  }

  Serial.print(" | V_L: ");
  Serial.print(velocity_L, 2);
  Serial.print(" V_R: ");
  Serial.print(velocity_R, 2);

  Serial.print(" | Err: ");
  Serial.print(lineInput, 2);
  Serial.print(" Out: ");
  Serial.print(lineOutput, 2);

  Serial.print(" | Mode: ");
  Serial.print(inSharpTurn ? "SHARP" : "NORM ");
  
  Serial.println();
}

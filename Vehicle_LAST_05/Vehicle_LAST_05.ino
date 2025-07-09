#include <MsTimer2.h>
#include <Servo.h>
#define SERVO_A_BASE 90
#define SERVO_B_BASE 50
#define SERVO_A_DOWN 180
#define SERVO_B_PICK 180
Servo servoA, servoB;
#define SERVO_A_PIN 8   //俯仰
#define SERVO_B_PIN 10  //爪子
#define SERVO_C_PIN 9   //旋转
#define ENCODER_A1 2    //电机 1
#define ENCODER_B1 4
#define ENCODER_A2 3  //电机 2
#define ENCODER_B2 5
#define PWM1 11
#define PWM2 12
#define DIR_LEFT 6   // 左侧电机方向控制
#define DIR_RIGHT 7  // 右侧电机方向控制

// 8路红外传感器定义 (A7-A0)
#define L8 A7  // 最左侧
#define L7 A6
#define L6 A5
#define L5 A4
#define R4 A3  // 中间右侧（与L5相邻）
#define R3 A2
#define R2 A1
#define R1 A0  // 最右侧

#define PERIOD 50  // 控制周期50ms
#define Kp 3.0     // 比例系数
#define Ti 0.1     // 积分系数
#define Td 0.05    // 微分系数
int judge = 1;
float velocity1;
float velocity2;
long encoderVal1;
long encoderVal2;
int output1;
int output2;
float target1 = 15.0;
float target2 = 15.0;
#define V 30 // 基础速度值

// 预测机制相关变量
int lost_count = 0;                    // 连续丢线计数器
float last_nonzero_target1 = 15.0;     // 上一次非零转向值（左轮）
float last_nonzero_target2 = 15.0;     // 上一次非零转向值（右轮）

// 转弯状态检测变量
int turn_direction = 0;            // 0=直行, 1=左转, -1=右转
int consecutive_turns = 0;         // 连续转弯计数
unsigned long last_turn_time = 0;  // 上次转弯时间
const int TURN_THRESHOLD = 2;      // 连续转弯阈值
const int TURN_TIMEOUT = 300;      // 转弯超时时间(ms)

// 直角弯检测
bool sharp_turn_detected = false;
unsigned long sharp_turn_start = 0;
const int SHARP_TURN_DURATION = 200; // 直角弯持续时间

// 转向平滑控制
float turn_smoothing_factor = 1.0;  // 转向平滑系数 (1.0=无平滑)

void Arm_setup(Servo *servoA, Servo *servoB) {
  servoA->attach(SERVO_A_PIN);
  servoB->attach(SERVO_B_PIN);
  servoA->write(SERVO_A_BASE);
  servoB->write(SERVO_B_BASE);
}

void Arm_pickup(Servo *servoA, Servo *servoB) {
  servoA->write(SERVO_A_DOWN);
  delay(750);
  servoB->write(SERVO_B_PICK);
  delay(500);
  servoA->write(SERVO_A_BASE);
}

void Arm_drop(Servo *servoA, Servo *servoB) {
  servoA->write(SERVO_A_DOWN);
  delay(500);
  servoB->write(SERVO_B_BASE);
  delay(500);
  servoA->write(SERVO_A_BASE);
  delay(500);
}

// 简化的转弯状态检测
void detectTurnState() {
  // 根据目标速度差判断转向
  float speed_diff = target2 - target1;

  // 检测直角弯（急转弯）
  if (abs(speed_diff) > 20.0) {
    if (!sharp_turn_detected) {
      sharp_turn_detected = true;
      sharp_turn_start = millis();
    }
  } else {
    // 检查直角弯是否结束
    if (sharp_turn_detected && (millis() - sharp_turn_start > SHARP_TURN_DURATION)) {
      sharp_turn_detected = false;
    }
  }

  // 简化转向响应
  turn_smoothing_factor = 1.0;  // 保持正常转向，不进行平滑调整
}

void control(void) {
  // 读取所有8个红外传感器
  bool l8 = digitalRead(L8);
  bool l7 = digitalRead(L7);
  bool l6 = digitalRead(L6);
  bool l5 = digitalRead(L5);
  bool r4 = digitalRead(R4);
  bool r3 = digitalRead(R3);
  bool r2 = digitalRead(R2);
  bool r1 = digitalRead(R1);

  // 默认直行
  float base_speed = 15.0;
  target1 = base_speed;
  target2 = base_speed;

  // 检查是否所有传感器都未检测到黑线（丢线状态）
  bool all_off = !(l8 || l7 || l6 || l5 || r4 || r3 || r2 || r1);

  if (all_off) {
    // 丢线状态 - 使用预测机制增强转向
    lost_count++;

    // 简化预测机制
    if (lost_count > 2) {
      // 使用上一次的转向值继续运动
      target1 = last_nonzero_target1;
      target2 = last_nonzero_target2;

      // 限制速度在合理范围内
      if (target1 > 30) target1 = 30;
      if (target1 < -30) target1 = -30;
      if (target2 > 30) target2 = 30;
      if (target2 < -30) target2 = -30;
    }
  } else {
    // 有传感器检测到线 - 重置丢线计数器
    lost_count = 0;

    // ==== 传感器处理逻辑 (优先级从高到低) ====

    // 检测多传感器组合情况（可能是直角弯）
    int active_sensors = l8 + l7 + l6 + l5 + r4 + r3 + r2 + r1;

    // 1. 优先处理最外侧传感器 - 直角弯/急转弯
    if (l8) {  // 极左位置 - 激进右转
      target1 = -10.0;
      target2 = 35.0;
    } else if (r1) {  // 极右位置 - 激进左转
      target1 = 35.0;
      target2 = -10.0;
    }
    // 2. 处理次外侧传感器 - 大角度转弯
    else if (l7) {  // 次左外
      target1 = 5.0;
      target2 = 25.0;
    } else if (r2) {  // 次右外
      target1 = 25.0;
      target2 = 5.0;
    }
    // 3. 处理内侧传感器 - 常规转弯
    else if (l6) {  // 左侧检测
      target1 = 8.0;
      target2 = 22.0;
    } else if (r3) {  // 右侧检测
      target1 = 22.0;
      target2 = 8.0;
    }
    // 4. 处理最内侧传感器
    else if (l5) {  // 左中检测
      target1 = 12.0;
      target2 = 18.0;
    } else if (r4) {  // 右中检测
      target1 = 18.0;
      target2 = 12.0;
    }
    // 5. 中间传感器都检测到或直行
    else {
      target1 = 15.0;
      target2 = 15.0;
    }

    // 记录非零转向值用于预测
    if (target1 != base_speed || target2 != base_speed) {
      last_nonzero_target1 = target1;
      last_nonzero_target2 = target2;
    }
  }

  // 简化：不调用复杂的转弯检测，保持基础控制

  // 简化速度计算
  velocity1 = encoderVal1 * 0.1;  // 简化的速度计算
  encoderVal1 = 0;
  velocity2 = encoderVal2 * 0.1;  // 简化的速度计算
  encoderVal2 = 0;

  // 电机控制
  output1 = pidcontroller1(target1, velocity1);
  if (output1 > 0) {
    digitalWrite(DIR_LEFT, HIGH);
  } else {
    digitalWrite(DIR_LEFT, LOW);
  }
  analogWrite(PWM1, abs(output1));

  output2 = pidcontroller2(target2, velocity2);
  if (output2 > 0) {
    digitalWrite(DIR_RIGHT, LOW);
  } else {
    digitalWrite(DIR_RIGHT, HIGH);
  }
  analogWrite(PWM2, abs(output2));
}

int pidcontroller1(float targetVelocity, float currentVelocity) {
  static float previous_error1 = 0;
  static float sum1 = 0;
  float error1 = targetVelocity - currentVelocity;
  sum1 += error1;

  // 限制积分饱和
  if (sum1 > 100) sum1 = 100;
  if (sum1 < -100) sum1 = -100;

  float derivative1 = error1 - previous_error1;
  float u1 = Kp * error1 + Ti * sum1 + Td * derivative1;
  previous_error1 = error1;

  if (u1 > 255) u1 = 255;
  if (u1 < -255) u1 = -255;
  return (int)u1;
}

int pidcontroller2(float targetVelocity, float currentVelocity) {
  static float previous_error2 = 0;
  static float sum2 = 0;
  float error2 = targetVelocity - currentVelocity;
  sum2 += error2;

  // 限制积分饱和
  if (sum2 > 100) sum2 = 100;
  if (sum2 < -100) sum2 = -100;

  float derivative2 = error2 - previous_error2;
  float u2 = Kp * error2 + Ti * sum2 + Td * derivative2;
  previous_error2 = error2;

  if (u2 > 255) u2 = 255;
  if (u2 < -255) u2 = -255;
  return (int)u2;
}

void getEncoder1(void) {
  if (digitalRead(ENCODER_A1) == LOW) {
    if (digitalRead(ENCODER_B1) == LOW) {
      encoderVal1++;
    } else {
      encoderVal1--;
    }
  } else {
    if (digitalRead(ENCODER_B1) == LOW) {
      encoderVal1--;
    } else {
      encoderVal1++;
    }
  }
}

void getEncoder2(void) {
  if (digitalRead(ENCODER_A2) == LOW) {
    if (digitalRead(ENCODER_B2) == LOW) {
      encoderVal2++;
    } else {
      encoderVal2--;
    }
  } else {
    if (digitalRead(ENCODER_B2) == LOW) {
      encoderVal2--;
    } else {
      encoderVal2++;
    }
  }
}

void setup() {
  TCCR1B = TCCR1B & B11111000 | B00000001;

  // 先初始化引脚，再设置舵机
  pinMode(DIR_LEFT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);
  pinMode(ENCODER_A1, INPUT);
  pinMode(ENCODER_B1, INPUT);
  pinMode(ENCODER_A2, INPUT);
  pinMode(ENCODER_B2, INPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);

  // 初始化红外传感器
  pinMode(L8, INPUT);
  pinMode(L7, INPUT);
  pinMode(L6, INPUT);
  pinMode(L5, INPUT);
  pinMode(R4, INPUT);
  pinMode(R3, INPUT);
  pinMode(R2, INPUT);
  pinMode(R1, INPUT);

  // 停止电机
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);

  Arm_setup(&servoA, &servoB);
  delay(1000);  // 等待舵机稳定
  Arm_pickup(&servoA, &servoB);
  delay(500);



  MsTimer2::set(PERIOD, control);
  MsTimer2::start();

  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), getEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2), getEncoder2, CHANGE);

  Serial.begin(9600);
}

void loop() {
  while (judge == 1) {
    // 检测是否到达终点（所有传感器都检测到黑线）
    if (digitalRead(L8) && digitalRead(L7) && digitalRead(L6) && digitalRead(L5) && digitalRead(R4) && digitalRead(R3) && digitalRead(R2) && digitalRead(R1)) {
      MsTimer2::stop();

      // 停止电机
      analogWrite(PWM1, 0);
      analogWrite(PWM2, 0);
      delay(200);

      // 放下物体
      Arm_drop(&servoA, &servoB);
      judge = 0;
    }
  }

  // 调试输出
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();

    Serial.print("Targets: ");
    Serial.print(target1);
    Serial.print(", ");
    Serial.print(target2);
    Serial.print(" | Outputs: ");
    Serial.print(output1);
    Serial.print(", ");
    Serial.print(output2);
    Serial.print(" | Lost: ");
    Serial.print(lost_count);
    Serial.print(" | Vel: ");
    Serial.print(velocity1);
    Serial.print(",");
    Serial.print(velocity2);
    Serial.print(" | Sensors: ");
    Serial.print(digitalRead(L8));
    Serial.print(digitalRead(L7));
    Serial.print(digitalRead(L6));
    Serial.print(digitalRead(L5));
    Serial.print(digitalRead(R4));
    Serial.print(digitalRead(R3));
    Serial.print(digitalRead(R2));
    Serial.println(digitalRead(R1));
  }
}

#include <MsTimer2.h> //定时器库的 头文件
//定义电机编码器 A B 相引脚
#define ENCODER_A 2 //编码器 A 项，使用中断 0，非常重要不要接错，用于中断触发
#define ENCODER_B 4 //编码器 B 项，使用 Arduino 的一个普通数字 I/0 引脚
//定义电机驱动的控制信号
#define PWML 10 //用于电机的 PWM 输出，调节电机速度
#define INL1 12 //11、12 引脚用于电机的转动方向控制
#define INL2 11
#define PERIOD 20 //定时周期设置
#define TARGET -20 //定义电机的目标速度，即给定值
//全局变量
volatile long encoderVal; //编码器值
float velocity;
float u = 0;
float eI;
float eII;
float eIII;
//获取编码器值
void getEncoder(void)
{
if (digitalRead(ENCODER_A) == LOW)
{
if (digitalRead(ENCODER_B) == LOW)
{
encoderVal--;
}
else
{
encoderVal++;
}
}
else
{
if (digitalRead(ENCODER_B) == LOW)
{
encoderVal++;
}
else
{
encoderVal--;
}
}
}
//PID 控制器，初始值供参考，Kp=500，Ti=10，Td=400，T=PERIOD
int pidController(float targetVelocity, float currentVelocity)
{
float output;
eI = targetVelocity - currentVelocity;
float Kp = 5, Ti = 140, Td = 80, T = PERIOD;
float q0 = Kp * (1 + T / Ti + Td / T);
float q1 = -Kp * (1 + 2 * Td / T);
float q2 = Kp * Td / T;
u = u + q0 * eI + q1 * eII + q2 * eIII;
eIII = eII;
eII = eI;
if (u >= 255)
{
u = 255;
}
if (u <= -255)
{
u = -255;
}
output = u;
return (int)output;
}
//直流电机控制
void control(void)
{
velocity= (encoderVal_R/780) * 3.1415 * 2.0 * (1000/PERIOD); 
encoderVal = 0;
/*计算轮子转动速度（弧度制的速度 2πRn）*/
int output = pidController(TARGET, velocity);
//TARGET 为目标速度值，目前大家的电机基本上最快为每秒转 50 弧度，所以不要设置超
过这个值
//以下代码很重要，大家可能需要根据电机实际转动，调整 INL1 和 INL2 的值
if (output > 0)
{
digitalWrite(INL1, LOW);
digitalWrite(INL2, HIGH);
analogWrite(PWML, output);
}
else
{
digitalWrite(INL1, HIGH);
digitalWrite(INL2, LOW);
analogWrite(PWML, abs(output));
}
}
void setup()
{
//PWM 频率调节，设置 9、10 引脚的 PWM 输出频率为 31372Hz，适合于我们使用的电机
TCCR1B = TCCR1B & B11111000 | B00000001;
//初始化各管脚，各全局变量，中断，串口通讯等
pinMode(PWML, OUTPUT);
pinMode(INL2, OUTPUT);
pinMode(INL1, OUTPUT);
pinMode(ENCODER_A, INPUT);
pinMode(ENCODER_B, INPUT);
attachInterrupt(0, getEncoder, CHANGE); //中断 0 设置，对应 2 引脚
Serial.begin(9600);
MsTimer2::set(PERIOD, control); //设定每隔 PERIOD 时间，执行一次 control
MsTimer2::start();
}
void loop()
{
//撰写代码，使其能够通过“串口绘图器”观察速度波形
Serial.print("velocity:");
Serial.print(velocity);
Serial.print("\r\n");
}

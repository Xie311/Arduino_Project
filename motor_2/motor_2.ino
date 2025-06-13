#include <MsTimer2.h>
#define ENCODER_A_L 2 //电机 1 的编码器 A 项接 Arduino 的 2 中断口，用于编码器计数
#define ENCODER_B_L 4
#define ENCODER_A_R 3 //电机 2 的编码器 A 项接 Arduino 的 3 中断口，用于编码器计数
#define ENCODER_B_R 5
#define PWML 9 //用于电机 1 的 PWM 输出，调节电机速度
#define INL1 12 //11 和 12 引脚用于电机 1 的转动方向控制
#define INL2 11 
#define PWMR 10 //用于电机 2 的 PWM 输出，调节电机速度
#define INR1 6 //6 和 8 引脚用于电机 2 的转动方向控制
#define INR2 8
#define PERIOD 20
volatile float TARGET_L = 15; //定义电机 1 的目标速度值
volatile float TARGET_R = 10; //定义电机 2 的目标速度值
volatile float encoderVal_L; //在中断里面使用的全局变量需要定义成 volatile 类型
volatile float encoderVal_R; 
volatile float velocity_L;
volatile float velocity_R;
volatile float uL=0;
volatile float uR=0;
volatile float LeI; //电机 1 当前时刻的误差 e(k)
volatile float LeII; //上一时刻的误差 e(k-1)
volatile float LeIII; //上上时刻的误差 e(k-2)
volatile int Loutput;
volatile float ReI; //电机 2 当前时刻的误差 e(k)
volatile float ReII; //上一时刻的误差 e(k-1)
volatile float ReIII; //上上时刻的误差 e(k-2)
volatile int Routput;
//获取电机 1 的编码器值
void getEncoder_L(void){
 if(digitalRead(ENCODER_A_L) == LOW){
 if (digitalRead(ENCODER_B_L) == LOW){
 encoderVal_L--;
 }
 else{
 encoderVal_L++;
 }
 }
 else{
 if (digitalRead(ENCODER_B_L) == LOW){
 encoderVal_L++;
 }
 else{
 encoderVal_L--;
 }
 }
}
//获取电机 2 的编码器值
void getEncoder_R(void){
 if(digitalRead(ENCODER_A_R) == LOW){
 if (digitalRead(ENCODER_B_R) == LOW){
 encoderVal_R--;
 }
 else{
 encoderVal_R++;
 }
 }
 else{
 if (digitalRead(ENCODER_B_R) == LOW){
 encoderVal_R++;
 }
 else{
 encoderVal_R--;
 }
 }
}
//电机 1 的 PID 算法
int pidcontrol_L(float target,float current)
{
 LeI = target - current;
 float kp = 6, TI =100 , TD =15,T = PERIOD;
 float q0 = kp * (1+T/TI + TD/T);
 float q1 = -kp * (1+2*TD/T);
 float q2 = kp * TD / T;
 uL = uL + q0 * LeI + q1 * LeII + q2 * LeIII;
 LeIII = LeII;
 LeII = LeI;
 if (uL > 255){
 uL = 255; 
 }
 else if (uL <= -255){
 uL = -255;
 }
 Loutput = uL;
 return (int)Loutput;
}
//电机 2 的 PID 算法
int pidcontrol_R(float target,float current)
{
 ReI = target - current;
 float kp = 6, TI =100 , TD =15,T = PERIOD;
 float q0 = kp * (1+T/TI + TD/T);
 float q1 = -kp * (1+2*TD/T);
 float q2 = kp * TD / T;
 uR = uR + q0 * ReI + q1 * ReII + q2 * ReIII;
 ReIII = ReII;
 ReII = ReI;
 if (uR > 255){
 uR = 255; 
 }
 else if (uR <= -255){
 uR = -255;
 }
 Routput = uR;
 return (int)Routput;
}
//电机 1 速度和方向控制
void control_L()
{
 velocity_L = (encoderVal_L/780) * 3.1415 * 2.0 * (1000/PERIOD); //计算轮子转动速度（弧
//度制的速度，2πRn）
 Loutput = pidcontrol_L(TARGET_L,velocity_L);
 if (Loutput > 0){
 digitalWrite(INL1,LOW);
 digitalWrite(INL2,HIGH);
 analogWrite(PWML,Loutput);
 }
 else {
 digitalWrite(INL1,HIGH);
 digitalWrite(INL2,LOW);
 analogWrite(PWML,abs(Loutput));
 }
 encoderVal_L = 0;
 
}
//电机 2 速度和方向控制
void control_R()
{
 velocity_R = (encoderVal_R/780) * 3.1415 * 2.0 * (1000/PERIOD); //计算轮子转动速度（弧
//度制的速度 2πRn）
 Routput = pidcontrol_R(TARGET_R,velocity_R);
 if (Routput > 0){
 digitalWrite(INR1,LOW);
 digitalWrite(INR2,HIGH);
 analogWrite(PWMR,Routput);
 }
 else {
 digitalWrite(INR1,HIGH);
 digitalWrite(INR2,LOW);
 analogWrite(PWMR,abs(Routput));
 }
 encoderVal_R = 0;
 
}
//封装函数
void control(void)
{
 control_L();
 control_R();
}
// put your setup code here, to run once
void setup() {
 TCCR1B = TCCR1B & B11111000 | B00000001; //PWM 频率调节，设置 9、10 引脚的 PWM 输
// 出频率为 31372Hz，适合于我们使用的电机
 pinMode(PWML,OUTPUT);
 pinMode(INL2,OUTPUT);
 pinMode(INL1,OUTPUT);
 pinMode(PWMR,OUTPUT);
 pinMode(INR2,OUTPUT);
 pinMode(INR1,OUTPUT);
 
 pinMode(ENCODER_A_L,INPUT);
 pinMode(ENCODER_B_L,INPUT);
 
 pinMode(ENCODER_A_R,INPUT);
 pinMode(ENCODER_B_R,INPUT);
 
 attachInterrupt(0,getEncoder_L,CHANGE); //中断 0 设置，对应 2 引脚
 attachInterrupt(1,getEncoder_R,CHANGE); //中断 1 设置，对应 3 引脚
 Serial.begin(9600);
 MsTimer2::set(PERIOD,control); //设定每隔 PERIOD 时间，执行一次 control
 MsTimer2::start(); //开始时间
}
// put your main code here, to run repeatedly
void loop() {
//可以在串口监视器中显示两条线
 Serial.print("velocity_1:");
 Serial.print(velocity_L);
 Serial.print("\t");
  Serial.print("velocity_2:");
 Serial.print(velocity_R);
 Serial.print("\t\n");
 
//Serial.print("ENCODER_R:");
 //Serial.print(encoderVal_R);
 //Serial.print("\n");
 //Serial.print("ReI:");
 //Serial.print(ReI);
 // Serial.print("\n");
 // Serial.print("ROUTPUT:");
 //Serial.print(Routput);
 //Serial.print("\n");
 
 //Serial.print("velocity_R:");
 //Serial.print(velocity_R);
 //Serial.print("\r\n");
}

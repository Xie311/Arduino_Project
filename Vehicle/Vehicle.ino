#include <MsTimer2.h> 
#define ENCODER_A_L  2  //电机 1 的编码器A项接Arduino 的2中断口，用于编码器计数 
#define ENCODER_B_L  4 
#define ENCODER_A_R  3  //电机 2 的编码器A项接Arduino 的3中断口，用于编码器计数 
#define ENCODER_B_R  5 
#define PWML 11  //用于电机1的PWM 输出，调节电机速度  
#define PWMR 12  //用于电机2的PWM输出，调节电机速度 
#define DIRL 6   //6 和7引脚用于电机的转动方向控制 
#define DIRR 7    

#define PERIOD 20
#define WHEEL_RADIUS 5.0f    // 轮子半径（cm）
#define WHEEL_BASE 2.0f      // 两轮间距（cm）
#define rIr0 0
#define rIr1 1
#define rIr2 2
#define rIr3 3
#define rIr4 4
#define rIr5 5
#define rIr6 6
#define rIr7 7
#define BLACK 1


//typedef struct {
//    float KP;        // PID参数P
//    float KI;        // PID参数I
//    float KD;        // PID参数D
//    float fdb;       // PID反馈值
//    float ref;       // PID目标值
//    float cur_error; // 当前误差
//    float error[2];  // 前两次误差
//    float output;    // 输出值
//    float outputMax; // 最大输出值的绝对值
//    float outputMin; // 最小输出值的绝对值用于防抖
//
//    float integral;//add by zyt
//} PID_t;


int r[8];
volatile float TARGET_L = 20;  //定义电机 1 的目标速度值 
volatile float TARGET_R = 20;  //定义电机 2 的目标速度值 
volatile float encoderVal_L;   
//在中断里面使用的全局变量需要定义成volatile类型 
volatile float encoderVal_R;  
volatile float velocity_L; 
volatile float velocity_R; 
volatile float uL=0; 
volatile float uR=0; 
volatile float LeI;   
volatile float LeII;   
//电机1当前时刻的误差e(k) 
//上一时刻的误差e(k-1) 
volatile float LeIII;  //上上时刻的误差 e(k-2) 
volatile int Loutput; 
volatile float ReI;   
volatile float ReII;   
//电机2当前时刻的误差e(k) 
//上一时刻的误差e(k-1) 
volatile float ReIII;  //上上时刻的误差 e(k-2) 
volatile int Routput; 

//获取电机1的编码器值 
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

//获取电机2的编码器值 
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


//void PID_Calc_P(__IO PID_t *pid)
//{
//    pid->cur_error = pid->ref - pid->fdb;
//    pid->integral += pid->cur_error;
//    pid->output = pid->KP * pid->cur_error + pid->KI * pid->integral + pid->KD * (pid->cur_error - pid->error[1]);
//    pid->error[0] = pid->error[1];  //这句已经没有用了
//    pid->error[1] = pid->ref - pid->fdb;
//    /*设定输出上限*/
//    if (pid->output > pid->outputMax) pid->output = pid->outputMax;
//    if (pid->output < -pid->outputMax) pid->output = -pid->outputMax;
//}


//电机1的PID算法 
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



//电机2的PID算法 
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

//电机1速度和方向控制 
void control_L() 
{ 
velocity_L = (encoderVal_L/780) * 3.1415 * 2.0 * (1000/PERIOD);   
Loutput = pidcontrol_L(TARGET_L,velocity_L); 
if (Loutput > 0){ 
digitalWrite(DIRL,HIGH); 
analogWrite(PWML,Loutput); 
} 
else { 
digitalWrite(DIRL,LOW); 
analogWrite(PWML,abs(Loutput)); 
} 
encoderVal_L = 0; 
//计算轮子转动速度（弧度制的速度，2πRn） 
} 

//电机2速度和方向控制 
void control_R() 
{ 
velocity_R = (encoderVal_R/780) * 3.1415 * 2.0 * (1000/PERIOD);   
Routput = pidcontrol_R(TARGET_R,velocity_R); 
if (Routput > 0){ 
digitalWrite(DIRR,HIGH); 
analogWrite(PWMR,Routput); 
} 
else { 
digitalWrite(DIRR,LOW); 
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

//两轮差速平台运算
//linear_vel：直线前进速度 单位：cm/s(maybe)
//angular_vel：角速度 单位：rad/s
void setDifferentialDrive(float linear_vel, float angular_vel) {
    // 计算左右轮速度
    float left_speed = linear_vel - (angular_vel * WHEEL_BASE / 2.0f);
    float right_speed = linear_vel + (angular_vel * WHEEL_BASE / 2.0f);
    TARGET_L=left_speed/WHEEL_RADIUS;
    TARGET_R=-right_speed/WHEEL_RADIUS;
}


// put your setup code here, to run once 
void setup() { 
//串口设置
Serial.begin(115200);
//红外，读取模拟口
analogReference(DEFAULT);
//pinMode(rIr0,INPUT);
//pinMode(rIr1,INPUT);
//pinMode(rIr2,INPUT);
//pinMode(rIr3,INPUT);
//pinMode(rIr4,INPUT);
//pinMode(rIr5,INPUT);
//pinMode(rIr6,INPUT);
//pinMode(rIr7,INPUT);

//计算轮子转动速度（弧度制的速度2πRn） 
TCCR1B = TCCR1B & B11111000 | B00000001; //PWM 频率调节，设置 9、10 引脚的 PWM输出频率为31372Hz，适合于我们使用的电机 
pinMode(PWML,OUTPUT); 
pinMode(DIRR,OUTPUT); 
pinMode(DIRL,OUTPUT); 
pinMode(PWMR,OUTPUT); 
pinMode(ENCODER_A_L,INPUT); 
pinMode(ENCODER_B_L,INPUT); 
pinMode(ENCODER_A_R,INPUT); 
pinMode(ENCODER_B_R,INPUT); 
attachInterrupt(0,getEncoder_L,CHANGE); //中断 0 设置，对应 2 引脚 
attachInterrupt(1,getEncoder_R,CHANGE); //中断 1 设置，对应 3 引脚 
Serial.begin(9600); 
MsTimer2::set(PERIOD,control);  //设定每隔 PERIOD 时间，执行一次 control 
MsTimer2::start(); //开始时间 
} 
// put your main code here, to run repeatedly 
void loop() { 

//判断红外的情况
r[0]=analogRead(rIr0);
r[1]=analogRead(rIr1);
r[2]=analogRead(rIr2);
r[3]=analogRead(rIr3);
r[4]=analogRead(rIr4);
r[5]=analogRead(rIr5);
r[6]=analogRead(rIr6);
r[7]=analogRead(rIr7);
for(int i=0;i<8;i++){
  if(r[i]>400){
    r[i]=1;
  }
  else r[i]=0;
}

  //Serial.println(r[0]);
//使用setDifferentialDrive(float linear_vel, float angular_vel)
//来调转方向之类的

if (r[0] == BLACK && r[1] != BLACK && r[2] != BLACK && r[3] != BLACK && r[4] != BLACK && r[5] != BLACK && r[6] != BLACK && r[7] != BLACK)
    {
        setDifferentialDrive(6,-12);//01111只剩最右边压线了，要最大量的往右边转
    }
    else if (r[0] != BLACK && (r[1] == BLACK || r[2] == BLACK )&& r[3] != BLACK && r[4] != BLACK && r[5] != BLACK && r[6] != BLACK && r[7] != BLACK)
    {
        setDifferentialDrive(6,-6);//
    }
    else if (r[0] != BLACK && r[1]!= BLACK && r[2] != BLACK && (r[3] == BLACK || r[4] == BLACK) && r[5] != BLACK && r[6] != BLACK && r[7] != BLACK)
    {
        setDifferentialDrive(6,0);//直行
    }
    else if (r[0] != BLACK && r[1] != BLACK && r[2] != BLACK && r[3] != BLACK && r[4] != BLACK && r[5] != BLACK && r[6] != BLACK && r[7] == BLACK)
    {
        setDifferentialDrive(6,12);//01111只剩最左边压线了，要最大量的往左边转
    }
    else if (r[0] != BLACK && r[1] != BLACK && r[2] != BLACK && r[3] != BLACK && r[4] != BLACK && (r[5] == BLACK || r[6] == BLACK) && r[7] != BLACK)
    {
        setDifferentialDrive(6,6);//
    }
    else
    {
        setDifferentialDrive(6,0);//直行
    }
    
delay(50);

//可以在串口监视器中显示两条线 
Serial.print("velocity_L:"); 
Serial.print(velocity_L); 
Serial.print("\t"); 
Serial.print("velocity_R:"); 
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

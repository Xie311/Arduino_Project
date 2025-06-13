#include <Servo.h>   
#define PIN_SERVO1 10   //定义舵机控制端口 
#define PIN_SERVO2 9
#define PIN_SERVO3 8  
Servo myservo1;      
Servo myservo2;      
Servo myservo3;      
  //创建一个舵机控制对象   
void setup() {   
// put your setup code here, to run once:   
// Serial.begin(9600);   //设置串口波特率，目的是将数据传到电脑串口监控器   
  myservo1.attach(PIN_SERVO1);   //定义舵机接口 10                                                                         
  myservo2.attach(PIN_SERVO2);   //定义舵机接口 10   
  myservo3.attach(PIN_SERVO3);   //定义舵机接口 10   
}   
void loop() {   
 // put your main code here, to run repeatedly:   
int angle1, angle2, angle3;   
  

angle1 = 0; //0~180   
angle2=40;     // 40度 平 顺时针
angle3=180;    // 90度嵌入舵机座，角度变大，向上抬升
myservo1.write(angle1);      
myservo2.write(angle2);      
myservo3.write(angle3);      
  //设置舵机旋转角度   
 
delay(100);   
}  

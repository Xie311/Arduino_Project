#include<MsTimer2.h>
#include <Servo.h>
#define SERVO_A_BASE 0
#define SERVO_B_BASE 0
#define SERVO_A_DOWN 45
#define SERVO_B_PICK 50
Servo servoA, servoB;
#define SERVO_A_PIN 51 //俯仰
#define SERVO_B_PIN 49 //爪子
#define SERVO_C_PIN 53 //旋转
#define ENCODER_A1 2 //电机 1
#define ENCODER_B1 5
#define ENCODER_A2 3 //电机 2
#define ENCODER_B2 4
#define PWM1 9
#define PWM2 10
#define IN1 11
#define IN2 8
#define IN3 13
#define IN4 12
#define L1 38 //左红外
#define L2 36
#define L3 34
#define R1 46 //右红外
#define R2 32
#define R3 42
#define M 40 //中红外
#define PERIOD 5
#define Kp 65
#define Ti 2.5
#define Td 15
int judge=1;
float velocity1;
float velocity2;
long encoderVal1;
long encoderVal2; 
int output1;
int output2;
float target1=10;
float target2=10;
#define V 14.0
void Arm_setup(Servo *servoA, Servo *servoB)
   { 
   servoA->attach(SERVO_A_PIN);
   servoB->attach(SERVO_B_PIN);
   servoA->write(SERVO_A_BASE);
   servoB->write(SERVO_B_BASE);
   }

void Arm_pickup(Servo *servoA, Servo *servoB)
  {
  servoA->write(SERVO_A_DOWN);
  delay(750);
  servoB->write(SERVO_B_PICK);
  delay(500);
  servoA->write(-SERVO_A_DOWN);
}

void Arm_drop(Servo *servoA, Servo *servoB)
{
  servoA->write(SERVO_A_DOWN);
  delay(500);
  servoB->write(SERVO_B_BASE);
  delay(500);
  servoA->write(SERVO_A_BASE);
  delay(500);
}
void control(void){
  if (digitalRead(M) == HIGH)//直行
  {
   target1 = V;
   target2 = V;
  }
   if (digitalRead(L3) == HIGH) //低左转
  {
   target1 = V * 0.70;
   target2 = V * 1.00;
  }
  if (digitalRead(R3) == HIGH) //低右转
  {
   target1 = V * 1.00;
   target2 = V * 0.70;
  }
   if (digitalRead(L2) == HIGH) //中左转
  {
   target1 = V * 0.40;
   target2 = V * 1.00;
  }
  if (digitalRead(R2) == HIGH) //中右转
  {
   target1 = V * 1.00;
   target2 = V * 0.40;
  }
  if(digitalRead(L1) == HIGH)
  {
    target1 = V * 0.1;
    target2 = V * 1.20;
  }
  if(digitalRead(R1) == HIGH)
  {
    target1 = V * 1.50;
    target2 = V * 0.1;
  }
  
  velocity1 = (encoderVal1 / 780.0) * 3.1415 * 2.0 * (1000 / PERIOD);
  encoderVal1 = 0;
  velocity2 = (encoderVal2 / 780.0) * 3.1415 * 2.0 * (1000 / PERIOD);
  encoderVal2 = 0;
  output1=pidcontroller1(target1,velocity1);
  if(output1>0){
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    analogWrite(PWM1,output1);
  }
  else{
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite(PWM1,abs(output1));
  }
  analogWrite(PWM1,abs(output1));
  output2=pidcontroller2(target2,velocity2);
  if(output2>0){
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
    analogWrite(PWM2,output2);
  }
  else{
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
    analogWrite(PWM2,abs(output2));
  }
  analogWrite(PWM2,abs(output2));
  }
int pidcontroller1(float targetVelocity,float currentVelocity){
  float error1;
  float previous_error1;
  float sum1;
  float derivative1;
  float u1;
  error1=targetVelocity-currentVelocity;
  sum1=sum1+error1;
  derivative1= error1-previous_error1;
  u1=Kp*error1+Ti*sum1+Td*derivative1;
  previous_error1=error1;
  if (u1 > 255)
  {
   u1 = 255;
  }
  if (u1 < -255)
  {
   u1 = -255;
  }
  return (int)u1;
}
int pidcontroller2(float targetVelocity,float currentVelocity){
  float error2;
  float previous_error2;
  float sum2;
  float derivative2;
  float u2;
  error2=targetVelocity-currentVelocity;
  sum2=sum2+error2;
  derivative2= error2-previous_error2;
  u2=Kp*error2+Ti*sum2+Td*derivative2;
  previous_error2=error2;
  if (u2 > 255)
  {
   u2 = 255;
  }
  if (u2 < -255)
  {
   u2 = -255;
  }
  return (int)u2;
}
void getEncoder1(void)
  {
    if (digitalRead(ENCODER_A1) == LOW)
    {
     if (digitalRead(ENCODER_B1) == LOW)
     {
       encoderVal1++;
     }
     else
     {
       encoderVal1--;
     }
    }
    else
    {
      if (digitalRead(ENCODER_B1) == LOW)
      {
        encoderVal1--;
      }
      else
      {
        encoderVal1++;
      }
    }
   }
void getEncoder2(void)
   {
     if (digitalRead(ENCODER_A2) == LOW)
     {
      if (digitalRead(ENCODER_B2) == LOW)
      {
        encoderVal2--;
      }
      else
      {
        encoderVal2++;
      }
     }
     else
     {
       if (digitalRead(ENCODER_B2) == LOW)
       { 
        encoderVal2++;
       }
       else
       {
        encoderVal2--;
       }
      }
   }

void setup() {
  TCCR1B = TCCR1B & B11111000 | B00000001;
  Arm_setup(&servoA, &servoB);
  Arm_pickup(&servoA, &servoB);
  delay(500);
  MsTimer2::set(PERIOD, control);
  MsTimer2::start();
  pinMode(ENCODER_A1, INPUT);
  pinMode(ENCODER_B1, INPUT);
  pinMode(ENCODER_A2, INPUT);
  pinMode(ENCODER_B2, INPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  attachInterrupt(0, getEncoder1, CHANGE);
  attachInterrupt(1, getEncoder2, CHANGE);
  Serial.begin(9600);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(R1, INPUT);
  pinMode(R2, INPUT);
  pinMode(R3, INPUT);
  pinMode(L1, INPUT);
  pinMode(L2, INPUT);
  pinMode(L3, INPUT);
  pinMode(M, INPUT);
}
void loop() {
 while(judge == 1){
    if((digitalRead(M)==HIGH)&&(digitalRead(R3)==HIGH)&&(digitalRead(L3)==HIGH)&&(digitalRead(L2)==HIGH)&&(digitalRead(R2)==HIGH))
  {
  MsTimer2::stop();
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,HIGH);
  delay(200);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  digitalWrite(PWM1,LOW);
  digitalWrite(PWM2,HIGH);
  delay(300);
  digitalWrite(PWM1,LOW);
  digitalWrite(PWM2,LOW);
  delay(1000);
  Arm_drop(&servoA,&servoB);
  judge = 0;
  }
  }
}
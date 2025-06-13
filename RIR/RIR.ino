#define rIr0 0
#define rIr1 1
#define rIr2 2
#define rIr3 3
#define rIr4 4
#define rIr5 5
#define rIr6 6
#define rIr7 7
int r[8];

void setup() {
  Serial.begin(115200);
  analogReference(DEFAULT);
//  pinMode(rIr0,INPUT);
//  pinMode(rIr1,INPUT);
//  pinMode(rIr2,INPUT);
//  pinMode(rIr3,INPUT);
//  pinMode(rIr4,INPUT);
//  pinMode(rIr5,INPUT);
//  pinMode(rIr6,INPUT);
//  pinMode(rIr7,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
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

  Serial.println(r[0],DEC);
}

/*
 * 適当にコメントを修正した版
*/
#include <FlexiTimer2.h>
#include <BalanceCar.h>
#include <KalmanFilter.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

//TB6612FNG
#define IN1M 6
#define IN2M 7
#define IN3M 12
#define IN4M 13
#define PWMA 4
#define PWMB 5
#define STBY 8


#define PinA_right 18  //INT5
#define PinA_left 2    //INT0

//Bluetooth通信向け
#define  Left_Enoder     0x01
#define  Right_Enoder    0x02
#define  Left_PWM        0x03
#define  Right_PWM       0x04
#define  Balance_Angle   0x05
#define  Upright_Kp      0x06
#define  Upright_Ki      0x07
#define  Upright_Kd      0x08
#define  Speed_Kp        0x09
#define  Speed_Ki        0x10
#define  Speed_Kd        0x0A
#define  Rotate_Kp       0x0B
#define  Rotate_Ki       0x0C
#define  Rotate_Kd       0x0D
#define  Contrl_val      0x0E
#define  angle_output    0x0F
#define  speed_output    0x11

MPU6050 mpu; 
BalanceCar balancecar;
KalmanFilter kalmanfilter;
int16_t ax, ay, az, gx, gy, gz;

double Outputs = 0;
double kp = 29.525, ki = 0.0, kd = 0.2765;
double kp_speed = 5.0225, ki_speed = 0.1528, kd_speed = 0.0;
double kp_turn = 23.1625, ki_turn = 0, kd_turn = 0.277;

double setp0 = 0; 


//********************angle data*********************//
float K1 = 0.05;
float angle0 = 1.928;
//********************angle data*********************//


//***************Kalman_Filter*********************//
float Q_angle = 0.001, Q_gyro = 0.005;
float R_angle = 0.5 , C_0 = 1;
float timeChange = 5;
float dt = timeChange * 0.001;    //dt=フィルタのサンプリング時間
//***************Kalman_Filter*********************//


//*********************************************
//******************** speed count ************
//*********************************************
volatile long count_right = 0;  //意図とした最適化されない様にVolatile宣言
volatile long count_left = 0;   //意図とした最適化されない様にVolatile宣言
int speedcc = 0;
int timer = 0;

//////////////////////pulse/////////////////////////
int lz = 0;
int rz = 0;
int rpluse = 0;
int lpluse = 0;
int sumam;
/////////////////////pulse////////////////////////////

//////////////ステアリング、回転パラーメータ///////////////////////////////
int turncount = 0; //ステアリング介入時間
float turnoutput = 0;
//////////////ステアリング、回転パラメータ///////////////////////////////

//////////////Bluetooth///////////////////
int front = 0;  //前進
int back = 0;   //後退
int turnl = 0;  //左ターン
int turnr = 0;  //右ターン
int spinl = 0;  //左旋回
int spinr = 0;  //右旋回
//////////////Bluetooth///////////////////


////////////////Bluetoothトランシーバー////////////
unsigned char a[12]={0xAA,0xAA,0xAA,0xAA,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF};
unsigned char RxBuf[12]={0};
unsigned int Ctrl_val;
int Speed_val = 85;        //直線速度
char i=0;

void Uart_Send(unsigned char function,unsigned int value)
{
  unsigned int temp;
  i=0;
  temp=value;
  a[5]=function;
  a[6]=(value>>8) & 0xFF;
  a[7]=temp & 0xFF;
  while(i!=12)
  {
    Serial.write(a[i]);
    i++;
  }
}

void Uart_Recieve()
{
  if(Serial.available()>=12)
  {
    i=0;
    while(i!=12)
    {
      RxBuf[i]=Serial.read();
      i++;
    }
    Ctrl_val=RxBuf[6]*0x100+RxBuf[7];
    switch(RxBuf[5])
    {
      case Upright_Kp : kp=(double)Ctrl_val/1000;break;         
      case Upright_Ki :break;           
      case Upright_Kd : kd=(double)Ctrl_val/1000;break;
      case Speed_Kp :kp_speed=(double)Ctrl_val/1000;break;       
      case Speed_Ki :ki_speed=(double)Ctrl_val/1000;break;     
      case Speed_Kd :break;
      case Rotate_Kp :kp_turn=(double)Ctrl_val/1000;break;
      case Rotate_Ki :break;
      case Rotate_Kd :kd_turn=(double)Ctrl_val/100;break;
      case Contrl_val:
        switch(RxBuf[7])
        {
          case 0x01: front = Speed_val;   break;                                      //前進
          case 0x02: back = -Speed_val;   break;                                      //後退
          case 0x03: turnl = 1;front = 0;back = 0;   break;                           //左周り
          case 0x04: turnr = 1;front = 0;back = 0;   break;                           //右回り
          case 0x05: spinl = 1;front = 0;back = 0;   break;                           //左旋回
          case 0x06: spinr = 1;front = 0;back = 0;break;                              //右旋回
          case 0x07: spinl = 0; spinr = 0;  front = 0; back = 0;  turnl = 0; turnr = 0;  break;                   //ボタンが離され、操作停止
          case 0x08: front = 0; back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0; break;      //ボタンが離され、操作停止
          default: front = 0; back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0; break;
        }break;
      default :break;
    }
    while(Serial.read()>= 0){}
  }
}
///////////Bluetoothトランシーバー//////////////




//////////////////////Pulse///////////////////////
void countpluse()
{

  lz = count_left;
  rz = count_right;
  
  count_left = 0;
  count_right = 0;

  lpluse = lz;
  rpluse = rz;

  if ((balancecar.pwm1 < 0) && (balancecar.pwm2 < 0))                     //後退時(PWM値が負のとき)はパルス数が負
  {
    rpluse = -rpluse;
    lpluse = -lpluse;
  }
  else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 > 0))                //前進時（PWM値が正のとき)はパルス数が正
  {
    rpluse = rpluse;
    lpluse = lpluse;
  }
  else if ((balancecar.pwm1 < 0) && (balancecar.pwm2 > 0))                //右旋回？
  {
    rpluse = -rpluse;
    lpluse = lpluse;
  }
  else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 < 0))                //左旋回？
  {
    rpluse = rpluse;
    lpluse = -lpluse;
  }

  //停止判断？
  balancecar.stopr += rpluse;
  balancecar.stopl += lpluse;

  //PULSEは5ms単位で更新される
  balancecar.pulseright += rpluse;
  balancecar.pulseleft += lpluse;
  sumam=balancecar.pulseright+balancecar.pulseleft;
}
////////////////////Pulse///////////////////////


//////////////////角度PD////////////////////
void angleout()
{
  balancecar.angleoutput = -kp * (kalmanfilter.angle + angle0) - kd * kalmanfilter.Gyro_x;
}
//////////////////角度PD////////////////////

//////////////////////////////////////////////////////////
//////////////////5ms割り込み////////////////////
/////////////////////////////////////////////////////////
void inter()
{
  sei();                                           
  countpluse();
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     //I2C MPU6050から6つのパラメータ取得 ax ay az gx gy gz
  kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro,R_angle,C_0,K1);
  angleout();

  
  speedcc++;
  if (speedcc >= 8)                                //40ms単位で処理スピードをコントロール
  {
    Outputs = balancecar.speedpiout(kp_speed,ki_speed,kd_speed,front,back,setp0);
    speedcc = 0;
  }
  turncount++;
  if (turncount > 4)                                //20ms単位で回転制御
  {
    turnoutput = balancecar.turnspin(turnl,turnr,spinl,spinr,kp_turn,kd_turn,kalmanfilter.Gyro_z);
    turncount = 0;
  }
  balancecar.posture++;
  balancecar.pwma(Outputs,turnoutput,kalmanfilter.angle,kalmanfilter.angle6,turnl,turnr,spinl,spinr,front,back,kalmanfilter.accelz,IN1M,IN2M,IN3M,IN4M,PWMA,PWMB);  //PWM出力
  
  timer++;
}
//////////////////////////////////////////////////////////
//////////////////5msタイマー割込み///////////////////
/////////////////////////////////////////////////////////

/////////////////BT///////////////////////////
void Bluetooth()
{
  if(timer==200)
  {
    Uart_Send(Left_Enoder,(unsigned int)lz);
    delay(2);
    Uart_Send(Right_Enoder,(unsigned int)rz);
    delay(2);
    Uart_Send(Left_PWM,abs(balancecar.pwm2));
    delay(2);
    Uart_Send(Right_PWM,abs(balancecar.pwm1));
    delay(2);
    Uart_Send(Balance_Angle,(unsigned int)kalmanfilter.angle*10);
    delay(2);
    Uart_Send(angle_output,(unsigned int)balancecar.angleoutput*10);        //PID角度
    delay(2);
    Uart_Send(speed_output,(unsigned int)Outputs*10);  
    timer=0;
  }
}

// ===    初期設定     ===
void setup() {
  // TB6612
  pinMode(IN1M, OUTPUT);                        //モーター１方向制御 01:正回転　10:逆回転
  pinMode(IN2M, OUTPUT);
  pinMode(IN3M, OUTPUT);                        //モーター２方向制御 01:正回転　10:逆回転
  pinMode(IN4M, OUTPUT);
  pinMode(PWMA, OUTPUT);                        //左モーター PWM出力
  pinMode(PWMB, OUTPUT);                        //右モーター PWM出力
  pinMode(STBY, OUTPUT);                        //TB6612FNG


  //モータードライバの初期化
  digitalWrite(IN1M, 0);
  digitalWrite(IN2M, 1);
  digitalWrite(IN3M, 1);
  digitalWrite(IN4M, 0);
  digitalWrite(STBY, 1);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  pinMode(PinA_left, INPUT);  //モーターエンコーダー割込み
  pinMode(PinA_right, INPUT);
  

    // I2C設定
  Wire.begin();
  Serial.begin(115200);
  delay(500);
  Serial.println("AT+NAMESainSmart");
  delay(1000);
  
  mpu.initialize();                       //MPU6050初期化
  delay(2);
  balancecar.pwm1 =0;
  balancecar.pwm2 =0;

 //timer2を使って5ms割込みを行う 注意：PWMはタイマを使用してでユーティサイクルを制御するため、Timer2を使用すると9番、10番ピンのPWM出力に影響する。そのため、タイマを使用する際は対応するタイマのピンの注意！
  FlexiTimer2::set(5, inter);    //5ms
  FlexiTimer2::start();

}


////////////////////////////////////////turn//////////////////////////////////



// ===       MAINLOOP       ===
void loop() {
  
  attachInterrupt(5, Code_right, CHANGE);
  attachInterrupt(0, Code_left, CHANGE);
  Uart_Recieve();
  Bluetooth();
}

////////////////////////////////////////pwm///////////////////////////////////



//////////////////////////PULSE/////////////////////////////////////

void Code_left() {

  count_left ++;

} //左



void Code_right() {

  count_right ++;

} //右

//////////////////////////PULSE/////////////////////////////////////

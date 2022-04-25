#include <PinChangeInt.h>
#include <MsTimer2.h>
#include <BalanceCar.h>
#include <KalmanFilter.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

MPU6050 mpu;
BalanceCar balancecar;
KalmanFilter kalmanfilter;
int16_t ax, ay, az, gx, gy, gz;
#define speak 3 //语音模块
#define IN1M 7
#define IN2M 6
#define IN3M 13
#define IN4M 12
#define PWMA 9
#define PWMB 10
#define STBY 8        // TB66612FNG开启（高电平）
#define PinA_left 2   //左轮A相编码
//#define PinB_left 5   //左轮B相编码
#define PinA_right 4  //右轮A相编码
//#define PinB_right 11 //右轮B相编码

//////////////前进 后退 转弯控制位/////////////////////
enum
{
  enSTOP = 0,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  enTLEFT,
  enTRIGHT,
  enspeak
};

/////////////////////蓝牙//////////////////////
byte inByte;             //串口接收字节
#define run_car '1'      //按键 前
#define back_car '2'     //按键 后
#define left_car '3'     //按键 左
#define right_car '4'    //按键 右
#define stop_car '0'     //按键 停
#define speak '5'        //按键 说话
int g_carstate = enSTOP; //  1前2后3左4右0停止
int incomingByte = 0;    // 接收到的 data byte
String inputString = ""; // 用来储存接收到的内容
int front = 0;           //前进变量
int back = 0;            //后退变量
int turnl = 0;           //左转标志
int turnr = 0;           //右转标志
int spinl = 0;           //左旋转标志
int spinr = 0;           //右旋转标志

int time;
int num;
double Setpoint;                                                 //角度DIP设定点，输入，输出
double Setpoints, Outputs = 0;                                   //速度DIP设定点，输入，输出
double kp = 35, ki = 0.0, kd = 0.505;                            //需要你修改的参数
double kp_speed = 3.4, ki_speed = 0.11, kd_speed = 0.0;          // 需要你修改的参数
double kp_turn = 28, ki_turn = 0, kd_turn = 0.29;                //旋转PID设定
const double PID_Original[6] = {35, 0.0, 0.505, 3.4, 0.11, 0.0}; //恢复默认PID参数
double setp0 = 0, dpwm = 0, dl = 0;                              //角度平衡点，PWM差，死区，PWM1，PWM2,转向PID参数
float value;
bool flag = false;                                               //是否启动小车标志位

////////////////////////角度数据///////////////////////////
float Q;
float Angle_ax; //由加速度计算的倾斜角度
float Angle_ay;
float K1 = 0.05;     // 对加速度计取值的权重
float angle0 = 0.00; //机械平衡角
int slong;

///////////////////////Kalman滤波/////////////////////////
float Q_angle = 0.001, Q_gyro = 0.005; //角度数据置信度,角速度数据置信度
float R_angle = 0.5, C_0 = 1;
float timeChange = 5;          //滤波法采样时间间隔毫秒
float dt = timeChange * 0.001; //注意：dt的取值为滤波器采样时间
volatile long count_right = 0;
volatile long count_left = 0;
int speedcc = 0;

//////////////////////脉冲计算/////////////////////////
int lz = 0;
int rz = 0;
int rpluse = 0;
int lpluse = 0;
int sumam;

//////////////////转向、旋转参数///////////////////////////
int turncount = 0; //转向介入时间计算
float turnoutput = 0;
boolean newLineReceived = false; //前一次数据结束标志
boolean startBit = false;        //协议开始标志
String returntemp = "";          //存储返回值
boolean g_autoup = false;
int g_uptimes = 5000;

//////////////////////脉冲计算///////////////////////
void countpluse()
{
  lz = count_left;
  rz = count_right;
  count_left = 0;
  count_right = 0;
  lpluse = lz;
  rpluse = rz;
  if ((balancecar.pwm1 < 0) && (balancecar.pwm2 < 0)) //小车运动方向判断 后退时（PWM即电机电压为负） 脉冲数为负数
  {
    rpluse = -rpluse;
    lpluse = -lpluse;
  }
  else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 > 0)) //小车运动方向判断 前进时（PWM即电机电压为正） 脉冲数为负数
  {
    rpluse = rpluse;
    lpluse = lpluse;
  }
  else if ((balancecar.pwm1 < 0) && (balancecar.pwm2 > 0)) //小车运动方向判断 前进时（PWM即电机电压为正） 脉冲数为负数
  {
    rpluse = rpluse;
    lpluse = -lpluse;
  }
  else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 < 0)) //小车运动方向判断 左旋转 右脉冲数为负数 左脉冲数为正数
  {
    rpluse = -rpluse;
    lpluse = lpluse;
  }
  balancecar.stopr += rpluse;
  balancecar.stopl += lpluse;
  balancecar.pulseright += rpluse;
  balancecar.pulseleft += lpluse;
  sumam = balancecar.pulseright + balancecar.pulseleft;
}

/////////////////////角度PD////////////////////////
void angleout()
{
  balancecar.angleoutput = kp * (kalmanfilter.angle + angle0) + kd * kalmanfilter.Gyro_x; // PD 角度环控制
}

///////////////////角度环 PD控制////////////////////
void inter()
{
  sei();
  countpluse();                                                                          //脉冲叠加子函数
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);                                          // IIC获取MPU6050六轴数据 ax ay az gx gy gz
  kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1); //获取angle 角度和卡曼滤波
  angleout();                                                                            //角度环 PD控制

  speedcc++;
  if (speedcc >= 8) // 40ms进入速度环控制
  {
    Outputs = balancecar.speedpiout(kp_speed, ki_speed, kd_speed, front, back, setp0);
    speedcc = 0;
  }
  turncount++;
  if (turncount > 4) // 40ms进入旋转控制
  {
    turnoutput = balancecar.turnspin(turnl, turnr, spinl, spinr, kp_turn, kd_turn, kalmanfilter.Gyro_z); //旋转子函数
    turncount = 0;
  }
  balancecar.posture++;
  //小车总PWM输出
  balancecar.pwma(Outputs, turnoutput, kalmanfilter.angle, kalmanfilter.angle6, turnl, turnr, spinl, spinr, front, back, kalmanfilter.accelz, IN1M, IN2M, IN3M, IN4M, PWMA, PWMB);
}

/////////////////////自动上报////////////////////////
void SendAutoUp()
{
  g_uptimes--;
  if ((g_autoup == true) && (g_uptimes == 0))
  {
    String CSB, VT;
    char temp[10] = {0};
    float fgx;
    float fay;
    float leftspeed;
    float rightspeed;
    fgx = gx; //传给局部变量
    fay = ay; //传给局部变量
    leftspeed = balancecar.pwm1;
    rightspeed = balancecar.pwm2;
    double Gx = (double)((fgx - 128.1f) / 131.0f); //角度转换
    double Ay = ((double)fay / 16384.0f) * 9.8f;
    if (leftspeed > 255 || leftspeed < -255)
      return;
    if (rightspeed > 255 || rightspeed < -255)
      return;
    if ((Ay < -20) || (Ay > 20))
      return;
    if ((Gx < -3000) || (Gx > 3000))
      return;
    returntemp = "";
    memset(temp, 0x00, sizeof(temp));
    dtostrf(leftspeed, 3, 1, temp);
    String LV = temp;
    memset(temp, 0x00, sizeof(temp));
    dtostrf(rightspeed, 3, 1, temp);
    String RV = temp;
    memset(temp, 0x00, sizeof(temp));
    dtostrf(Ay, 2, 2, temp);
    String AC = temp;
    memset(temp, 0x00, sizeof(temp));
    dtostrf(Gx, 4, 2, temp);
    String GY = temp;
    CSB = "0.00";
    VT = "0.00";
    returntemp = "$LV" + LV + ",RV" + RV + ",AC" + AC + ",GY" + GY + ",CSB" + CSB + ",VT" + VT + "#";
  }

  if (g_uptimes == 0)
    g_uptimes = 5000;
}

void ResetPID()
{
  kp = PID_Original[0];
  ki = PID_Original[1];
  kd = PID_Original[2]; //需要修改的参数
  kp_speed = PID_Original[3];
  ki_speed = PID_Original[4];
  kd_speed = PID_Original[5]; // 需要修改的参数
}

void ResetCarState()
{
  turnl = 0;
  turnr = 0;
  front = 0;
  back = 0;
  spinl = 0;
  spinr = 0;
  turnoutput = 0;
}

/****************************脉冲中断计算*********************************/
///////////////////////左测速码盘计数/////////////////////////////////////
void Code_left()
{
  count_left++;
}

/////////////////////////////右测速码盘计数//////////////////////////////
void Code_right()
{
  count_right++;
}

////////////////////////蓝牙接收////////////////////////////
int num1 = 0;
void serialEvent()
{
  while (Serial.available()) //当无法接收到信号时，一直在这循环
  {
  }
  while (Serial.available())
  {
    incomingByte = Serial.read();
    if (incomingByte == '$') //获得命令 以'$'开头
    {
      num1 = 0;
      startBit = true;
    }
    if (startBit == true)
    {
      num1++;
      inputString += (char)incomingByte;
    }
    if (startBit == true && incomingByte == '#') //结束命令 以'#'结尾
    {
      newLineReceived = true;
      startBit = false;
    }
    if (num1 >= 80)
    {
      num1 = 0;
      startBit = false;
      newLineReceived = false;
      inputString = "";
    }
  }
}

void setup()
{
  pinMode(speak, OUTPUT); //控制语音模块
  pinMode(IN1M, OUTPUT);  //控制电机1的方向，01为正转，10为反转
  pinMode(IN2M, OUTPUT);
  pinMode(IN3M, OUTPUT); //控制电机2的方向，01为正转，10为反转
  pinMode(IN4M, OUTPUT);
  pinMode(PWMA, OUTPUT); //左电机PWM
  pinMode(PWMB, OUTPUT); //右电机PWM
  pinMode(STBY, OUTPUT); // TB6612FNG使能
  digitalWrite(IN1M, LOW);
  digitalWrite(IN2M, HIGH);
  digitalWrite(IN3M, HIGH);
  digitalWrite(IN4M, LOW);
  digitalWrite(STBY, HIGH);
  analogWrite(PWMA, LOW);
  analogWrite(PWMB, LOW);
  pinMode(PinA_left, INPUT);
  pinMode(PinA_right, INPUT);
  //pinMode(PinB_left, INPUT);
  //pinMode(PinB_right, INPUT);

  // 加入I2C总线
  Wire.begin();
  Serial.begin(9600);
  delay(1500);
  mpu.initialize();
  delay(3);
  balancecar.pwm1 = 0;
  balancecar.pwm2 = 0;
}

void loop()
{
  String returnstr = "$0,0,0,0,0,0,0,0,0,0,0,0cm,8.2V#";
  //主函数中循环检测及叠加脉冲 测定小车车速  使用电平改变既进入脉冲叠加 增加电机的脉冲数，保证小车的精确度。
  attachInterrupt(2, Code_left, CHANGE);
  attachPinChangeInterrupt(4, Code_right, CHANGE);
  serialEvent();
  if (true)   //  inputString[1] == '6'
  {
    MsTimer2::set(5, inter);
    MsTimer2::start();
    flag = true;
  }
  if (newLineReceived && flag)
  {
    switch (inputString[1])
    {
    case speak:
      g_carstate = enspeak;
      break;
    case run_car:
      g_carstate = enRUN;
      break;
    case back_car:
      g_carstate = enBACK;
      break;
    case left_car:
      g_carstate = enLEFT;
      break;
    case right_car:
      g_carstate = enRIGHT;
      break;
    case stop_car:
      g_carstate = enSTOP;
      break;
    default:
      g_carstate = enSTOP;
      break;
    }
    if (inputString.length() < 21)
    {
      //恢复默认
      inputString = "";
      newLineReceived = false;
      goto a;
    }
    if (inputString[3] == '1' && inputString.length() == 21) //左摇
    {
      g_carstate = enTLEFT;
    }
    else if (inputString[3] == '2' && inputString.length() == 21) //右摇
    {
      g_carstate = enTRIGHT;
    }
    if (inputString[5] == '1')
    {
      char charkp[7], charkd[7], charkpspeed[7], charkispeed[7];
      dtostrf(kp, 3, 2, charkp);            // 相当于 %3.2f
      dtostrf(kd, 3, 2, charkd);            // 相当于 %3.2f
      dtostrf(kp_speed, 3, 2, charkpspeed); // 相当于 %3.2f
      dtostrf(ki_speed, 3, 2, charkispeed); // 相当于 %3.2f
      String strkp = charkp;
      String strkd = charkd;
      String strkpspeed = charkpspeed;
      String strkispeed = charkispeed;
      returntemp = "$0,0,0,0,0,0,AP" + strkp + ",AD" + strkd + ",VP" + strkpspeed + ",VI" + strkispeed + "#";
    }
    else if (inputString[5] == '2') //恢复PID
    {
      ResetPID();
    }
    if (inputString[7] == '1') //自动上报
    {
      g_autoup = true;
    }
    else if (inputString[7] == '2') //停止自动上报
    {
      g_autoup = false;
    }
    if (inputString[9] == '1') //角度环更新
    {
      int i = inputString.indexOf("AP");
      int ii = inputString.indexOf(",", i);
      if (ii > i)
      {
        String m_skp = inputString.substring(i + 2, ii);
        m_skp.replace(".", "");
        int m_kp = m_skp.toInt();
        kp = (double)((double)m_kp / 100.0f);
      }
      i = inputString.indexOf("AD");
      ii = inputString.indexOf(",", i);
      if (ii > i)
      {
        String m_skd = inputString.substring(i + 2, ii);
        m_skd.replace(".", "");
        int m_kd = m_skd.toInt();
        kd = (double)((double)m_kd / 100.0f);
      }
    }
    if (inputString[11] == '1') //速度环更新
    {
      int i = inputString.indexOf("VP");
      int ii = inputString.indexOf(",", i);
      if (ii > i)
      {
        String m_svp = inputString.substring(i + 2, ii);
        m_svp.replace(".", "");
        int m_vp = m_svp.toInt();
        kp_speed = (double)((double)m_vp / 100.0f);
      }

      i = inputString.indexOf("VI");
      ii = inputString.indexOf("#", i);
      if (ii > i)
      {
        String m_svi = inputString.substring(i + 2, ii);
        m_svi.replace(".", "");
        int m_vi = m_svi.toInt();
        ki_speed = (double)((double)m_vi / 100.0f);
      }
    }
    //恢复默认
    inputString = "";
    newLineReceived = false;
  }
a:
  switch (g_carstate)
  {
  case enRUN:
    ResetCarState();
    front = 250;
    break;
  case enLEFT:
    turnl = 1;
    break;
  case enRIGHT:
    turnr = 1;
    break;
  case enBACK:
    back = -250;
    break;
  case enTLEFT:
    spinl = 1;
    break;
  case enTRIGHT:
    spinr = 1;
    break;
  case enspeak:
    digitalWrite(speak, 0);
    digitalWrite(speak, 1);
    delay(10);
    digitalWrite(speak, 0);
    ResetCarState();
    break;
  default:
    front = 0;
    back = 0;
    turnl = 0;
    turnr = 0;
    spinl = 0;
    spinr = 0;
    turnoutput = 0;
    break;
  }

  //增加自动上报
  SendAutoUp();
}

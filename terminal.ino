#include <LoRa.h>
#include <SPI.h>
#include <arduino.h>

#define ADDRESS 0x40     //主节点地址
#define ADDRESS_BC 0x41  //广播地址

#define CHAR_ACK '@'    //ACK
#define CHAR_ACS '^'    //请求接入网络信号
#define NULL_DATA " "   //空数据

//终端
#define MAX_WAIT_TIME_ACS 1500
#define MAX_WAIT_TIME_TER 1000
#define MAX_WAIT_TIME_CONTACT 10000
#define DELAY_SEND 50

String s_message = NULL_DATA;
String r_message = NULL_DATA;
byte D_Addr;

typedef struct MESSAGE
{
  byte DAddr;       //目的地址
  byte SAddr;       //源地址 
  byte Function;    //功能位
  String MsgData;   //数据
} Message;

//定时=============================================================================================================
bool flag;                //是否开启过计时器
unsigned long t_start;
unsigned long t_current;
int t_ineterval;

void Timer_start(int t_ineterval_in)//重置计时
{
  flag = true;
  t_start = millis();
  t_ineterval = t_ineterval_in;
}

bool Timer_result()//判断定时是否到
{
  if(flag)
  {
    t_current = millis();
    if((t_current - t_start) >= t_ineterval)
    {
      flag = false; //关闭计时器
      return true;  //说明计时到了
    }
    else
      return false;
  }
}

void Timer_close() //关闭计时器
{
  flag = false;
}

//通信=============================================================================================================

byte Addr_Self = 0x50;  //地址
String Name_Self = "Huang's phone";
bool Acs = false;       //是否接入了网络
bool Link = false;      //是否与主机建立了连接

void SendToCN(Message* msg)             //参数为整个报文
{
  delay(DELAY_SEND);
  LoRa.beginPacket();
  LoRa.write(msg->DAddr);               //发送目的地址
  LoRa.write(msg->SAddr);               //发送源地址
  LoRa.write(msg->Function);            //发送功能位
  LoRa.print((msg->MsgData));           //发送数据（字符串变量用print）
  LoRa.endPacket();
}

bool RecvFrCN(Message* pmsg)            //返回整个报文
{
  if(LoRa.parsePacket() == 0) return false;
  String str;
  while(LoRa.available())                                   //开始读数据（根据协议可知，不会收到连续的两帧）
  {
    str += (char)LoRa.read();                               //读取收到的所有数据
  }
  
  if((str.charAt(0) == Addr_Self || str.charAt(0) == ADDRESS_BC)&& str.charAt(1) == ADDRESS)  //确认是主节点发送给本机，则开始对报文结构体属性赋值
  {
    Serial.println("data from CN: "+str);
    pmsg->DAddr = str.charAt(0);                                     
    pmsg->SAddr = (byte)str.charAt(1);                      //源地址为第二位（索引为1）
    pmsg->Function = (byte)str.charAt(2);                   //功能位为第三位（索引为2）
    str.remove(0,3);
    pmsg->MsgData = str;                                    //删除前3字节后剩下的字节全为数据
    return true;
  }
  return false;
}
//========================================================================================================================================
//等待广播帧闭并请求接入 
//返回值false：收到了广播帧但请求超时 可能是数据丢失或被加入黑名单
//返回值true：收到了广播帧且请求成功
bool RequestAcs()                                                    
{
  Message msg_s;
  Message msg_r;
  msg_s.DAddr = ADDRESS;
  msg_s.SAddr = Addr_Self;
  msg_s.Function = '7';
  msg_s.MsgData = Name_Self;

  while(1)
  { 
    if(RecvFrCN(&msg_r))
    {
      if(msg_r.Function == '6' && Acs == false )   //广播通知接入
      {
        Serial.println("收到广播通知接入并申请接入");
        break;
      }
    }
  }
  delay(random(50)*10);               //随机延时一段时间
  SendToCN(&msg_s);
  Timer_start(MAX_WAIT_TIME_ACS);
  while(1)
  {
    if(Timer_result())
    {
      Serial.println("请求接入超时失败");
      Acs = false;
      return false;
    }
    if(RecvFrCN(&msg_r))
    {
      if(msg_r.Function == '8' && (msg_r.MsgData).charAt(0) == CHAR_ACK)
      {
        Serial.println("请求接入成功");
        Timer_start(MAX_WAIT_TIME_CONTACT);
        Acs = true;
        return true;
      }
    }
  }
}

void Response()                      //根据从主节点接收到的帧执行相应命令
{
  Message msg_r;
  Message msg_s;
  msg_s.DAddr = ADDRESS;
  msg_s.SAddr = Addr_Self;
  
  if(RecvFrCN(&msg_r))
  {
    if(msg_r.Function == '0')                //接到要求退网命令
    {
      Serial.println("接到要求退网命令");
      Acs = false;                            
      Link = false;
      msg_s.Function = (byte)'0';
      (msg_s.MsgData) = CHAR_ACK;
      SendToCN(&msg_s);
    }
    else if(msg_r.Function == '1')           //接收到更改频率命令
    {
      Timer_start(MAX_WAIT_TIME_CONTACT); 
      Serial.println("接收到更改频率命令");
      msg_s.Function = '1';
      msg_s.MsgData = CHAR_ACK;
      SendToCN(&msg_s);
      LoRa.setFrequency(msg_r.MsgData.toInt() * 1E6);
    }
    else if(msg_r.Function == '2')          //收到建立连接命令
    {
      Timer_start(MAX_WAIT_TIME_CONTACT);     
      Serial.println("收到建立连接命令");
      Link = true;
      msg_s.Function = '2';
      (msg_s.MsgData) = CHAR_ACK;
      SendToCN(&msg_s);
    }
    else if(msg_r.Function == '3' && Link)         //收到断开连接命令
    {
      Timer_start(MAX_WAIT_TIME_CONTACT);     
      Serial.println("收到断开连接命令");
      Link = false;
      msg_s.Function = '3';
      msg_s.MsgData = CHAR_ACK;
      SendToCN(&msg_s);
    }
    else if(msg_r.Function == '4' && Link)         //收到发送数据命令
    {
      Timer_start(MAX_WAIT_TIME_CONTACT);     
      Serial.println("收到发送数据命令");
      msg_s.Function = '4';
      msg_s.MsgData = String(GetData());
      SendToCN(&msg_s);
    }

    else if(msg_r.Function == '5')         //保持联系
    {
      if (msg_r.MsgData != NULL_DATA)
      {
        r_message = msg_r.MsgData;
        byte r_Addr = r_message.charAt(0); 
        r_message.remove(0,1);
        Serial.print("message received from ");Serial.print(r_Addr);Serial.print(':');Serial.println(r_message);
      }
      Timer_start(MAX_WAIT_TIME_CONTACT);
      Serial.println("收到保持联系命令");
      msg_s.Function = '5';
      if(s_message != NULL_DATA)
      {
        msg_s.MsgData = s_message;
      }
      else
      {
        msg_s.MsgData = CHAR_ACK;
      }
      SendToCN(&msg_s);
      s_message = NULL_DATA;  
    }
  }
  if(Timer_result())
  {
    Acs = false;
    Link = false;
    Timer_close();
    Serial.println("连接超时，断开连接");
  }
} 

byte GetData()                                                      //获取数据
{
  return (random(0,255));
}

//===================================================================================================

void setup() 
{
  Serial.begin(9600);//定义串口波特率为9600
  while (!Serial);
  Serial.println("LoRa Sender");//每次重新打开串口监视器时打印此语句
  if (!LoRa.begin(510E6))//ISM频段 包括433、868、915MHZ等，可在LoRa芯片上查看 
  {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() 
{
  String str;
  if(Serial.available())                                    //串口缓冲区有数据
  {
    delay(10);                                              //延迟10ms等待数据报发完
    str = Serial.readStringUntil('\n');
    Serial.println(str);
    if(str == "connect")
    {
      if(Acs == false) {RequestAcs();}
      else {Serial.println("connected already");}
    }
    else if (str=="disconnect") 
    {
      if(Acs == false) {Serial.println("disconnected already");}
      else {Acs=false;}
    }
    else if (str.charAt(0)==':')     //':'+三字节地址+数据
    { 
      str.remove(0,1);               //去掉冒号
      D_Addr = (str.substring(0, 3)).toInt();   //解析出目的地址
      str.remove(0,3);                          //解析出数据
      Serial.print("message send to ");Serial.print(D_Addr);Serial.print(':');Serial.println(str);
      s_message = (char)D_Addr;
      s_message += str;         //传给中心节点：目的地址（一字节）+数据            
    }
    else 
    {
      Serial.println("input 'connect' or 'disconnect' or ':'");
    }
    str = "";
  }
  if(Acs == true)
  {
    Response();
  }
}

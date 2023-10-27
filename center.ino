#include <LoRa.h>
#include <SPI.h>
#include <arduino.h>

#define ADDRESS 0x40
#define ADDRESS_BC 0x41
#define CHAR_SYNC '#'   //同步位
#define CHAR_END '*'    //结束位
#define CHAR_ACK '@'    //ACK
#define CHAR_ACS '^'    //请求接入网络信号
#define NULL_DATA " "   //空数据

#define MAX_WAIT_TIME 1000
#define WAIT_DELAY 50
#define MAX_WAIT_TIME_BROADCAST 1000

#define Time_Timeout 0x03  //超时重传次数
//=====================================================================================================================
//主节点与终端节点通信帧结构
typedef struct MESSAGE
{
  byte DAddr;       //目的地址
  byte SAddr;       //源地址
  byte Function;    //功能位
  String MsgData;   //数据
} Message;

//网络状态链表 ==========================================================================================================
typedef struct node NODE;
typedef struct node{
  NODE* Next;       //下一节点
  byte Addr;        //地址
  char* Name;       //名字
  byte State;       //退网时掉线 切换频率时掉线 建立连接时掉线 断开连接时掉线 传输数据时掉线 保持联系时掉线
  byte Time;        //超时重传次数
}NODE;

NODE* NodeList = NULL;     //链表表头
byte StateMask[6] = {0x01,0x02,0x04,0x08,0x10,0x20};

NODE* Node_Link = NULL;
//黑名单链表============================================================================================================
typedef struct BlackNode BNODE;
typedef struct BlackNode{
  BNODE* Next;       //下一节点
  byte Addr;        //地址
  char* Name;      //名字
}BNODE;

BNODE* BlackList = NULL;               //链表表头
//链表函数=============================================================================================================
void Add_Node(byte addr,char* str)     //节点加入链表
{ 
  NODE* pnode;
  pnode = (NODE*)malloc(sizeof(NODE));
  pnode->Next = NodeList;
  pnode->Addr = addr;
  char *strtmp;
  strtmp = (char*)malloc(sizeof(char)*20);
  strcpy(strtmp,str);
  pnode->Name = strtmp;
  pnode->State = 0x00;
  pnode->Time = Time_Timeout;          //初始化重传3次
  NodeList = pnode;
}

NODE* Find_Node(byte addr)
{
  NODE* pnode = NodeList;
  if(pnode == NULL) return NULL;
  while(pnode != NULL)
  {
    if(pnode->Addr == addr)
      return pnode;
    else
      pnode = pnode->Next;
  }
  return NULL;
}

bool Del_Node(byte addr)     //节点退出链表
{
  NODE* nodefront = NodeList;
  NODE* nodeback = NodeList;
  if(NodeList->Addr == addr){        //删除链表中的第一个节点
    NodeList = NodeList->Next;
    free(nodeback->Name);
    free(nodeback);
    return true;
  }
  else                        
  {
    do{
      nodeback = nodefront->Next;
      if(nodeback->Addr == addr){
        nodefront->Next = nodeback->Next;
        free(nodeback->Name);
        free(nodeback);
        return true;
      }
      else{
        nodefront = nodefront->Next;
      }
    }while(nodeback != NULL);
    return false;           //该节点不在
  }
}

void Add_BNode(byte addr,char* str)     //节点加入链表
{ 
  BNODE* pnode;
  pnode = (BNODE*)malloc(sizeof(BNODE));
  pnode->Next = BlackList;
  pnode->Addr = addr;
  char *strtmp;
  strtmp = (char*)malloc(sizeof(char)*20);
  strcpy(strtmp,str);
  pnode->Name = strtmp;
  BlackList = pnode;
}

BNODE* Find_BNode(byte addr)
{
  BNODE* pnode = BlackList;
  if(pnode == NULL) return NULL;
  while(pnode != NULL)
  {
    if(pnode->Addr == addr)
      return pnode;
    else
      pnode = pnode->Next;
  }
  return NULL;
}

bool Del_BNode(byte addr)     //节点退出链表
{
  BNODE* nodefront = BlackList;
  BNODE* nodeback = BlackList;
  if(BlackList->Addr == addr){        //删除链表中的第一个节点
    BlackList = BlackList->Next;
    free(nodeback->Name);
    free(nodeback);
    return true;
  }
  else                        
  {
    do{
      nodeback = nodefront->Next;
      if(nodeback->Addr == addr){
        nodefront->Next = nodeback->Next;
        free(nodeback->Name);
        free(nodeback);
        return true;
      }
      else{
        nodefront = nodefront->Next;
      }
    }while(nodeback != NULL);
    return false;           //该节点不在
  }
}

//定时=============================================================================================================
unsigned long t_start;
unsigned long t_current;
int t_ineterval;

void Timer_start(int t_ineterval_in)//重置计时
{
  t_start = millis();
  t_ineterval = t_ineterval_in;
}

bool Timer_result()//判断定时是否到
{
    t_current = millis();
    if((t_current - t_start) >= t_ineterval)  {return true;}
    else {return false;}
  }
}

//校验===================================================================================================
//生成奇偶检验序列(输入不包含FCS)
String generateFCS(String cmd)
{
  int len = cmd.length();
  long int sum = 0;
  String FCS;
  while(len)
  {
    sum += cmd.charAt(0);
    cmd.remove(0,1);
    len--;                          
  }
  FCS = (char)(sum / 128);    //需保证sum>128该字节才存在 
  FCS += (char)(sum % 128);
  return FCS;
}

//校验FCS（输入包含FCS）
bool checkFCS(String cmd)
{
  String FCS = generateFCS(cmd);
  if(FCS.toInt() == 0) return true;
  return false;
}

//通信=============================================================================================================
void SendToTer(Message* msg)           //参数为整个报文
{
  delay(WAIT_DELAY);
  LoRa.beginPacket();
  LoRa.write(msg->DAddr);               //发送目的地址
  LoRa.write(msg->SAddr);               //发送源地址
  LoRa.write(msg->Function);            //发送功能位
  LoRa.print((msg->MsgData));           //发送数据（字符串变量用print）
  LoRa.endPacket();
  /*
  Serial.print("send to Tr:");
  Serial.write(msg->DAddr);               //发送目的地址
  Serial.write(msg->SAddr);               //发送源地址
  Serial.write(msg->Function);            //发送功能位
  Serial.println((msg->MsgData));           //发送数据（字符串变量用print）
  */
}

bool RecvFrTer(Message* pmsg)           //返回整个报文
{
  if(LoRa.parsePacket() == 0) return false;
  String str;
  while(LoRa.available())                                   //开始读数据（根据协议可知，不会收到连续的两帧）
  {
    str += (char)LoRa.read();                               //读取收到的所有数据
  }
  if(str.charAt(0) == (char)ADDRESS)                        //确认是发送给本机，则开始对报文结构体属性赋值
  {
    //Serial.print("data from Tr:");
    //Serial.println(str);
    pmsg->DAddr = ADDRESS;                                     
    pmsg->SAddr = (byte)str.charAt(1);                      //源地址为第二位（索引为1）
    pmsg->Function = (byte)str.charAt(2);                   //功能位为第三位（索引为2）
    str.remove(0,3);
    pmsg->MsgData = str;                                    //删除前3字节后剩下的字节全为数据
    return true;
  }
  return false;
}

bool RecvFrPC(String* pCommand)          //返回功能+数据 部分
{
  int length_a;
  int length_r;
  char buffer_r[100];
  String buffer_rc;
  if(length_a=Serial.available())                                    //串口缓冲区有数据
  {
    delay(WAIT_DELAY);                                             //延迟等待数据报发完
    length_r = Serial.readBytesUntil(CHAR_SYNC, buffer_r, 100);//找同步位
    if(length_r==length_a) return false;          //未找到同步位

    length_a=Serial.available();
    length_r = Serial.readBytesUntil(CHAR_END, buffer_r, 100);//找结束位
    if(length_r==Serial.available()) return false;          //未找到结束位
    
    buffer_r[length_r] = '\0';
    buffer_rc = buffer_r;
  
    *pCommand = buffer_rc;        //返回功能+数据+FCS
    //Serial.println("Newdata from pc:"+*pCommand);
    return true;
  }
  return false;
}

void SendToPC(String MsgData,byte Function)    //参数为功能+数据+FCS
{
  delay(WAIT_DELAY); //两帧之间保持距离
  Serial.write(CHAR_SYNC);
  Serial.write(Function);
  Serial.print(MsgData);
  Serial.write(CHAR_END);
}

//根据主机命令（输入参数）执行任务==================================================================================================
//PC允许入网
void PC_Permit(String pCommand)          //参数为函数RecvFrPC的返回字符串，即功能+数据
{
  String cmd = pCommand;
  if(cmd.charAt(0) != '/') return; 
  cmd.remove(0, 1);
  byte lenFrPC = cmd.length();
  
  byte Addr_Permit; 
  String Data_toPC;
  Message msg_s;
  
  for(int i = 0;i < lenFrPC;i++)
  {
    Addr_Permit = (byte)cmd.charAt(i); 
    if(Find_BNode(Addr_Permit) == NULL)       //节点不在黑名单
    {
      Data_toPC="addr "+String(Addr_Permit)+" :permited already(erro)";
      SendToPC(Data_toPC,'/');
    }
    else
    {
      Del_BNode(Addr_Permit);                //从黑名单中删除
      Data_toPC="addr "+String(Addr_Permit)+" :permited successfully";
      SendToPC(Data_toPC,'/');
    }
  }
}

//执行退网命令
void PC_Logout(String pCommand)   //参数为函数RecvFrPC的返回字符串，即功能+数据
{
  String cmd = pCommand;
  if(cmd.charAt(0) != '0') return;                                
  cmd.remove(0, 1);                                                       //删掉功能
  byte lenFrPC = cmd.length();
  
  String Data_toPC;
  Message msg_s;
  Message msg_r;
  
  msg_s.SAddr = ADDRESS;                                                  //源地址为中心节点地址
  msg_s.Function = '0';                                                   //功能为要求退网
  msg_s.MsgData = NULL_DATA;
  
  for(int i = 0;i < lenFrPC;i++)
  {
    msg_s.DAddr = (byte)cmd.charAt(i);     //解析出目的地址
    NODE* Dnode = Find_Node(msg_s.DAddr);
    if(Dnode == NULL)                      //节点不在网络
    {
      Data_toPC="addr "+String(msg_s.DAddr)+" :off net(erro)";
      SendToPC(Data_toPC,'0');
    }
    else
    {
      SendToTer(&msg_s);
      Timer_start(MAX_WAIT_TIME);
      while(1)
      {
        if(Timer_result())                                        //定时到
        {
          Dnode->State |= StateMask[0];
          Data_toPC="addr "+String(msg_s.DAddr)+": logout-response timed out(erro)";
          SendToPC(Data_toPC,'0');
          break;
        }
        if(RecvFrTer(&msg_r))                                    //接收到数据   
        {
          //确定为来自目标节点的ACK
          if(msg_r.SAddr == msg_s.DAddr && msg_r.Function == (byte)'0' && (msg_r.MsgData).charAt(0) == CHAR_ACK)
          { 

            Add_BNode(msg_r.SAddr,(Find_Node(msg_r.SAddr))->Name);
            Del_Node(msg_r.SAddr);
            Data_toPC="addr "+String(msg_s.DAddr)+": successfully logout";
            SendToPC(Data_toPC,'0');
            break;
          }
        }
      }
    }
  }
}

void PC_ChangeFre(String pCommand)   //参数为函数RecvFrPC的返回字符串，即功能+数据
{
  String cmd = pCommand;
  if(cmd.charAt(0) != '1') return;                               
  cmd.remove(0, 1);               //删掉功能
  int Fre = cmd.toInt();          
  
  NODE* pnode = NodeList;
  Message msg_s;
  Message msg_r;
  
  msg_s.SAddr = ADDRESS;
  msg_s.Function = '1';                       
  msg_s.MsgData = cmd;    
   
  while(pnode != NULL)                 
  {
    msg_s.DAddr = pnode->Addr;
    SendToTer(&msg_s); 
    Timer_start(MAX_WAIT_TIME);           //开始计时
    while(1)
    { 
      if(Timer_result())
      {
        pnode->State |= StateMask[1];
        break;
      }
      if(RecvFrTer(&msg_r))
      {
        if(msg_r.SAddr == pnode->Addr && msg_r.Function == (byte)'1' && (msg_r.MsgData).charAt(0) == CHAR_ACK)
        { 
          Serial.print(pnode->Addr);Serial.print("OK");
          delay(1000);
          break;
        }
      }
    }
    pnode = pnode->Next;
  }
  LoRa.setFrequency(Fre * 1E6);Serial.print("OKK");
}

void PC_Link(String pCommand)          //参数为函数RecvFrPC的返回字符串，即功能+数据
{
  String cmd = pCommand;
  if(cmd.charAt(0) != '2') return;                                
  cmd.remove(0, 1);                                                       //删掉功能
  byte Addr_toLink = cmd.charAt(0);                                       //取出数据

  NODE* pnode_Linked = Node_Link;
  NODE* pnode_toLink = Find_Node(Addr_toLink);
  String Data_toPC;  
  Message msg_s;
  Message msg_r;
  
  if(pnode_toLink == NULL)
  {
     Data_toPC="addr "+String(Addr_toLink)+" :off net (erro)";
     SendToPC(Data_toPC,'2');
     return;
  }
  msg_s.SAddr = ADDRESS;                                                  //源地址为中心节点地址
  msg_s.DAddr = Addr_toLink; 
  msg_s.Function = '2';                                                   //功能为建立连接
  msg_s.MsgData = NULL_DATA;
  SendToTer(&msg_s); 
  
  Timer_start(MAX_WAIT_TIME);          //开始计时
  while(1)
  { 
    if(Timer_result())
    { 
      pnode_toLink->State |= StateMask[2];
      Data_toPC="addr "+String(msg_s.DAddr)+": Link-response timed out(erro)";
      SendToPC(Data_toPC,'2');
      return;   //不执行断开连接动作
    }
    if(RecvFrTer(&msg_r))
    {
      if(msg_r.SAddr == Addr_toLink && msg_r.Function == (byte)'2' && (msg_r.MsgData).charAt(0) == CHAR_ACK)  
      { 
        Node_Link = pnode_toLink;
        Data_toPC="addr "+String(msg_s.DAddr)+" :successfully linked";
        SendToPC(Data_toPC,'2');
        break;   //执行断开连接动作
      }
    }
  }
  //断开连接（不管是否断开连接成功，优先建立连接）
  msg_s.DAddr = pnode_Linked->Addr; 
  msg_s.Function = '3'; 
  SendToTer(&msg_s); 
  Timer_start(MAX_WAIT_TIME);          //开始计时
  if(pnode_Linked != NULL && pnode_Linked != pnode_toLink)
  {
    while(1)
    { 
      if(Timer_result())
      {
        pnode_Linked->State |= StateMask[3];
        Data_toPC="addr"+String(msg_s.DAddr)+": Link-broken-response timed out(erro)";
        SendToPC(Data_toPC,'3');
        break;
      }
      if(RecvFrTer(&msg_r))
      {
        if(msg_r.SAddr == pnode_Linked->Addr && msg_r.Function == (byte)'3' && (msg_r.MsgData).charAt(0) == CHAR_ACK)  
        { 
          Data_toPC="addr"+String(msg_s.DAddr)+": successfully broke the old link";
          SendToPC(Data_toPC,'3');
          break;
        }
      }
    }
  }
}

//执行更新入网状态命令
void PC_Update(String pCommand)          //参数为函数RecvFrPC的返回字符串，即功能+数据
{
  String cmd = pCommand;
  if(cmd.charAt(0) != '5') return;                                
  
  String Data_toPC;
  NODE* pnode = NodeList;
  while(pnode != NULL)
  {
    Data_toPC += (char)pnode->Addr;
    Data_toPC += pnode->Name;
    Data_toPC += (char)'?';
    pnode = pnode->Next;
  }
  SendToPC(Data_toPC,'5');
}

//更新黑名单
void PC_BlackList(String pCommand)          //参数为函数RecvFrPC的返回字符串，即功能+数据
{
  String cmd = pCommand;
  if(cmd.charAt(0) != '6') return;                                
  
  String Data_toPC;
  char buffer_Data[255];
  BNODE* pnode = BlackList;
  while(pnode != NULL)
  {
    Data_toPC += (char)pnode->Addr;
    Data_toPC += pnode->Name;
    Data_toPC += (char)'?';
    pnode = pnode->Next;
  }
  SendToPC(Data_toPC,'6');
}
//与终端之间通信（无主机参与部分）====================================================================================================================
//广播通知接入
void Broadcast()
{
  Message msg_s;
  Message msg_r;

  msg_s.DAddr = ADDRESS_BC;                     //目的地址设置为广播地址
  msg_s.SAddr = ADDRESS;
  msg_s.Function = '6';                         
  msg_s.MsgData = CHAR_ACS;    
  SendToTer(&msg_s);  

  Timer_start(MAX_WAIT_TIME_BROADCAST);           //开始计时
  while(1)
  { 
    if(Timer_result())
    {
      return;
    }
    if(RecvFrTer(&msg_r))
    {
      if(Find_Node(msg_r.SAddr) == NULL && Find_BNode(msg_r.SAddr) == NULL && msg_r.DAddr == ADDRESS && msg_r.Function == (byte)'7')
      { 
        const char* str0 = (msg_r.MsgData).c_str();
        char* str1 =const_cast<char*>(str0);
        Add_Node(msg_r.SAddr,str1);          //加入接入链表
        msg_s.DAddr = msg_r.SAddr;                
        msg_s.SAddr = ADDRESS;
        msg_s.Function = '8';       
        msg_s.MsgData = CHAR_ACK;    
        SendToTer(&msg_s);  
      }
    }
  }
}

//保持联系
void KeepContact()   
{
  NODE* pnode = NodeList;
  Message msg_s;
  Message msg_r;
  
  msg_s.SAddr = ADDRESS;
  msg_s.Function = '5';                        
  msg_s.MsgData = NULL_DATA;    
 
  while(pnode != NULL)
  { 
      msg_s.DAddr = pnode->Addr;
      msg_s.Function = '5';
      SendToTer(&msg_s); 
      Timer_start(MAX_WAIT_TIME);           //开始计时
      while(1)
      {
        if(Timer_result())
        {
          pnode->State |= StateMask[5];
          Serial.print(pnode->Addr);Serial.print("State:");Serial.println(pnode->State);
          break;
        }
        if(RecvFrTer(&msg_r))
        {
          if(msg_r.SAddr == pnode->Addr && msg_r.Function == '5')  
          { 
            if(!(msg_r.MsgData.length() == 1 && msg_r.MsgData.charAt(0)== CHAR_ACK))                     //表示有数据要发送给其他终端
            {
              //Serial.print("somebody have messege to send:"); Serial.println(msg_r.MsgData);
              msg_s.DAddr = (char)((msg_r.MsgData).charAt(0));     //解析出目的地址写入发送帧
              (msg_r.MsgData).remove(0,1);                         //去掉目的地址并换成源地址                        
              msg_s.MsgData = (char)msg_r.SAddr; //Serial.println(msg_r.SAddr);Serial.println(msg_s.MsgData);
              msg_s.MsgData += msg_r.MsgData;
              //Serial.print("已转发："); Serial.println(msg_s.MsgData);
              SendToTer(&msg_s);
            }
            break;
          }
        }
      }
    pnode = pnode->Next;
  }
}

void AskExist()
{ 
  NODE* pnode = NodeList;
  Message msg_s;
  Message msg_r;
  String Data_toPC;
  byte i;
  
  msg_s.SAddr = ADDRESS;                                              //源地址为中心节点地址                                              
  msg_s.MsgData = NULL_DATA;

  while(pnode != NULL)
  { 
    //Serial.print("0AskExisting ");Serial.print(pnode->Addr);Serial.print(":");Serial.println(pnode->State);
    if((pnode->State & 0x3f) != 0x00)
    {
      //Serial.print("1AskExisting ");
      for (i = 0; i <= 5; i++)
      {
        if((pnode->State&StateMask[i])!=0)
        {
          //Serial.print("2AskExisting ");Serial.println(i);
          break;
        }
      }
      msg_s.DAddr = pnode->Addr;
      msg_s.Function = i+'0';
      SendToTer(&msg_s); 
      Timer_start(MAX_WAIT_TIME);           //开始计时
      while(1)
      {
        if(Timer_result())
        {
          pnode->Time--;
          if(pnode->Time == 0)
          {
            Del_Node(pnode->Addr);
            //Serial.print(pnode->Addr);Serial.println("lose contact");
            if(i == 3) 
            {
              if(Node_Link == pnode){Node_Link = NULL;}
            }
            break;
          }
        }
        if(RecvFrTer(&msg_r))
        {
          if(msg_r.SAddr == pnode->Addr && msg_r.Function == msg_s.Function)  
          { 
            pnode->Time == Time_Timeout;
            pnode->State &= ~StateMask[i];
            break;
          }
        }
      }
    }
    pnode = pnode->Next;
  }
}

//获取Link终端数据
void GetLinkData()
{
  String Data_toPC;  
  Message msg_s;
  Message msg_r;

  msg_s.SAddr = ADDRESS;                                                  //源地址为中心节点地址
  msg_s.DAddr = Node_Link->Addr; 
  msg_s.Function = '4';                                                   //功能为获取数据
  msg_s.MsgData = NULL_DATA;
  SendToTer(&msg_s); 
  Timer_start(MAX_WAIT_TIME);          //开始计时
  while(1)
  { 
    if(Timer_result())
    {
      Node_Link->State |= StateMask[4];
      Data_toPC="00000000";           //Link终端表示断开连接
      SendToPC(Data_toPC,'3');
      break;
    }
    if(RecvFrTer(&msg_r))
    {
      if(msg_r.SAddr == msg_s.DAddr && msg_r.Function == (byte)'4')
      { 
        Data_toPC = msg_r.MsgData;   //int转string后的string
        SendToPC(Data_toPC,'3');
        break;
      }
    }
  }
}

//=================================================================================================================
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
  String Command = "";                  //定义接收主机命令的变量
  if(RecvFrPC(&Command))                //判断是否有来自主机的命令
  {
    PC_Permit(Command);                 //入网允许命令
    PC_Logout(Command);                 //退网命令
    PC_ChangeFre(Command);              //改变网络参数命令
    PC_Link(Command);                   //更新Link终端命令
    PC_Update(Command);                 //更新网络状态表命令
    PC_BlackList(Command);              //更新黑名单命令
  }
  Broadcast();                          //广播通知附近设备可接入
  KeepContact();                        //与已接入设备保持联系
  AskExist();                           //再次联系可能断开连接的设备
  if(Node_Link != NULL) GetLinkData();  //对Link终端采集数据
  delay(100);                           
}
  /*
  NODE* pnode = NodeList;
  while(pnode != NULL)
  {
    Serial.println(pnode->Name);
    pnode = pnode->Next;
  }
  */

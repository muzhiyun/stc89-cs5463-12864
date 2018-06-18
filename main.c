//闲得*疼报了电子设计大赛  学长遗留的项目带了1k行程序 
//新手上路  死磕7小时磕到半夜两点 终于用单片机点亮一颗LED了  还好没猝死 
//找了一堆伪大神托人家帮改下代码  要么满嘴跑火车 指导老师被吓得玩失联   还得自己上
//整个架构推倒重写   自己加了一个判断函数 总算可以用了
//原来的1200行代码 被我加了几个读写e2prom的函数  不顺手又自己写了几个  变成900行  上一届学长是有多水  
//说了半天废话 说正事 项目是STC89C52+CS5463+12864   2017年国赛K题 单相用电器检测分析装置


#include <REG52.h>
#include <string.h>
#include <intrins.h>
#define uint  unsigned int
#define uchar unsigned char    //有冲突  暂时改为typedef
#define RdCommand 0x01 
#define PrgCommand 0x02
#define EraseCommand 0x03
#define Error 1
#define Ok 0
#define WaitTime 0x01 

typedef bit  bool;
typedef unsigned char  uint8;                   /* defined for unsigned 8-bits integer variable 	无符号8位整型变量  */
typedef signed   char  int8;                    /* defined for signed 8-bits integer variable		有符号8位整型变量  */
typedef unsigned int   uint16;                  /* defined for unsigned 16-bits integer variable 	无符号16位整型变量 */
typedef signed   int   int16;                   /* defined for signed 16-bits integer variable 		有符号16位整型变量 */
typedef unsigned long  uint32;                  /* defined for unsigned 32-bits integer variable 	无符号32位整型变量 */
typedef signed   long  int32;                   /* defined for signed 32-bits integer variable 		有符号32位整型变量 */
typedef float          fp32;                    /* single precision floating point variable (32bits) 单精度浮点数（32位长度） */
//typedef unsigned char  uchar;    

sfr ISP_DATA=0xe2; 
sfr ISP_ADDRH=0xe3;
sfr ISP_ADDRL=0xe4;
sfr ISP_CMD=0xe5;
sfr ISP_TRIG=0xe6;
sfr ISP_CONTR=0xe7;
sbit dula=P2^6; 
sbit wela=P2^7; 

/*****************************************************************************
                      12864串行接口定义
*****************************************************************************/
sbit LCM_cs   = P2^3;  //RS
sbit LCM_std  = P2^4;  //SID
sbit LCM_sclk = P2^5;  //SCLK
sbit LCM_psb  = P2^6;   //H=并口; L=串口;
sbit LCM_rst  = P2^7;   //Reset Signal 低电平有效	sbit LCM_rst  = P2^0; 
//sbit beep     = P2^5;
//sbit p2_4     = P2^6;  //定义背光控制口
 char a,b,c;
 char aa,bb,cc,dd,ee;
 char i,q,T=125;
uchar code tab1[]={
"电压            "
"功率            "
"电流            "
"可能是          "
};

/****************************************************************************
                            按键定义
******************************************************************************/
sbit K1 = P3^2;	//按钮1

sbit K2 = P3^3;	//按钮2

/******************************************************************
                           CS5463接口定义
******************************************************************/
sbit SCLK=P1^0;	 //ck
sbit MOSI=P1^1;	 //DI
sbit MISO=P1^2;	 //DO	  
sbit INT=P1^3;
sbit CS=P1^4;
sbit RST=P1^5;	//RST

#define CS5463_VScale       525             //计算电压比例,220V*250mv/110mv=500V
#define CS5463_IScale       (250/10)        //计算电流比例

static uint8 RX_Buff[4];					          //CS5463读写缓冲区
uint8 sta;									                //芯片状态

#define READ_MASK		  0xBF	                //读寄存器时的屏蔽码，与（写）地址相与
#define CMD_SYNC0    	0XFE                 	//结束串口重新初始化 
#define CMD_SYNC1    	0XFF                 	//开始串口重新初始化
#define REG_CONFR   	0x40                 	//配置 
#define REG_CYCCONT 	0x4A                 	//一个计算周期的A/D转换数 
#define REG_STATUSR 	0x5E 	                //状态 
#define REG_MODER   	0x64                 	//操作模式 
#define REG_MASKR   	0x74                 	//中断屏蔽 
#define REG_CTRLR   	0x78                 	//控制 
#define CMD_STARTC   	0XE8                 	//执行连续计算周期

#define REG_VRMSR   	0X18              	  //VRMS
#define REG_IRMSR   	0X16                 	//IRMS
#define REG_Pactive   0X14  	              //Pactive

/***************************************************************************************
**************************开始定义CS5463有关函数**************************
****************************************************************************************/

/*************************************************************
** CS5463函数名称:uDelay
** 函数功能:延时
**************************************************************/
static  void uDelay(uint8 j)

{ 
  uint8 i;
  for(;j>0;j--)
	{ for(i=0;i<255;i--)
		{
		;
		}
	}
}
/*************************************************************
** 函数名称:CS5463CMD
** 函数功能:CS5463命令函数
**************************************************************/
static void CS5463CMD(uint8 cmd)
{
 uint8 i;
 SCLK = 1;
 CS = 0;
 i = 0;
 while(i<8)
 {
  	uDelay(50);
	SCLK = 0;
	if(cmd&0x80)MOSI = 1;
	else		MOSI = 0;
	uDelay(50);
	SCLK = 1;			 		//在时钟上升沿，数据被写入CS5463
	cmd <<= 1;
	i++;
 }
 uDelay(50);
 CS = 1;
}
/*************************************************************
** 函数名称:CS5463WriteReg
** 函数功能:CS5463写寄存器函数
**************************************************************/
void CS5463WriteReg(uint8 addr,uint8 *p)
{
 uint8 i,j;
 uint8 dat;
 SCLK = 1;
 CS = 0;
 i = 0;
 while(i<8)
 {
  	uDelay(50);
	SCLK = 0;
	if(addr&0x80)MOSI = 1;
	else		 MOSI = 0;
	uDelay(50);
	SCLK = 1;		 			//在时钟上升沿，数据被写入CS5463
	addr <<= 1;
	i++;
 }
 j = 0;
 while(j<3)
 {
  	dat = *(p+j);
	i = 0;
	while(i<8)
	{
  		uDelay(50);
		SCLK = 0;
		if(dat&0x80)MOSI = 1;
		else		MOSI = 0;
		uDelay(50);
		SCLK = 1;		  		//在时钟上升沿，数据被写入CS5463
		dat <<= 1;
		i++;
	}
	j++;
 }
 uDelay(50);
 CS = 1;
}
/*************************************************************
** 函数名称:CS5463ReadReg
** 函数功能:CS5463读寄存器函数
**************************************************************/
void CS5463ReadReg(uint8 addr,uint8 *p)
{
 uint8 i,j;
 uint8 dat;
 SCLK = 0;
 CS = 0;
 addr &= READ_MASK;
 i = 0;
 while(i<8)
 {
  	uDelay(50);
	SCLK = 0;
	if(addr&0x80)MOSI = 1;
	else		 MOSI = 0;
	uDelay(50);
	SCLK = 1;
	addr <<= 1;				 	//在时钟上升沿，数据被写入CS5463
	i++;
 }
 uDelay(50);
 MOSI = 1;
 j = 0;
 while(j<3)
 {
	i = 0;
	dat = 0;
	while(i<8)
	{
		if(i==7)MOSI = 0;
		else	MOSI = 1;
		SCLK = 0;
		uDelay(50);
		dat <<= 1;			 			
		if(MISO)dat |= 0x01;
		else	dat &= 0xFE;
		SCLK = 1;
		uDelay(50);					 		
		i++;
	}
	*(p+j) = dat;
	j++;
 }
 MOSI = 1;
 CS = 1;
}
/*****************************************************************
** CS5463 应用函数
******************************************************************/

/*************************************************************
** 函数名称:CS5463Init
** 函数功能:CS5463复位和初始化函数
**************************************************************/
bit CS5463_Init(void)	   //bit 
{
 RST = 0;
 uDelay(200);
 RST = 1;
 uDelay(100);
//----------------------
//发送同步序列
 RX_Buff[0] = CMD_SYNC1;
 RX_Buff[1] = CMD_SYNC1;
 RX_Buff[2] = CMD_SYNC0;
 CS5463WriteReg(CMD_SYNC1,RX_Buff);	  //#define CMD_SYNC1    	0XFF  	//开始串口重新初始化		
//----------------------
//初始化--配置寄存器
//相位补偿为PC[6:0]=[0000000]；
//电流通道增益为Igain=10；
//EWA=0;
//INT中断为低电平有效IMODE:IINV=[00]
//iCPU=0
//K[3:0]=[0001]
 RX_Buff[0] = 0x00;					 	
 RX_Buff[1] = 0x00;
 RX_Buff[2] = 0x01;
 CS5463WriteReg(REG_CONFR,RX_Buff);	 //	#define REG_CONFR   	0x40   	//配置 
//----------------------
//初始化--操作寄存器
 RX_Buff[0] = 0x00; //B0000_0000;  //这是什么鬼 可以这样吗？ 					 	
 RX_Buff[1] = 0x00;//B0000_0000;
 RX_Buff[2] = 0x60;//B0110_0000;
 CS5463WriteReg(REG_MODER,RX_Buff);	  //#define REG_MODER   	0x64  	//操作模式 
 RX_Buff[0] = 0x00;
 RX_Buff[1] = 0x0F;
 RX_Buff[2] = 0xA0;						//#define REG_CYCCONT 	0x4A   	//一个计算周期的A/D转换数 
 CS5463WriteReg(REG_CYCCONT,RX_Buff);	//初始化--CYCLE COUNT 寄存器,4000

 RX_Buff[0] = 0xFF;
 RX_Buff[1] = 0xFF;
 RX_Buff[2] = 0xFF;
 CS5463WriteReg(REG_STATUSR,RX_Buff);	//初始化--状态寄存器  #define REG_STATUSR 	0x5E 	//状态 
//----------------------
 RX_Buff[0] = 0x80;						//开电流、电压、功率测量完毕中断
 RX_Buff[1] = 0x00;
 RX_Buff[2] = 0x80;						//开温度测量完毕中断
 CS5463WriteReg(REG_MASKR,RX_Buff);		//初始化--中断屏蔽寄存器    #define REG_MASKR   	0x74  	//中断屏蔽 
//----------------------
 RX_Buff[0] = 0x00;
 RX_Buff[1] = 0x00;
 RX_Buff[2] = 0x00;
 CS5463WriteReg(REG_CTRLR,RX_Buff);		//初始化--控制寄存器   #define REG_CTRLR   	0x78  	//控制  
//----------------------
 CS5463CMD(CMD_STARTC);				   	//启动连续转换	    #define CMD_STARTC   	0XE8  	//执行连续计算周期
 //CS5463_Status = 0;						//初始化任务进程状态
 //Load_Status = 0;
 //CS5463_CrmsSmallCunt = 0;
 //CS5463_CrmsOverCunt = 0;
 return(1);	  	  //只要做完这些步骤就返回true  1
}
/*************************************************************
** 函数名称:CS5463_ResetStatusReg
** 函数功能:复位状态寄存器函数
**************************************************************/
static void CS5463_ResetStatusReg(void)
{
 RX_Buff[0] = 0xFF;
 RX_Buff[1] = 0xFF;
 RX_Buff[2] = 0xFF;
 CS5463WriteReg(0x5E,RX_Buff);		//复位状态寄存器	#define REG_STATUSR 	0x5E 	//状态  
}
/*************************************************************
** 函数名称:CS5463_GetStatusReg
** 函数功能:读取状态寄存器函数
**************************************************************/
static uint8 CS5463_GetStatusReg(void)
{ 
 uint8 sta=0;
 CS5463ReadReg(0x1E,RX_Buff);	   //?状态寄存器
 if(RX_Buff[0]&0x80)		   			//检测：电流、电压、功率测量是否完毕
 {
	//检测电流/电压是否超出范围
	//检测电流有效值/电压有效值/电能是否超出范围
	if((RX_Buff[0]&0x03)||(RX_Buff[1]&0x70))
	{
	 	CS5463_ResetStatusReg();		//复位状态寄存器
	}
	else
	{
		sta |= 0x01;//B0000_0001;	
	}
 }

 if(RX_Buff[2]&0x80)			   	//检测：温度测量是否完毕
 {
  	sta |=0x02; //B0000_0010;
 } 
 return(sta);	
}  

/***************************************************************************************
**************************开始定义12864相关函数**************************
****************************************************************************************/

//uchar num;
uchar r[3]={0x00,0x00,0x00};

/******************************************************************************
延时函数
******************************************************************************/
void Delay(int num)
{
	while(num--);
}

/******************************************************************************
延时函数第二个
******************************************************************************/
void DelayM(unsigned int a)      
{
		unsigned char i;
		while( --a != 0)
       {		
		for(i = 0; i < 125; i++);  //?? ; ?????,CPU???0??125,CPU?????1???
		}   				   
}

/******************************************************************************/
//12864写指令或数据  （0，指令） （1，数据）
void LCM_WriteDatOrCom(bit dat_comm,uchar content)
 {
  uchar a,i,j;
  Delay(50);
  a=content;
  LCM_cs=1;
  LCM_sclk=0;
  LCM_std=1;
  for(i=0;i<5;i++)
  {
    LCM_sclk=1;
    LCM_sclk=0;
  }
  LCM_std=0;
  LCM_sclk=1;
  LCM_sclk=0;
  if(dat_comm)
    LCM_std=1;   //data
  else
   LCM_std=0;   //command
  LCM_sclk=1;
  LCM_sclk=0;
  LCM_std=0;
  LCM_sclk=1;
  LCM_sclk=0;
  for(j=0;j<2;j++)
  {
    for(i=0;i<4;i++)
    {
      a=a<<1;
      LCM_std=CY;
      LCM_sclk=1;
      LCM_sclk=0;
    }
    LCM_std=0;
    for(i=0;i<4;i++)
    {
      LCM_sclk=1;
      LCM_sclk=0;
    }
  }
}
/*********************************************************************************/


/*****************************************************************************/
//初始化LCM
void LCM_init(void)	 
{
  LCM_rst=1;
  LCM_psb=0;
  LCM_WriteDatOrCom (0,0x30);  /*30---基本指令动作*/   
  LCM_WriteDatOrCom (0,0x01);  /*清屏，地址指针指向00H*/
  Delay (100);
  LCM_WriteDatOrCom (0,0x06);  /*光标的移动方向*/
  LCM_WriteDatOrCom(0,0x0c);   /*开显示，关游标*/ 
 }

void chn_disp (uchar code *chn)   //显示4行 指针
{
  uchar i,j;
  LCM_WriteDatOrCom  (0,0x30);	 //	0 是指令 这2个命令 是干什么 的？
  LCM_WriteDatOrCom  (0,0x80);	 //
  for (j=0;j<4;j++)
  {
    for (i=0;i<16;i++)
    LCM_WriteDatOrCom  (1,chn[j*16+i]);
  }
}
/*****************************************************************************/
//清屏函数
void LCM_clr(void)
{
  LCM_WriteDatOrCom (0,0x30);
  LCM_WriteDatOrCom (0,0x01);   /*清屏，地址指针指向00H*/
  Delay (180);
}
/*****************************************************************************/
//向LCM发送一个字符串,长度64字符之内。
//应用：LCM_WriteString("您好！"); 
void LCM_WriteString(unsigned char *str)
{
		while(*str != '\0')
       {
			LCM_WriteDatOrCom(1,*str++);
        }
		*str = 0;	  
}




/***************************************************************************************
***********************五位数显示函数**************************
****************************************************************************************/
	void LCD_5num(uint32 num1)	
	{
		int a1,b1,c1,d1,e1;
		a1=	num1/10000;
	  LCM_WriteDatOrCom(1,a1+0x30);
   	b1=	(num1%10000)/1000;
	  LCM_WriteDatOrCom(1,b1+0x30);
	  c1=(num1%1000)/100;
  	LCM_WriteDatOrCom(1,c1+0x30);
	  d1=	(num1%100)/10;
	  LCM_WriteDatOrCom(1,d1+0x30);
  	e1=num1%10;
  	LCM_WriteDatOrCom(1,e1+0x30);
	}




/***************************************************************************************
**************************开始定义CS5463读取显示函数**************************
****************************************************************************************/

/*************************************************************
** 函数名称:CS5463_GetCurrentRMS
** 函数功能:读取电流有效值函数
**************************************************************/
static void CS5463_GetCurrentRMS(void)
{
 fp32 G = 0.5,result;
 uint32 temp1;
 uint8 temp,i,j;
 CS5463ReadReg(REG_IRMSR,RX_Buff);   		//读取电流有效值
 //SndCom1Data(RX_Buff,3);
 i = 0;
 result = 0;
 while(i<3)
 {
  	temp = RX_Buff[i];			   		
	j = 0;
	while(j<8)
	{
	 	if(temp&0x80)
		{
		 	result += G;	
		}
		temp <<= 1;
		j++;
		G = G/2;	
	}
	i++;
 }
 result = result*CS5463_IScale;
 result *= 1000;								//单位mA(毫安)  
 temp1 = (uint32)result;

 LCM_WriteDatOrCom  (0,0x94);
 LCD_5num(temp1);	
LCM_WriteString(" mA");
}



/*************************************************************
** 函数名称:CS5463_GetPactiveRMS
** 函数功能:读取有功功率函数
**************************************************************/
static uint32 CS5463_GetPactiveRMS(void)
{
 fp32 G = 1.0,result;
 uint8 temp,i,j;
 uint32 temp1;
 CS5463ReadReg(0x14,RX_Buff);   	//读取有功功率REG_Pactive
 //SndCom1Data(RX_Buff,3);
 temp = RX_Buff[0];
 if(temp&0x80)						  	//如果为负数，计算原码
 {
  	RX_Buff[0] = ~RX_Buff[0];			//本来为取反+1，这里因为精度的原因，不+1
	RX_Buff[1] = ~RX_Buff[1];
	RX_Buff[2] = ~RX_Buff[2];		 	
 }
 i = 0;
 result = 0;
 while(i<3)
 {
  	temp = RX_Buff[i];			   		
	j = 0;
	while(j<8)
	{
	 	if(temp&0x80)
		{
		 	result += G;	
		}
		temp <<= 1;
		j++;
		G = G/2;	
	}
	i++;
 }
// result = result*P_Coff;				//计算功率，单位W(瓦特)
// result = Vrms*Irms;					////////直接计算功率
  result = result*13125;
 temp1 = (uint32)result;
 
  LCM_WriteDatOrCom  (0,0x8C);		//26W  12345W
	LCD_5num(temp1);                 //功率不用小数显示 故调用LCD_5num函数
	LCM_WriteString(" W");
  return (temp1);
}


/*************************************************************
** 函数名称:CS5463_GetVoltRMS
** 函数功能:读取电压有效值函数
**************************************************************/
static void CS5463_GetVoltRMS(void)		 
{
 float G = 0.5,result;		//typedef float          fp32;	  就是浮点类型
 int temp1;			  //  int 
 uint8 temp,i,j;		  //  byte
 CS5463ReadReg(REG_VRMSR,RX_Buff);   		//读取电压有效值	    
 //SndCom1Data(RX_Buff,3);					//#define REG_VRMSR   	0x58	//电压有效值  电压有效值0x58吗？是写
 i = 0;
 result = 0;
 while(i<3)
 {
  	temp = RX_Buff[i];			   		
	j = 0;
	while(j<8)
	{
	 	if(temp&0x80)
		{
		 	result += G;	
		}
		temp <<= 1;
		j++;
		G = G/2;	
	}
	i++;								
 }						
result = result*CS5463_VScale;//V_Coff;				//计算电压值220V*250mv/(110mv/1.414)=704.8V	    可以暂时不用  
// if(result<=100)return;					//如果测量读出电压小于100V，确认读数错误
 result *= 100;						//单位为mV（毫伏） 
 temp1 = (uint32)result;


	LCM_WriteDatOrCom  (0,0x84);
	aa=	temp1/10000;
	LCM_WriteDatOrCom(1,aa+0x30);
	bb=	(temp1%10000)/1000;
	LCM_WriteDatOrCom(1,bb+0x30);
	cc=(temp1%1000)/100;
	LCM_WriteDatOrCom(1,cc+0x30);    // 电压需要小数点  故不调用LCD_5num函数
	LCM_WriteDatOrCom(1,0x2e);
	dd=	(temp1%100)/10;
	LCM_WriteDatOrCom(1,dd+0x30);
	ee=temp1%10;
	LCM_WriteDatOrCom(1,ee+0x30);
	LCM_WriteString(" V");
}

/*************************************************************
** 函数名称:CS5463_GetPowerFactor
** 函数功能:读取功率因数函数
**************************************************************/
static uint32 CS5463_GetPowerFactor(void)  
//static void CS5463_GetPowerFactor(void)   //开启后可以在第四行实时显示功率因数
{
 fp32 G = 1.0,result;
 uint8 temp,i,j;
 uint32 temp1;
 CS5463ReadReg(0x32,RX_Buff);   		//读取功率因数
 //SndCom1Data(RX_Buff,3);
 temp = RX_Buff[0];
 if(temp&0x80)						  	//如果为负数，计算原码
 {
  	RX_Buff[0] = ~RX_Buff[0];			//本来为取反+1，这里因为精度的原因，不+1
	RX_Buff[1] = ~RX_Buff[1];
	RX_Buff[2] = ~RX_Buff[2];		 	
 }
 i = 0;
 result = 0;
 while(i<3)
 {
  	temp = RX_Buff[i];			   		
	j = 0;
	while(j<8)
	{
	 	if(temp&0x80)
		{
		 	result += G;	
		}
		temp <<= 1;
		j++;
		G = G/2;	
	}
	i++;
 }
 result *= 10000;
 temp1 = (uint32)result;
 return (temp1);   //返回功率因数 注释后不再返回  关闭后在第四行实时显示功率因数
   //  LCM_WriteDatOrCom  (0,0x9C);     //在第四行显示
   //  LCD_5num(temp1);   //功率因数以无小数点格式输出
}


/***************************************************************************************
**************************开始定义EEPROM读写保持掉电保存相关函数**************************
****************************************************************************************/

void ISP_IAP_enable(void)

{

 EA = 0;       /* 关中断   */

 ISP_CONTR =ISP_CONTR & 0x18;       /* 0001,1000*/

 ISP_CONTR =ISP_CONTR | WaitTime; /* 写入硬件延时 */

 ISP_CONTR =ISP_CONTR | 0x80;       /* ISPEN=1  */

}

/* =============== 关闭 ISP,IAP 功能 ================== */

void ISP_IAP_disable(void)

{

 ISP_CONTR =ISP_CONTR & 0x7f; /*ISPEN = 0 */

 ISP_TRIG = 0x00;

 EA   =  1;   /* 开中断 */

}

/* ================ 公用的触发代码==================== */

void ISPgoon(void)

{

 ISP_IAP_enable();   /* 打开 ISP,IAP 功能 */

 ISP_TRIG =0x46;  /* 触发ISP_IAP命令字节1 */

 ISP_TRIG =0xb9;  /* 触发ISP_IAP命令字节2 */

 _nop_();

}

/* ==================== 字节读======================== */

unsigned char byte_read(unsigned int byte_addr)

{

 ISP_ADDRH =(unsigned char)(byte_addr >> 8);/* 地址赋值 */

 ISP_ADDRL =(unsigned char)(byte_addr & 0x00ff);

 ISP_CMD   = ISP_CMD & 0xf8;   /* 清除低3位  */

 ISP_CMD   = ISP_CMD | RdCommand; /* 写入读命令 */

 ISPgoon();       /* 触发执行  */

 ISP_IAP_disable();    /* 关闭ISP,IAP功能 */

 return(ISP_DATA);    /* 返回读到的数据 */

}

/* ================== 扇区擦除======================== */

void SectorErase(unsigned int sector_addr)

{

 unsigned int iSectorAddr;

 iSectorAddr =(sector_addr & 0xfe00); /* 取扇区地址 */

 ISP_ADDRH =(unsigned char)(iSectorAddr >> 8);

 ISP_ADDRL =0x00;

 ISP_CMD =ISP_CMD & 0xf8;   /* 清空低3位  */

 ISP_CMD = ISP_CMD| EraseCommand; /* 擦除命令3  */

 ISPgoon();       /* 触发执行  */

 ISP_IAP_disable();    /* 关闭ISP,IAP功能 */

}

/* ==================== 字节写======================== */

void byte_write(unsigned int byte_addr, unsigned char original_data)

{

 ISP_ADDRH =(unsigned char)(byte_addr >> 8); /* 取地址  */

 ISP_ADDRL =(unsigned char)(byte_addr & 0x00ff);

 ISP_CMD  = ISP_CMD & 0xf8;    /* 清低3位 */

 ISP_CMD  = ISP_CMD | PrgCommand;  /* 写命令2 */

 ISP_DATA =original_data;   /* 写入数据准备 */

 ISPgoon();       /* 触发执行  */

 ISP_IAP_disable();     /* 关闭IAP功能 */

}

/***************************************************************************************
**************************判断用电器函数**************************
****************************************************************************************/

void yongdianqi(void)	
	{
	uint32 temp1,num1;
	num1 = CS5463_GetPactiveRMS();   //获取功率
    temp1=CS5463_GetPowerFactor();   //获取功率因数
    LCM_WriteDatOrCom  (0,0x9C);     //在第四行显示
	
	if(1>num1)	    //功率小于1w
		{
			LCM_WriteString("无电器");
		}
	if(10>num1&&1<=num1)	//功率大于1w 小于10w 
		{		 
         if(temp1>5900)	   //实际测试发现LED功率因数大  视为呈阻性
	      LCM_WriteString("LED灯");
         else 
		 {
			 if (10>num1&&5<=num1)    
			   LCM_WriteString("充电器");
			 else LCM_WriteString("路由器");
		 }
        }
		
	if(40>num1&&10<=num1)	
		{		 
         if(temp1>5500)	
	      LCM_WriteString("节能灯");
		else
	       LCM_WriteString("机顶盒");
        }
		
	if(80>num1&&40<=num1)	
		{		 
        if(temp1>5500)	
	      LCM_WriteString("电风扇");
		else
	       LCM_WriteString("不告诉你");
		}
		
	if(2200>num1&&300<=num1)	
		{		 
        if(temp1>5500)	
	      LCM_WriteString("热水壶");
		else
	       LCM_WriteString("你猜啊");
		}
		
	
	}

/***************************************************************************************
**************************主函数**************************
****************************************************************************************/


void main() 
{
	
	CS5463_Init();
  LCM_init();       //初始化液晶显示器
	LCM_clr();       //清屏
	chn_disp(tab1); //显示欢迎字
	DelayM(500);  //显示等留3秒
	// LCM_clr();       //清屏
  //num1=byte_read(0x2000);
	//if(num1<0)
		//num1=0;
	
	  while(1)
	  {
			
		    sta	= CS5463_GetStatusReg();		  	//检测中断产生的原因
			CS5463_ResetStatusReg();			//清除标志
			CS5463_GetVoltRMS();				//获取电压
			CS5463_GetCurrentRMS();				//获取电流
			CS5463_GetPactiveRMS();				//获取功率
			//CS5463_GetPowerFactor();
			yongdianqi();
	
		//DelayM(2000);
		//num1++;
		//SectorErase(0x2000);	
    //byte_write(0x2000,num1);
		//测试读写内部EEPROM函数
		
		  
		  
		 /*   if(!K1)
		{
		 while(!K1);
		SectorErase(0x2000);	// 擦除第1个扇区
    num1=0;
		byte_write(0x2000,0);	// 对EEPROM区2002h写入2

	}

		if(!K2)

		{

		while(!K2);

		LED1=0;LED2=0;

		cc(0x2000);	// 擦除第1个扇区（2000h~21FFh）

		xcx(0x2002,6);	// 对EEPROM区2002h写入6

		} */
		//预留的按键切换开启学习模式函数
		
        }


}

#include "AS608.h"
#include "HardwareSerial.h"
#include <algorithm> // min函数的头文件
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
uint32_t AS608Addr = 0XFFFFFFFF; //默认
#define AS608_RXBUF_SIZE 256
static uint8_t as608_rxbuf[AS608_RXBUF_SIZE];
static uint16_t as608_rxlen = 0;
extern void IRAM_ATTR onSerial1Data() ;

extern uint8_t aRxBuffer[RXBUFFERSIZE];//接收缓冲
extern uint8_t RX_len;//接收字节计数
uint16_t ValidN;//模块内有效模板个数

extern 	TFT_eSPI tft ; 
void displayOnTFT(const char* content, int x, int y, int width, int height); // 前向声明函数
	uint16_t ID;
	uint16_t user_id;
	uint16_t fingerOK = 0; //匹配成功标志位
	uint16_t currentUser =1;
	uint16_t storageID = 0; // 存储指纹ID
	uint16_t OK_ = 0; // 录入成功标志位

// uint16_t i=0;//接收数据标志
// void UsartReceive(HardwareSerial &serial) {
// 	static size_t lastLength = 0;
// 	size_t availableBytes = serial.available(); // 获取当前接收缓冲区中的字节数
// 	if (availableBytes > 0) {
// 		size_t bytesToRead = std::min(availableBytes, RXBUFFERSIZE - lastLength); // 确保不会超出缓冲区大小
// 		serial.readBytes(aRxBuffer + lastLength, bytesToRead); // 读取数据到缓冲区
// 		lastLength += bytesToRead;
// 		RX_len = lastLength; // 记录接收数据长度
// 		if( RX_len > RXBUFFERSIZE) 
// 			{
// 				RX_len = RXBUFFERSIZE; // 确保不超过缓冲区大小
// 			}
// 		//lastLength = 0; 
// 		// if (lastLength >= RXBUFFERSIZE) {
// 		// 	// 缓冲区已满，处理数据
// 		// 	RX_len = lastLength; // 记录接收数据长度
// 		// 	if( RX_len > RXBUFFERSIZE) RX_len = RXBUFFERSIZE; // 确保不超过缓冲区大小
// 		// 	lastLength = 0; 
// 		// 	// 重置缓冲区计数
// 		// }
// 	}
// }


void AS608_SerialPoll() {
    while (Serial1.available()) {
		uint8_t data = Serial1.read();
        if (as608_rxlen < AS608_RXBUF_SIZE) {
            as608_rxbuf[as608_rxlen++] = data;
        } else {
            // 缓冲区满，丢弃最前面一个字节
            memmove(as608_rxbuf, as608_rxbuf + 1, --as608_rxlen);
            as608_rxbuf[as608_rxlen++] = Serial1.read();
        }
		//Serial.printf("AS608_SerialPoll received: 0x%02X\n", data);
    }
}
//录指纹
void Add_FR(void)
{
	OK_ =0;
	uint8_t i=0,ensure ,processnum=0;
	uint16_t a=0;

	if (currentUser == 1) {
		ID = 0; // 设置指纹ID为存储指纹ID
	} 
	else if(currentUser == 2) {
		ID = 1; // 设置指纹ID为0
	}

	while(1)
	{
		switch (processnum)
		{
			// case 0:
			// 	i++;
			// 	displayOnTFT("请按手指", 0, 32, tft.width(), 16); // 只清除并填充第二行
			// 	Serial.println("请按手指");
			// 	ensure=GZ_GetImage();
			// 	if(ensure==0x00) 
			// 	{
			// 		ensure=GZ_GenChar(CharBuffer1);//生成特征
			// 		Serial.println("生成特征成功");
			// 		displayOnTFT("生成特征成功", 0, 48, tft.width(), 16); // 只清除并填充第三行
			// 		if(ensure==0x00)
			// 		{
			// 			i=0;
			// 			processnum=1;//跳到第二步						
			// 		}			
			// 	}						
			// 	break;
			case 0:
				i++;
				//displayOnTFT("请按手指", 0, 32, tft.width(), 16);
				Serial.printf("请按手指(模板编号：%d)\n", ID+1);
				ensure = GZ_GetImage();
				if (ensure == 0x00) {
					ensure = GZ_GenChar(CharBuffer1);
					if (ensure == 0x00) {
						Serial.println("生成特征成功");
						//displayOnTFT("生成特征成功", 0, 48, tft.width(), 16);
						i = 0;
						processnum = 1; // 跳到第二步
						Serial.println("跳转到第二步");
					} else {
						Serial.println("生成特征失败");
						processnum = 0; // 重新开始
					}
				} 
				break;
			case 1:
				i++;
				ensure=GZ_GetImage();
				if(ensure==0x00) 
				{
					ensure=GZ_GenChar(CharBuffer2);//生成特征			
					if(ensure==0x00)
					{
						i=0;
						processnum=2;//跳到第三步
					}
					else
					{
						Serial.println("回0,录入重新开始");
						processnum=0; // 重新开始
					}
				}	
				break;

			case 2:
				ensure=GZ_Match();
				if(ensure==0x00) 
				{
					processnum=3;//跳到第四步
				}
				else 
				{
					i=0;
					processnum=0;//跳回第一步		
				}
				//vTaskDelay(pdMS_TO_TICKS(1000)); // 延时后清除显示
				//delay(1000);
				break;

			case 3:
				ensure=GZ_RegModel();
				if(ensure==0x00) 
				{
					processnum=4;//跳到第五步
				}else {processnum=0;}
				//vTaskDelay(pdMS_TO_TICKS(1000)); // 延时后清除显示
				//delay(1000);
				break;
				
			case 4:	
				ensure=GZ_StoreChar(CharBuffer2,ID);//储存模板
				if(ensure==0x00) 
				{	
					//displayOnTFT("录入成功", 0, 64, tft.width(), 16); // 只清除并填充第四行
					Serial.printf("录入成功,模板编号：%d", ID+1);
					GZ_ValidTempleteNum(&ValidN);//读库指纹个数
						// Serial.print("当前指纹数量: ");
						// Serial.println(ValidN); // 打印指纹数量
						ID++;
						OK_ = 1;
					vTaskDelay(pdMS_TO_TICKS(1500)); // 延时后清除显示
					return ;
				}else {processnum=0;}					
				break;				
		}
		vTaskDelay(pdMS_TO_TICKS(1000)); // 延时后清除显示
		//delay(800);
		if(i==10)//超过10次没有按手指则退出
		{
				 if (a<=0)
				 {
					Serial.println("录入超时");
					//displayOnTFT("录入超时", 0, 64, tft.width(), 16); // 只清除并填充第四行
				 	a=1;
				 }
			
			break;	
		}				
	}
}
SearchResult seach;
uint8_t press_FR(void) {
    uint8_t ensure;
    //displayOnTFT("请按手指", 0, 16, tft.width(), 16); // 只清除并填充第一行
    ensure = GZ_GetImage();
    if (ensure == 0x00) { // 获取图像成功
        ensure = GZ_GenChar(CharBuffer1);
        if (ensure == 0x00) { // 生成特征成功
            ensure = GZ_HighSpeedSearch(CharBuffer1, 0, 300, &seach);
			if(ensure==0x00)//搜索成功
			{	
				if(seach.mathscore>100)
							{
								Serial.printf("匹配成功，模板编号：%d，匹配分数：%d\n", seach.pageID+1, seach.mathscore);
								//vTaskDelay(pdMS_TO_TICKS(1500));
								return 1; // 匹配成功，返回 1
							}else
							{
								Serial.println("匹配失败");
								return 0; // 匹配失败，返回 0
							}
			}else
			{
				//tft.printf("cuo wu");
				Serial.println("错误");
				return 0; // 错误，返回 0
			}
        }
    }
}
// //刷指纹
// void press_FR(void)
// {
// 	SearchResult seach;
// 	uint8_t ensure;
// 	char *str;
	
// 	displayOnTFT("请按手指", 0, 16, tft.width(), 16); // 只清除并填充第一行
// 	//Serial.println("请按指纹");
// 	ensure=GZ_GetImage();
// 	if(ensure==0x00)//获取图像成功 
// 	{	
// 		ensure=GZ_GenChar(CharBuffer1);
// 		if(ensure==0x00) //生成特征成功
// 		{		
// 			ensure=GZ_HighSpeedSearch(CharBuffer1,0,300,&seach);
// 			if(ensure==0x00)//搜索成功
// 			{	
//           if(seach.mathscore>100)
// 					{
// 						displayOnTFT("匹配成功", 0, 80, tft.width(), 16); // 只清除并填充第五行
// 						Serial.printf("匹配成功,目标编号：%d", seach.pageID+1);
// 						//delay(20000);
// 						vTaskDelay(pdMS_TO_TICKS(2000)); // 延时后清除显示
// 						fingerOK = 1; //匹配成功标志位
// 					}else
// 					{
// 						displayOnTFT("匹配失败", 0, 80, tft.width(), 16); // 只清除并填充第五行
// 						Serial.println("匹配失败");
// 					}

// 			}else
// 			{
// 				displayOnTFT("匹配失败", 0, 80, tft.width(), 16); // 只清除并填充第五行
// 				Serial.println("匹配失败");
// 			}

// 	  }
// 	}
		
// }

//串口发送一个字节
static void Com_SendData(uint8_t data)
{
	
	Serial1.write(data);
}
//发送包头
static void SendHead(void)
{
	Com_SendData(0xEF);
	Com_SendData(0x01);
}
//发送地址
static void SendAddr(void)
{
	Com_SendData(AS608Addr>>24);
	Com_SendData(AS608Addr>>16);
	Com_SendData(AS608Addr>>8);
	Com_SendData(AS608Addr);
}
//发送包标识,
static void SendFlag(uint8_t flag)
{
	Com_SendData(flag);
}
//发送包长度
static void SendLength(int length)
{
	Com_SendData(length>>8);
	Com_SendData(length);
}
//发送指令码
static void Sendcmd(uint8_t cmd)
{
	Com_SendData(cmd);
}
//发送校验和
static void SendCheck(uint16_t check)
{
	Com_SendData(check>>8);
	Com_SendData(check);
}

//判断中断接收的数组有没有应答包
//waittime为等待中断接收数据的时间（单位1ms）
//返回值：数据包首地址
// extern uint8_t RX_len;//接收字节计数
// static uint8_t *JudgeStr(uint16_t waittime)
// {
// 	char *data;
// 	uint8_t str[8];
// 	str[0]=0xef;
// 	str[1]=0x01;
// 	str[2]=AS608Addr>>24;
// 	str[3]=AS608Addr>>16;
// 	str[4]=AS608Addr>>8;
// 	str[5]=AS608Addr;
// 	str[6]=0x07;
// 	str[7]='\0';
// 	// RX_len = 0; // 清空接收字节计数
// 	// memset(aRxBuffer, 0, sizeof(aRxBuffer));
// 	// while(--waittime)
// 	// {
// 	// 	//UsartReceive_IDLE(); // 调用接收函数
// 	// 	//vTaskDelay(pdMS_TO_TICKS(1)); // 延时1ms，避免过快的循环
// 	// 	delay(1);
//     //     while (Serial1.available()) {
//     //         aRxBuffer[RX_len++] = Serial1.read();
//     //     }
//     //     if (RX_len) {
//     //         RX_len = 0;
//     //         data = strstr((const char *)aRxBuffer, (const char *)str);
//     //         if (data) {
//     //             return (uint8_t *)data;
//     //         }
//     //     }
//     // }\
	
// 	//memset(aRxBuffer, 0, sizeof(aRxBuffer)); // 清空接收缓冲区
//     while (--waittime) {
//         vTaskDelay(pdMS_TO_TICKS(1)); // 延时1ms，避免过快的循环

//         while (Serial1.available()) {
//                 aRxBuffer[RX_len++] = Serial1.read();
//         }

//         if (RX_len ) { // 检查是否接收到至少9字节的数据包
//             // 打印接收到的数据（调试用）
//             Serial.print("Received data: ");
//             for (int i = 0; i < RX_len; i++) {
//                 Serial.printf("0x%02X ", aRxBuffer[i]);
//             }
//             Serial.println();
// 			RX_len = 0; // 清空接收字节计数
//             data = strstr((const char *)aRxBuffer, (const char *)str);
//             if (data) {
//                 return (uint8_t *)data;
//             }
//         }
//     }
// 	Serial.println("没有接收到指纹数据包");
// 	Serial1.onReceive(onSerial1Data); // 注册中断回调函数
// 	return 0;
// }
static uint8_t *JudgeStr(uint16_t waittime)
{
    uint8_t str[7];
    str[0]=0xef;
    str[1]=0x01;
    str[2]=AS608Addr>>24;
    str[3]=AS608Addr>>16;
    str[4]=AS608Addr>>8;
    str[5]=AS608Addr;
    str[6]=0x07;

    while (--waittime) {
        vTaskDelay(pdMS_TO_TICKS(1));
        AS608_SerialPoll();

        // 至少要有包头+长度字段
        if (as608_rxlen >= 9) {
            for (int i = 0; i <= as608_rxlen - 7; i++) {
                if (memcmp(as608_rxbuf + i, str, 7) == 0) {
                    // 包头后第7、8字节是包长度
                    uint16_t packetLen = (as608_rxbuf[i+7] << 8) | as608_rxbuf[i+8];
                    uint16_t totalLen = 9 + packetLen;
                    if (as608_rxlen - i >= totalLen) {
                        // 找到完整包
                        // Serial.print("JudgeStr: 找到完整包: ");
                        // for (int j = 0; j < totalLen; j++) {
                        //     Serial.printf("0x%02X ", as608_rxbuf[i+j]);
                        // }
                        // Serial.println();
                        // 可选：清空缓冲区
                        as608_rxlen = 0;
                        return as608_rxbuf + i;
                    }
                }
            }
        }
    }
    Serial.println("没有接收到指纹数据包");
    return 0;
}

//录入图像 GZ_GetImage
//功能:探测手指，探测到后录入指纹图像存于ImageBuffer。 
//模块返回确认字
uint8_t GZ_GetImage(void)
{
  uint16_t temp;
  uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x03);
	Sendcmd(0x01);
  temp =  0x01+0x03+0x01;
	SendCheck(temp);
	data=JudgeStr(2000);
	
	if(data){
		ensure=data[9];
		//Serial.printf("GZ_GetImage=%d\n",data[9]);
	}
	else{
		ensure=0xff;
		//Serial.println("GZ_GetImage error");
	}
	return ensure;
}
//生成特征 GZ_GenChar
//功能:将ImageBuffer中的原始图像生成指纹特征文件存于CharBuffer1或CharBuffer2			 
//参数:BufferID --> charBuffer1:0x01	charBuffer1:0x02												
//模块返回确认字
uint8_t GZ_GenChar(uint8_t BufferID)
{
	uint16_t temp;
  uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x04);
	Sendcmd(0x02);
	Com_SendData(BufferID);
	temp = 0x01+0x04+0x02+BufferID;
	SendCheck(temp);
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;
}
//精确比对两枚指纹特征 GZ_Match
//功能:精确比对CharBuffer1 与CharBuffer2 中的特征文件 
//模块返回确认字
uint8_t GZ_Match(void)
{
	uint16_t temp;
  uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x03);
	Sendcmd(0x03);
	temp = 0x01+0x03+0x03;
	SendCheck(temp);
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;
}
//搜索指纹 GZ_Search
//功能:以CharBuffer1或CharBuffer2中的特征文件搜索整个或部分指纹库.若搜索到，则返回页码。			
//参数:  BufferID @ref CharBuffer1	CharBuffer2
//说明:  模块返回确认字，页码（相配指纹模板）
uint8_t GZ_Search(uint8_t BufferID,uint16_t StartPage,uint16_t PageNum,SearchResult *p)
{
	uint16_t temp;
  uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x08);
	Sendcmd(0x04);
	Com_SendData(BufferID);
	Com_SendData(StartPage>>8);
	Com_SendData(StartPage);
	Com_SendData(PageNum>>8);
	Com_SendData(PageNum);
	temp = 0x01+0x08+0x04+BufferID
	+(StartPage>>8)+(uint8_t)StartPage
	+(PageNum>>8)+(uint8_t)PageNum;
	SendCheck(temp);
	data=JudgeStr(2000);
	if(data)
	{
		ensure = data[9];
		p->pageID   =(data[10]<<8)+data[11];
		p->mathscore=(data[12]<<8)+data[13];	
	}
	else
		ensure = 0xff;
	return ensure;	
}
//合并特征（生成模板）GZ_RegModel
//功能:将CharBuffer1与CharBuffer2中的特征文件合并生成 模板,结果存于CharBuffer1与CharBuffer2	
//说明:  模块返回确认字
uint8_t GZ_RegModel(void)
{
	uint16_t temp;
  uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x03);
	Sendcmd(0x05);
	temp = 0x01+0x03+0x05;
	SendCheck(temp);
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;		
}
//储存模板 GZ_StoreChar
//功能:将 CharBuffer1 或 CharBuffer2 中的模板文件存到 PageID 号flash数据库位置。			
//参数:  BufferID @ref charBuffer1:0x01	charBuffer1:0x02
//       PageID（指纹库位置号）
//说明:  模块返回确认字
uint8_t GZ_StoreChar(uint8_t BufferID,uint16_t PageID)
{
	uint16_t temp;
  uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x06);
	Sendcmd(0x06);
	Com_SendData(BufferID);
	Com_SendData(PageID>>8);
	Com_SendData(PageID);
	temp = 0x01+0x06+0x06+BufferID
	+(PageID>>8)+(uint8_t)PageID;
	SendCheck(temp);
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;	
}

//删除模板 GZ_DeletChar
//功能:  删除flash数据库中指定ID号开始的N个指纹模板
//参数:  PageID(指纹库模板号)，N删除的模板个数。
//说明:  模块返回确认字
uint8_t GZ_DeletChar(uint16_t PageID,uint16_t N)
{
	uint16_t temp;
  uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x07);
	Sendcmd(0x0C);
	Com_SendData(PageID>>8);
	Com_SendData(PageID);
	Com_SendData(N>>8);
	Com_SendData(N);
	temp = 0x01+0x07+0x0C+(PageID>>8)+(uint8_t)PageID+(N>>8)+(uint8_t)N;
	SendCheck(temp);
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;
}
//清空指纹库 GZ_Empty
//功能:  删除flash数据库中所有指纹模板
//参数:  无
//说明:  模块返回确认字
uint8_t GZ_Empty(void)
{
	uint16_t temp;
  uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x03);
	Sendcmd(0x0D);
	temp = 0x01+0x03+0x0D;
	SendCheck(temp);
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;
}
//写系统寄存器 GZ_WriteReg
//功能:  写模块寄存器
//参数:  寄存器序号RegNum:4\5\6
//说明:  模块返回确认字
uint8_t GZ_WriteReg(uint8_t RegNum,uint8_t DATA)
{
	uint16_t temp;
  uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x05);
	Sendcmd(0x0E);
	Com_SendData(RegNum);
	Com_SendData(DATA);
	temp = RegNum+DATA+0x01+0x05+0x0E;
	SendCheck(temp);
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;
}
//读系统基本参数 GZ_ReadSysPara
//功能:  读取模块的基本参数（波特率，包大小等)
//参数:  无
//说明:  模块返回确认字 + 基本参数（16bytes）
uint8_t GZ_ReadSysPara(SysPara *p)
{
	uint16_t temp;
  uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x03);
	Sendcmd(0x0F);
	temp = 0x01+0x03+0x0F;
	SendCheck(temp);
	data=JudgeStr(2000);
	if(data)
	{
		ensure = data[9];
		Serial.printf("GZ_ReadSysPara=%d\n",ensure);
		p->GZ_max = (data[14]<<8)+data[15];
		p->GZ_level = data[17];
		p->GZ_addr=(data[18]<<24)+(data[19]<<16)+(data[20]<<8)+data[21];
		p->GZ_size = data[23];
		p->GZ_N = data[25];
	}		
	else
	{
			Serial.println("No data received from JudgeStr");
		ensure=0xff;
	}
	return ensure;
}
//设置模块地址 GZ_SetAddr
//功能:  设置模块地址
//参数:  GZ_addr
//说明:  模块返回确认字
uint8_t GZ_SetAddr(uint32_t GZ_addr)
{
	uint16_t temp;
  uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x07);
	Sendcmd(0x15);
	Com_SendData(GZ_addr>>24);
	Com_SendData(GZ_addr>>16);
	Com_SendData(GZ_addr>>8);
	Com_SendData(GZ_addr);
	temp = 0x01+0x07+0x15
	+(uint8_t)(GZ_addr>>24)+(uint8_t)(GZ_addr>>16)
	+(uint8_t)(GZ_addr>>8) +(uint8_t)GZ_addr;				
	SendCheck(temp);
	AS608Addr=GZ_addr;//发送完指令，更换地址
  data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;	
		AS608Addr = GZ_addr;
	if(ensure==0x00)//设置地址成功
	{
		
	}
	return ensure;
}
//功能： 模块内部为用户开辟了256bytes的FLASH空间用于存用户记事本,
//	该记事本逻辑上被分成 16 个页。
//参数:  NotePageNum(0~15),Byte32(要写入内容，32个字节)
//说明:  模块返回确认字
uint8_t GZ_WriteNotepad(uint8_t NotePageNum,uint8_t *Byte32)
{
	uint16_t temp;
  uint8_t  ensure,i;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(36);
	Sendcmd(0x18);
	Com_SendData(NotePageNum);
	for(i=0;i<32;i++)
	 {
		 Com_SendData(Byte32[i]);
		 temp += Byte32[i];
	 }
  temp =0x01+36+0x18+NotePageNum+temp;
	SendCheck(temp);
  data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;
}
//读记事GZ_ReadNotepad
//功能：  读取FLASH用户区的128bytes数据
//参数:  NotePageNum(0~15)
//说明:  模块返回确认字+用户信息
uint8_t GZ_ReadNotepad(uint8_t NotePageNum,uint8_t *Byte32)
{
	uint16_t temp;
  uint8_t  ensure,i;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x04);
	Sendcmd(0x19);
	Com_SendData(NotePageNum);
	temp = 0x01+0x04+0x19+NotePageNum;
	SendCheck(temp);
  data=JudgeStr(2000);
	if(data)
	{
		ensure=data[9];
		for(i=0;i<32;i++)
		{
			Byte32[i]=data[10+i];
		}
	}
	else
		ensure=0xff;
	return ensure;
}
//高速搜索GZ_HighSpeedSearch
//功能：以 CharBuffer1或CharBuffer2中的特征文件高速搜索整个或部分指纹库。
//		  若搜索到，则返回页码,该指令对于的确存在于指纹库中 ，且登录时质量
//		  很好的指纹，会很快给出搜索结果。
//参数:  BufferID， StartPage(起始页)，PageNum（页数）
//说明:  模块返回确认字+页码（相配指纹模板）
uint8_t GZ_HighSpeedSearch(uint8_t BufferID,uint16_t StartPage,uint16_t PageNum,SearchResult *p)
{
	uint16_t temp;
  uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x08);
	Sendcmd(0x1b);
	Com_SendData(BufferID);
	Com_SendData(StartPage>>8);
	Com_SendData(StartPage);
	Com_SendData(PageNum>>8);
	Com_SendData(PageNum);
	temp = 0x01+0x08+0x1b+BufferID
	+(StartPage>>8)+(uint8_t)StartPage
	+(PageNum>>8)+(uint8_t)PageNum;
	SendCheck(temp);
	data=JudgeStr(2000);
 	if(data)
	{
		ensure=data[9];
		p->pageID 	=(data[10]<<8) +data[11];
		p->mathscore=(data[12]<<8) +data[13];
	}
	else
		ensure=0xff;
	return ensure;
}
//读有效模板个数 GZ_ValidTempleteNum
//功能：读有效模板个数
//参数: 无
//说明: 模块返回确认字+有效模板个数ValidN
uint8_t GZ_ValidTempleteNum(uint16_t *ValidN)
{
	uint16_t temp;
	uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x03);
	Sendcmd(0x1d);
	temp = 0x01+0x03+0x1d;
	SendCheck(temp);
	data=JudgeStr(2000);
	if(data)
	{
		ensure=data[9];
		*ValidN = (data[10]<<8) +data[11];
		Serial.println("GZ_ValidTempleteNum: Success");
	}		
	else
	{
		ensure=0xff;
		Serial.println("GZ_ValidTempleteNum: Failed");
	}
	
	return ensure;
}
//与AS608握手 GZ_HandShake
//参数: GZ_Addr地址指针
//说明: 模块返新地址（正确地址）	
// uint8_t GZ_HandShake(uint32_t *GZ_Addr)
// {
// 	SendHead();
// 	SendAddr();
// 	Com_SendData(0X01);
// 	Com_SendData(0X00);
// 	Com_SendData(0X00);	
// 	//UsartReceive_IDLE() ; // 读取串口数据
// 	vTaskDelay(pdMS_TO_TICKS(200)); // 
// 	//delay(200);
// 	if (RX_len) {
		
// 		if (RX_len >= 9) {
// 			// 打印接收到的数据（调试用）
// 			Serial.print("Received data: ");
// 			for (int i = 0; i < RX_len; i++) {
// 				Serial.printf("0x%02X ", aRxBuffer[i]);
// 			}
// 			Serial.println();
			
// 			}
        
//         if (aRxBuffer[0] == 0xEF && aRxBuffer[1] == 0x01 && aRxBuffer[6] == 0x07) {
//             *GZ_Addr = (aRxBuffer[2] << 24) + (aRxBuffer[3] << 16)
//                        + (aRxBuffer[4] << 8) + aRxBuffer[5];
// 			RX_len = 0;		   
//             return 1;
//         }
//     }
//     return 0;

// }
//与AS608握手 GZ_HandShake
//参数: GZ_Addr地址指针
//说明: 模块返新地址（正确地址）	
uint8_t GZ_HandShake(uint32_t *GZ_Addr)
{
    SendHead();
    SendAddr();
    Com_SendData(0X01);
    Com_SendData(0X00);
    Com_SendData(0X00);

    // 等待数据到来，最多200ms
    uint16_t waittime = 200;
    while (waittime--) {
        vTaskDelay(pdMS_TO_TICKS(1));
        AS608_SerialPoll();

        if (as608_rxlen >= 9) {
            // 打印接收到的数据（调试用）
            Serial.print("Received data: ");
            for (int i = 0; i < as608_rxlen; i++) {
                Serial.printf("0x%02X ", as608_rxbuf[i]);
            }
            Serial.println();

            // 判断是否为应答包
            if (as608_rxbuf[0] == 0xEF && as608_rxbuf[1] == 0x01 && as608_rxbuf[6] == 0x07) {
                *GZ_Addr = (as608_rxbuf[2] << 24) + (as608_rxbuf[3] << 16)
                         + (as608_rxbuf[4] << 8) + as608_rxbuf[5];
                as608_rxlen = 0;
                return 1;
            }
        }
    }
    Serial.println("没有接收到握手应答包");
    return 0;
}
//模块应答包确认码信息解析
//功能：解析确认码错误信息返回信息
//参数: ensure
const char *EnsureMessage(uint8_t ensure) 
{
	const char *p;
	switch(ensure)
	{
		case  0x00:
			p="OK";break;		
		case  0x01:
			p="数据包接收错误";break;
		case  0x02:
			p="传感器上没有手指";break;
		case  0x03:
			p="录入指纹图像失败";break;
		case  0x04:
			p="指纹图像太干、太淡而生不成特征";break;
		case  0x05:
			p="指纹图像太湿、太糊而生不成特征";break;
		case  0x06:
			p="指纹图像太乱而生不成特征";break;
		case  0x07:
			p="指纹图像正常，但特征点太少（或面积太小）而生不成特征";break;
		case  0x08:
			p="指纹不匹配";break;
		case  0x09:
			p="没搜索到指纹";break;
		case  0x0a:
			p="特征合并失败";break;
		case  0x0b:
			p="访问指纹库时地址序号超出指纹库范围";
		case  0x10:
			p="删除模板失败";break;
		case  0x11:
			p="清空指纹库失败";break;	
		case  0x15:
			p="缓冲区内没有有效原始图而生不成图像";break;
		case  0x18:
			p="读写 FLASH 出错";break;
		case  0x19:
			p="未定义错误";break;
		case  0x1a:
			p="无效寄存器号";break;
		case  0x1b:
			p="寄存器设定内容错误";break;
		case  0x1c:
			p="记事本页码指定错误";break;
		case  0x1f:
			p="指纹库满";break;
		case  0x20:
			p="地址错误";break;
		default :
			p="模块返回确认码有误";break;
	}
 return p;	
}



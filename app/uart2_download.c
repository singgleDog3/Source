#include "uart2_download.h"
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "ff.h"
#include "led.h"


uint8_t ConnSuccess_ACK[] = {'b'+100, 'l'+100,'u'+100,'e'+100,'s'+100,'p'+100,'e'+100, 'a'+100, 'k'+100};
uint32_t AckNum;

uint32_t PatchInfo_TotalNum;
uint32_t PatchInfo_BeTransferedNum;
uint32_t PatchInfo_LeftNum;

uint32_t  ParamInfo_TotalNum;
uint32_t  ParamInfo_BeTransferedNum;
uint32_t  ParamInfo_LeftNum;	
uint32_t  ParamInfo_BaseAddr;
uint32_t  ParamInfo_BlueAddr_offset;

uint32_t DownlaodOK_Num = 0;
//  read  info 
uint32_t ReadTotal = 0;
uint32_t ReadLeft  = 0;
uint32_t ReadNum   = 0;
uint32_t ReadSize  = 0;
	
#define OneTime_Num (512)
#define PatchBaseAddr  (0x800040e4)	

#define ChangeBaudRate (1)
	
uint8_t DataBuf[OneTime_Num];
uint8_t ReadDataBuf[OneTime_Num + 9];

void AckNum_Clear(void)
{
	AckNum = 0;
}

uint32_t GetReadSize(void)
{
	if(ReadLeft > OneTime_Num)
		return OneTime_Num;
	else
		return ReadLeft;
}

void ReadInfo_Init(void)
{
	ReadTotal = ParamInfo_TotalNum;
	ReadLeft  = ReadTotal;
	ReadNum   = 0;
}
	
void ReadInfo_Update(void)
{
	uint32_t size;
	size = GetReadSize();
	ReadNum += size;
	ReadLeft = ReadTotal - ReadNum;
}

void PatchInfo_Init(uint32_t total)
{
	PatchInfo_TotalNum = total;
	PatchInfo_LeftNum = total;
	PatchInfo_BeTransferedNum = 0;
}

void IntToStr(uint8_t *str, uint32_t num)
{
	*str++ = num>>24;
	*str++ = num>>16;
	*str++ = num>>8;
	*str   = num>>0 ;
}

void ParamInfo_Init(uint32_t param_total, uint32_t patch_total)
{
	ParamInfo_TotalNum = param_total;
	ParamInfo_BeTransferedNum = 0;
	ParamInfo_LeftNum = param_total;
	ParamInfo_BaseAddr = patch_total;
	ParamInfo_BlueAddr_offset = 0x68;
}

enum UART2Download_State UART2Doanload_State ;

void My_strcpy(char *str1, char *str2, uint32_t num)
{
		uint32_t temp;
		while(temp < num){
			*(str1 + temp) = *(str2 + temp);
			temp++;		
		}

}


//接收到连接请求后发送  
void uart2_SuccessConnACK(void)
{
	uart2_WriteStr((char *)ConnSuccess_ACK, sizeof(ConnSuccess_ACK));
}

//读取Flash中的数据
void uart2_tryReadFlash(void)
{
	uint8_t ReadFlash_CMD[] = {0x05, 0x00,0x00,0x00,0x00,  0x00,0x00,0x00,0x04};
	uart2_WriteStr((char *)ReadFlash_CMD, sizeof(ReadFlash_CMD));
}


uint8_t Read_FlashRead_ACK(char data){
	static uint8_t ack[13];
	uint8_t STD_ack[9] = {0x10, 0x00,0x00,0x00,0x00,  0x00,0x00,0x00,0x04};
	ack[AckNum] = data;
	if(AckNum == 12){
		if(memcmp(STD_ack, ack, 9) == 0){
			return TRUE;
		}
		return FALSE;
	}
	AckNum++;
	return READING;
}

// PatchBaseAddr  0x800040e4
void uart2_PatchWrite_REQ(void)
{
	uint8_t req[9];
	uint32_t addr = PatchBaseAddr + PatchInfo_BeTransferedNum;
	uint32_t size;
	
	if( PatchInfo_LeftNum > OneTime_Num){
		size = OneTime_Num;
	}else{
		size = PatchInfo_LeftNum;
	}
	
	req[0]  = 0x04;
	//addr
	req[1] = addr >> 24;
	req[2] = addr >> 16;
	req[3] = addr >> 8;
	req[4] = addr >> 0;
	// size 
	req[5] = size >> 24;
	req[6] = size >> 16;
	req[7] = size >> 8;
	req[8] = size >> 0;
	
	uart2_WriteStr((char *)req, sizeof(req));
}

uint8_t Read_PatchWrite_ACK(char data)
{
	static uint8_t ack[9] ;
	uint8_t STD_ack[9];
	uint32_t addr ;
	uint32_t size;
	
	ack[AckNum] = data;
	if(AckNum == 8){
		addr = PatchBaseAddr + PatchInfo_BeTransferedNum;
		if(PatchInfo_LeftNum > OneTime_Num){
			size = OneTime_Num;
		}else{
			size = PatchInfo_LeftNum;
		}
		STD_ack[0] = 0x40;
		STD_ack[1] = addr >> 24;
		STD_ack[2] = addr >> 16;
		STD_ack[3] = addr >> 8;
		STD_ack[4] = addr >> 0;
		
		STD_ack[5] = size >> 24;
		STD_ack[6] = size >> 24;
		STD_ack[7] = size >> 24;
		STD_ack[8] = size >> 24;
		
		if(memcpy(STD_ack, ack, sizeof(ack)) == 0){
			return TRUE;
		}else{
			return TRUE;
		}
	}
	AckNum++;
	return READING;
}

void uart2_PatchDataWrite(void)
{
	uint32_t size;
	uint32_t temp;
	
	if(PatchInfo_LeftNum > OneTime_Num){
		size = OneTime_Num;
	}else{
		size = PatchInfo_LeftNum;
	}
#if 0
	for(temp = 0; temp < size; temp++){
		DataBuf[temp] = 0xaa;
	}
#else
	//get patch data from sram
	FSMC_SRAM_ReadBuffer(DataBuf, PatchInfo_BeTransferedNum, size);
#endif
	uart2_WriteStr((char *)DataBuf, size);
	PatchInfo_BeTransferedNum +=size;
	PatchInfo_LeftNum = PatchInfo_TotalNum - PatchInfo_BeTransferedNum;
}

void uart2_ReplaceFunc(void)
{//0x80000078
	uint8_t rep_cmd[9] = {0x04, 0x80, 0x00, 0x00, 0x78, 0x00,0x00,0x00,0x04};
	uart2_WriteStr((char *)rep_cmd, sizeof(rep_cmd));
}

uint8_t uart2_ReplaceFunc_ACK(char data)
{
	static uint8_t ack[9];
	ack[AckNum++] = data;
	if(AckNum == 9){
		return TRUE;
	}
	return READING;
}

void uart2_Replace_WriteData(void)
{
	uint8_t data[4] = {0xe5, 0x40, 0x00 , 0x80}; //\xe5\x40\x00\x80
	uart2_WriteStr((char *)data, sizeof(data));
}

void uart2_Reconn_CMD(void)
{
	uint8_t cmd[9] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
	//uint8_t cmd[] = {'b'+100, 'l'+100,'u'+100,'e'+100,'s'+100,'p'+100,'e'+100, 'a'+100, 'k'+100};
	uart2_WriteStr((char *)cmd, sizeof(cmd));
}

void uart2_readmem(void)
{
	uint8_t cmd[9] = {0x03,0x80,0x00,0x00,0x78,0x00,0x00,0x00,0x04 };
	uart2_WriteStr((char *)cmd, sizeof(cmd));
}

void uart2_readDevInfo(void)
{
	uint8_t cmd[9] = {0x03,0x80,0x00,0x00,0xb0,0x00,0x00,0x00,0x14 };
	uart2_WriteStr((char *)cmd, sizeof(cmd));
}
	
void uart2_readPatchHead(void)
{
	uint8_t cmd[9] = {0x03,0x80,0x00,0x40,0xe4,0x00,0x00,0x00,0x08 };
	uart2_WriteStr((char *)cmd, sizeof(cmd));
}
void uart2_ErasureFlash_req(void)
{
	uint8_t cmd[9] = {0x08, 0x00, 0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00};
	uart2_WriteStr((char *)cmd, sizeof(cmd));
}

uint8_t uart2_Redefine_Ack(char data)
{
	static uint8_t ack[18];	
	ack[AckNum++] = data;
	if(AckNum == 18){
		return TRUE;
	}
	return READING;
}

void uart2_ChangeBaudrate_req(uint32_t baudrate)
{
	uint8_t cmd[9];
	cmd[0] = 0x0b;
	IntToStr(&cmd[1], 0x01);
	IntToStr(&cmd[5], baudrate);
	uart2_WriteStr((char *)&cmd[0], sizeof(cmd));
}
	
uint8_t uart2_ChangeBaudRate_ACK(char data, uint32_t baudrate)
{
	static uint8_t ack[9];
	uint8_t STD_ack[9] ;
	ack[AckNum++] = data;
	if(AckNum == 9){
		STD_ack[0] = 0x0b;
		IntToStr(&STD_ack[1], 0x01);
		IntToStr(&STD_ack[5], baudrate);
		
		if( memcmp(ack, STD_ack, 9) == 0){
			return TRUE;
		}
		return FALSE;
	}
	return READING;
}

uint8_t uart2_ChangeBaudRateSuccess_ACK(char data)
{
	static uint8_t ack[9];
	uint8_t STD_ack[9];
	ack[AckNum++] = data;
	if(AckNum == 9){
		STD_ack[0] = 0xa0;
		IntToStr(&STD_ack[1], 0x55555555);
		IntToStr(&STD_ack[5], 0x99999999);
		
		if(memcmp(ack, STD_ack, sizeof(ack)) == 0){
			return TRUE;
		}else{
			return FALSE;
		}
	}
	return READING;
}

uint8_t uart2_Erasesure_ACK(char data)
{
	static uint8_t ack[9];
	uint8_t STD_ack[9];
	
	ack[AckNum++] = data;
	if(AckNum == 9){
		STD_ack[0] = 0x80;
		IntToStr(&STD_ack[1], 0x00);
		IntToStr(&STD_ack[5], 0x00);
		if( memcmp(ack, STD_ack, sizeof(ack)) == 0){
			return TRUE;
		}else{
			return FALSE;
		}
	}
	return READING;
}

void uart2_WriteParam_req(void)
{
	uint8_t cmd[9];
	uint32_t size;
	uint32_t addr;
	if(ParamInfo_LeftNum < OneTime_Num){
		size = ParamInfo_LeftNum;
	}else{
		size = OneTime_Num;
	}
	addr = ParamInfo_BaseAddr + ParamInfo_BeTransferedNum;
	
	cmd[0] = 0x06;
	IntToStr(&cmd[1], addr);
	IntToStr(&cmd[5], size);
	uart2_WriteStr((char *)cmd, sizeof(cmd));
}

uint8_t uart2_ParamWrite_ACK(char data)
{
	static uint8_t ack[9];
	uint8_t STD_ack[9];
	uint32_t addr;
	uint32_t size;
	
	ack[AckNum++] = data;
	if( AckNum == 9 ){
		addr = ParamInfo_BaseAddr + ParamInfo_BeTransferedNum;
		if(ParamInfo_LeftNum > OneTime_Num){
			size = OneTime_Num;
		}else{
			size = ParamInfo_LeftNum;
		}
		
		STD_ack[0] = 0x60;
		IntToStr(&STD_ack[1], addr);
		IntToStr(&STD_ack[5], size);
		if( memcmp(ack, STD_ack, sizeof(ack)) == 0){
			return TRUE;
		}else{
			return FALSE;
		}
	}
	return READING;
}
	
void uart2_WriteParamFile(void)
{
	uint32_t addr;
	uint32_t size;
	uint8_t cmd[9];
	
	//for test
	uint32_t temp;
	
	addr = ParamInfo_BaseAddr + ParamInfo_BeTransferedNum;
	if(ParamInfo_LeftNum > OneTime_Num){
		size = OneTime_Num;
	}else{
		size = ParamInfo_LeftNum;
	}
	FSMC_SRAM_ReadBuffer(DataBuf, addr ,size);
	uart2_WriteStr((char *)DataBuf, size);

}

// the length of str is four(max)
UL_int StrToInt(uint8_t *str)
{
	UL_int temp = 0;
	temp |= *(str);
	temp |= *(str + 1) << 8;
	temp |= *(str + 2) << 16;
	temp |= *(str + 3) << 24;
	return temp;
}

void ChangeBlueAddr(uint32_t BaseAddr, uint32_t AddrOffset, uint32_t num)
{
	uint32_t offset_pointer = BaseAddr + AddrOffset;
	uint8_t  offset_num[2];
	uint32_t offset_value  = 0;
	
	uint8_t current_addr[6];
	uint32_t BlueAddr_valueH;
	uint32_t BlueAddr_valueL;
	
	FSMC_SRAM_ReadBuffer(offset_num, offset_pointer, sizeof(offset_num));
	offset_value |= offset_num[0] ;
	offset_value |= offset_num[1] <<8 ;

	FSMC_SRAM_ReadBuffer(current_addr, BaseAddr + offset_value , sizeof(current_addr));
	BlueAddr_valueL = StrToInt(&current_addr[0]);
	BlueAddr_valueH = StrToInt(&current_addr[4]);
	BlueAddr_valueL +=1;
	if(BlueAddr_valueL >(uint32_t)(0xfffffff) ){
		BlueAddr_valueH+=1;
		BlueAddr_valueL = 0;
	}
	current_addr[0] = BlueAddr_valueL >> 0;
	current_addr[1] = BlueAddr_valueL >> 8;
	current_addr[2] = BlueAddr_valueL >> 16;
	current_addr[3] = BlueAddr_valueL >> 24;
	current_addr[4] = BlueAddr_valueH >> 0;
	current_addr[5] = BlueAddr_valueH >> 8;
	
	FSMC_SRAM_WriteBuffer(current_addr,  BaseAddr + offset_value, sizeof(current_addr));
}

void Update_ParamInfo(void)
{
	uint32_t size;
	if(ParamInfo_LeftNum < OneTime_Num)
		size = ParamInfo_LeftNum;
	else
		size = OneTime_Num;
	
	ParamInfo_BeTransferedNum +=size;
	ParamInfo_LeftNum = ParamInfo_TotalNum - ParamInfo_BeTransferedNum;
}

uint8_t uart2_ParamWriteOver_ACK(char data)
{
	static uint8_t ack[9];
	uint8_t STD_ack[9];
	uint32_t addr;
	uint32_t size;
	
	ack[AckNum++] = data;
	if(AckNum == 9){
		addr = ParamInfo_BaseAddr + ParamInfo_BeTransferedNum;
		if(ParamInfo_LeftNum < OneTime_Num)
			size = ParamInfo_LeftNum;
		else
			size = OneTime_Num;
		
		STD_ack[0] = 0x20;
		IntToStr(&STD_ack[1], addr);
		IntToStr (&STD_ack[5], size);
		if( memcmp(ack, STD_ack, sizeof(ack)) == 0){
				
			return TRUE;
		}else{
			return FALSE;
		}
	}
	return READING;
}

void uart2_ReadParamFile_req(void)
{
	uint32_t addr;
	uint32_t size;
	uint8_t cmd[9];
	
	addr = ParamInfo_BaseAddr + ReadNum;
	size  =GetReadSize();
	
	cmd[0] = 0x05;
	IntToStr(&cmd[1], addr);
	IntToStr(&cmd[5], size);
	uart2_WriteStr((char *) cmd, sizeof(cmd));
}

uint8_t uart2_ReadParamFile_ACK(char data)
{
	uint32_t addr;
	ReadDataBuf[AckNum ++] = data;

	if(AckNum >= GetReadSize() + 9){
		addr = ParamInfo_BaseAddr + ReadNum;
		
			FSMC_SRAM_ReadBuffer(DataBuf, addr,  GetReadSize());
			if(memcmp(DataBuf, &ReadDataBuf[9], GetReadSize()) == 0){
				return  TRUE;
			}else{
				return FALSE;
			}
	}
	return READING;
}

extern u8 StartFlag;
uint8_t Fail_Flag = 0;
uint8_t Success_Flag = 0;

uint8_t uart2_DownloadEntry(char data)
{
	uint8_t ste_flag = 0;
	
	switch(UART2Doanload_State){
		case NoCore:
			break;
		
		case Connect_ACK:
			break;
		
		case ReadFlashData_ACK:
			ste_flag = Read_FlashRead_ACK(data);
			if(ste_flag == FALSE){
				goto uart2_DownloadFail;
			}else if(ste_flag == READING){
				UART2Doanload_State = ReadFlashData_ACK;
			}else if(ste_flag == TRUE){
				UART2Doanload_State = PatchWrite_ACK;
				delay_ms(10);
				uart2_PatchWrite_REQ();
				AckNum_Clear();
			}
			break;
			
		case PatchWrite_ACK:
			ste_flag = Read_PatchWrite_ACK(data);
			if(ste_flag == READING){
				UART2Doanload_State = PatchWrite_ACK;
			}else if(ste_flag == FALSE){
				goto uart2_DownloadFail;
			}else if(ste_flag == TRUE){
				AckNum_Clear();
				// transfer patch data over ?
				if(PatchInfo_BeTransferedNum + OneTime_Num >= PatchInfo_TotalNum){
					uart2_PatchDataWrite(); // the last one time to write patch data
					
					delay_ms(100);
					//replace new func
					UART2Doanload_State = RepaceFunc_Ack;
					AckNum_Clear();
					uart2_ReplaceFunc();
					
					break;
				}
				uart2_PatchDataWrite();
				uart2_PatchWrite_REQ();
			}
		break;
		
		case RepaceFunc_Ack:
			ste_flag = uart2_ReplaceFunc_ACK(data);
			if(ste_flag == READING){
				UART2Doanload_State = RepaceFunc_Ack;
			}else{
				uart2_Replace_WriteData();
				delay_ms(10);
				UART2Doanload_State = Redefine_Ack;
				AckNum_Clear();
				uart2_Reconn_CMD();
			}
		break;
		
		case Redefine_Ack:
			ste_flag = uart2_Redefine_Ack(data);
			if(ste_flag == READING){
				UART2Doanload_State = Redefine_Ack;
			}else if(ste_flag == TRUE){
				AckNum_Clear();
#if ChangeBaudRate
				uart2_ChangeBaudrate_req(921600);
				UART2Doanload_State = ChangeBaudRate_ACK;
#else
				UART2Doanload_State = Erasesure_ACK;
				uart2_ErasureFlash_req();
#endif
			}
			break;
		
		case ChangeBaudRate_ACK:
			ste_flag = uart2_ChangeBaudRate_ACK(data, 921600);
			if(ste_flag == READING){
				ste_flag = ChangeBaudRate_ACK;
			}else if(ste_flag == FALSE){
				goto uart2_DownloadFail;
				
				uart2_ChangeBaudRate(115200);
				
				UART2Doanload_State = Erasesure_ACK;
				AckNum_Clear();
				uart2_ErasureFlash_req();
				
			}else if(ste_flag == TRUE){
				uart2_ChangeBaudRate(921600);
				AckNum_Clear();
				UART2Doanload_State = ChangeBaudRateSuccess_ACK;
			}
			break;
		
		case ChangeBaudRateSuccess_ACK:
			ste_flag = uart2_ChangeBaudRateSuccess_ACK(data);
			if(ste_flag == READING){
				UART2Doanload_State = ChangeBaudRateSuccess_ACK;
			}else if(ste_flag == FALSE){//修改波特率失败
				uart2_ChangeBaudRate(115200);
				AckNum_Clear();
				uart2_ErasureFlash_req();
				UART2Doanload_State = Erasesure_ACK;
			}else if(ste_flag == TRUE){ //修改波特率成功
				AckNum_Clear();
				uart2_ErasureFlash_req();
				UART2Doanload_State = Erasesure_ACK;
			}
			break;
			
			
		case Erasesure_ACK:
			ste_flag = uart2_Erasesure_ACK(data);
			if(ste_flag == READING){
				UART2Doanload_State = Erasesure_ACK;
			}else if(ste_flag == FALSE){
				goto uart2_DownloadFail;
			}else if(ste_flag == TRUE){
				AckNum_Clear();
				uart2_WriteParam_req();
				UART2Doanload_State = ParamWrite_ACK;
			}
			break;
		
		case ParamWrite_ACK:
			ste_flag = uart2_ParamWrite_ACK(data);
			if(ste_flag == READING){
				UART2Doanload_State = ParamWrite_ACK;
			}else if(ste_flag == FALSE){
				goto uart2_DownloadFail;
			}else if(ste_flag == TRUE){
				AckNum_Clear();
				uart2_WriteParamFile();
				UART2Doanload_State = ParamWriteOver_ACK;
			}
			break;
		
		case ParamWriteOver_ACK:
			ste_flag = uart2_ParamWriteOver_ACK(data);
			if(ste_flag == READING){
				UART2Doanload_State = ParamWriteOver_ACK;
			}else if(ste_flag == FALSE){
				goto uart2_DownloadFail;
			}else if(ste_flag == TRUE){
				AckNum_Clear();
				Update_ParamInfo();
				if(ParamInfo_BeTransferedNum >= ParamInfo_TotalNum){
					//goto uart2_DownloadSuccess;
					// for check the paramfile is right to write
					UART2Doanload_State = ReadParamFile_ACK;
					ReadInfo_Init();
					uart2_ReadParamFile_req();
					break;
					
				}	
				UART2Doanload_State = ParamWrite_ACK;
				uart2_WriteParam_req();
			}
			break;
			
		case ReadParamFile_ACK:
			ste_flag = uart2_ReadParamFile_ACK(data);
			if(ste_flag == READING){
				UART2Doanload_State = ReadParamFile_ACK;
			}else if(ste_flag == FALSE){
				goto uart2_DownloadFail;
			}else if(ste_flag == TRUE){
				AckNum_Clear();
				ReadInfo_Update();
				
				if(ReadNum >= ReadTotal){
					goto uart2_DownloadSuccess;
				}
				uart2_ReadParamFile_req();
				UART2Doanload_State = ReadParamFile_ACK;
			}
			break;
			
		default:
			break;
			
	}
	
return 0;
	
uart2_DownloadSuccess:
			DisEnableBusy();
			StartFlag = 0;
			OK_Single();
			Success_Flag = 1;
			DownlaodOK_Num++;
			ChangeBlueAddr(ParamInfo_BaseAddr, ParamInfo_BlueAddr_offset, 1);
#if LED_ShowState
			LED1 = LED_OFF;		
			LED2 = LED_OFF;
#endif
			delay_ms(100);
			EnableBusy();
			//blueaddr ++
			return 0;
		
uart2_DownloadFail:
			DisEnableBusy();
			StartFlag = 0;	
			NG_Single();
#if LED_ShowState
			LED2 = LED_OFF;
#endif
			Fail_Flag = 1;
			delay_ms(100);
			EnableBusy();
		return 0;
		
}

void BusyPin_Init(void)
{
// PE6
	 //PE4
	GPIOE->CRL &= ~(0x03 << 24);
	GPIOE->CRL |=  0x01 <<24;
	GPIOE->CRL &= ~(0x03 << 26);
	GPIOE->CRL |=  0x00 <<26;
	
	GPIOE->ODR &=  ~(0x01 << 6);  //初始化后输出低电平
}

void EnableBusy(void)
{
GPIOE->ODR |= (0x01 << 6); 
}

void DisEnableBusy(void)
{
GPIOE->ODR &=  ~(0x01 << 6); 
}

void OKPin_Init(void)
{
 //PE4
	GPIOE->CRL &= ~(0x03 << 16);
	GPIOE->CRL |=  0x01 <<16;
	GPIOE->CRL &= ~(0x03 << 18);
	GPIOE->CRL |=  0x00 <<18;
	
	GPIOE->ODR &=  ~(0x01 << 4);
}

void NGPin_Init(void)
{
 //PE5 
	GPIOE->CRL &= ~(0x03 << 20);
	GPIOE->CRL |=  0x01 <<20;
	GPIOE->CRL &= ~(0x03 << 22);
	GPIOE->CRL |=  0x00 <<22;
	GPIOE->ODR &=  ~(0x01 << 5);
}

void NG_Single(void)
{
	GPIOE->ODR |= (0x01 << 5);
	delay_ms(1000);
	GPIOE->ODR &=  ~(0x01 << 5);
}

void OK_Single(void)
{
	GPIOE->ODR |= (0x01 << 4);
	delay_ms(1000);
	GPIOE->ODR &=  ~(0x01 << 4);
}



#include <string.h>
#include "led.h"
#include "ff.h"
#include "burn_flash.h"
#include "flash.h" 
#include "spi.h"
#include "delay.h"	   
#include "usart.h"	
#include "malloc.h"
#include "lower.h"
#include "uart2_download.h"

#define FLASH_SECTOR_SIZE	4096
u16 BURN_FLASH_TYPE=W25Q64;//默认就是25Q64
typedef struct{
	u16 flash_ID;
	u32 flash_capacity;
	u32 flash_sector_size;
	u32 flash_sector_num;
	u32 flash_page_size;
}flash_info_s;
//4Kbytes为一个Sector
//16个扇区为1个Block
//W25Q64
//容量为8M字节,共有128个Block,2048个Sector 
													 
//初始化SPI FLASH的IO口
void BURN_Flash_Init(void)
{
	RCC->APB2ENR|=1<<2;  	//PORTA时钟使能 
	GPIOA->CRL&=0XFFF0FFFF; 
	GPIOA->CRL|=0X00030000;	//PA4 推挽 	    
	GPIOA->ODR|=1<<4;    	//PA4上拉
	SPI1_Init();		   	//初始化SPI
	SPI1_SetSpeed(SPI_SPEED_64);//设置为18M时钟,高速模式
	BURN_FLASH_TYPE=BURN_Flash_ReadID();//读取FLASH ID.
}  

//读取BURN_Flash的状态寄存器
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:默认0,状态寄存器保护位,配合WP使用
//TB,BP2,BP1,BP0:FLASH区域写保护设置
//WEL:写使能锁定
//BUSY:忙标记位(1,忙;0,空闲)
//默认:0x00
u8 BURN_Flash_ReadSR(u8 *byte)   
{  
	do{ 
		if(byte==NULL)
			break;
		BURN_FLASH_CS=0;                            //使能器件   
		if(SPI1_ReadWriteByte(W25X_ReadStatusReg,NULL)==FALSE)//发送读取状态寄存器命令  	
			break;
		if(SPI1_ReadWriteByte(0Xff,byte)==FALSE)//读取一个字节  
			break;
		BURN_FLASH_CS=1;                            //取消片选     
		return TRUE;
	}while(0);
	BURN_FLASH_CS=1;
	return FALSE;
} 
//写BURN_Flash状态寄存器
//只有SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)可以写!!!
void BURN_Flash_Write_SR(u8 sr)   
{   
	BURN_FLASH_CS=0;                            //使能器件   
	SPI1_ReadWriteByte(W25X_WriteStatusReg,NULL);   //发送写取状态寄存器命令    
	SPI1_ReadWriteByte(sr,NULL);               //写入一个字节  
	BURN_FLASH_CS=1;                            //取消片选     	      
}   
//擦除整个芯片		  
//等待时间超长...
void BURN_Flash_Erase_Chip(void)   
{                                   
    BURN_Flash_Write_Enable();                  //SET WEL 
    BURN_Flash_Wait_Busy();   
  	BURN_FLASH_CS=0;                            //使能器件   
    SPI1_ReadWriteByte(W25X_ChipErase,NULL);        //发送片擦除命令  
	BURN_FLASH_CS=1;                            //取消片选     	      
	BURN_Flash_Wait_Busy();   				   //等待芯片擦除结束
}   
//擦除一个扇区
//Dst_Addr:扇区地址 根据实际容量设置
//擦除一个山区的最少时间:150ms
void BURN_Flash_Erase_Sector(u32 Dst_Addr)   
{  
	//监视falsh擦除情况,测试用   
 	//printf("fe:%x\r\n",Dst_Addr);	  
 	Dst_Addr*=4096;
    BURN_Flash_Write_Enable();                  //SET WEL 	 
    BURN_Flash_Wait_Busy();   
  	BURN_FLASH_CS=0;                            //使能器件   
    SPI1_ReadWriteByte(W25X_SectorErase,NULL);      //发送扇区擦除指令 
    SPI1_ReadWriteByte((u8)((Dst_Addr)>>16),NULL);  //发送24bit地址    
    SPI1_ReadWriteByte((u8)((Dst_Addr)>>8),NULL);   
    SPI1_ReadWriteByte((u8)Dst_Addr,NULL);  
	BURN_FLASH_CS=1;                            //取消片选     	      
    BURN_Flash_Wait_Busy();   				   //等待擦除完成
}  
//等待空闲
void BURN_Flash_Wait_Busy(void)   
{   
	u8 sr=0x01;
	u8 byte;
	BURN_FLASH_CS=0;                            //使能器件   
	SPI1_ReadWriteByte(W25X_ReadStatusReg,NULL);//发送读取状态寄存器命令  	
	do{
		if(SPI1_ReadWriteByte(0Xff,&byte)==TRUE)//读取一个字节  
			sr=byte; 
	}while((sr&0x01)==0x01);   // 等待BUSY位清空
	BURN_FLASH_CS=1;                            //取消片选    
}  
//进入掉电模式
void BURN_Flash_PowerDown(void)   
{ 
  	BURN_FLASH_CS=0;                            //使能器件   
    SPI1_ReadWriteByte(W25X_PowerDown,NULL);        //发送掉电命令  
	BURN_FLASH_CS=1;                            //取消片选     	      
    delay_us(3);                               //等待TPD  
}   
//唤醒
void BURN_Flash_WAKEUP(void)   
{  
  	BURN_FLASH_CS=0;                            //使能器件   
    SPI1_ReadWriteByte(W25X_ReleasePowerDown,NULL);   //  send W25X_PowerDown command 0xAB    
	BURN_FLASH_CS=1;                            //取消片选     	      
    delay_us(3);                               //等待TRES1
}  
//BURN_Flash写使能	
//将WEL置位   
void BURN_Flash_Write_Enable(void)   
{
	BURN_FLASH_CS=0;                            //使能器件   
    SPI1_ReadWriteByte(W25X_WriteEnable,NULL);      //发送写使能  
	BURN_FLASH_CS=1;                            //取消片选     	      
} 
//BURN_Flash写禁止	
//将WEL清零  
void BURN_Flash_Write_Disable(void)   
{  
	BURN_FLASH_CS=0;                            //使能器件   
    SPI1_ReadWriteByte(W25X_WriteDisable,NULL);     //发送写禁止指令    
	BURN_FLASH_CS=1;                            //取消片选     	      
} 		
//读取芯片ID
//返回值如下:				   
//0XEF13,表示芯片型号为W25Q80  
//0XEF14,表示芯片型号为W25Q16    
//0XEF15,表示芯片型号为W25Q32  
//0XEF16,表示芯片型号为W25Q64   	  
u16 BURN_Flash_ReadID(void)
{
	u8 datal,datah;
	u16 Temp = 0;	  
	BURN_FLASH_CS=0;
	SPI1_ReadWriteByte(0x90,NULL);//发送读取ID命令	    
	SPI1_ReadWriteByte(0x00,NULL); 	    
	SPI1_ReadWriteByte(0x00,NULL); 	    
	SPI1_ReadWriteByte(0x00,NULL); 	 	
	SPI1_ReadWriteByte(0xFF,&datah);	
	SPI1_ReadWriteByte(0xFF,&datal);
	Temp|=datah<<8;  
	Temp|=datal;	 
	BURN_FLASH_CS=1;				    
	return Temp;
}   		 \




//读取SPI FLASH  
//在指定地址开始读取指定长度的数据
//pBuffer:数据存储区
//ReadAddr:开始读取的地址(24bit)
//NumByteToRead:要读取的字节数(最大65535)
void BURN_Flash_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead)   
{ 
 	u16 i;   										    
	BURN_FLASH_CS=0;                            //使能器件   
    SPI1_ReadWriteByte(W25X_FastReadData,NULL);         //发送读取命令   
    SPI1_ReadWriteByte((u8)((ReadAddr)>>16),NULL);  //发送24bit地址    
    SPI1_ReadWriteByte((u8)((ReadAddr)>>8),NULL);   
    SPI1_ReadWriteByte((u8)ReadAddr,NULL); 
		SPI1_ReadWriteByte(W25X_Dummuy, NULL);
    for(i=0;i<NumByteToRead;i++)
	{ 
        SPI1_ReadWriteByte(0XFF,&pBuffer[i]);   //循环读数  
    }
	BURN_FLASH_CS=1;  				    	      
}  
//SPI在一页(0~65535)内写入少于256个字节的数据
//在指定地址开始写入最大256字节的数据
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大256),该数不应该超过该页的剩余字节数!!!	 
void BURN_Flash_Write_Page(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)
{
 	u16 i;  
    BURN_Flash_Write_Enable();                  //SET WEL 
	BURN_FLASH_CS=0;                            //使能器件   
    SPI1_ReadWriteByte(W25X_PageProgram,NULL);      //发送写页命令   
    SPI1_ReadWriteByte((u8)((WriteAddr)>>16),NULL); //发送24bit地址    
    SPI1_ReadWriteByte((u8)((WriteAddr)>>8),NULL);   
    SPI1_ReadWriteByte((u8)WriteAddr,NULL);   
    for(i=0;i<NumByteToWrite;i++)SPI1_ReadWriteByte(pBuffer[i],NULL);//循环写数  
	BURN_FLASH_CS=1;                            //取消片选 
	BURN_Flash_Wait_Busy();					   //等待写入结束
} 

//无检验写SPI FLASH 
//必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
//具有自动换页功能 
//在指定地址开始写入指定长度的数据,但是要确保地址不越界!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大65535)
//CHECK OK
void BURN_Flash_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 			 		 
	u16 pageremain;	   
	pageremain=256-WriteAddr%256; //单页剩余的字节数		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//不大于256个字节
	while(1)
	{	   
		BURN_Flash_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//写入结束了
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //减去已经写入了的字节数
			if(NumByteToWrite>256)pageremain=256; //一次可以写入256个字节
			else pageremain=NumByteToWrite; 	  //不够256个字节了
		}
	};	    
} 

bool BURN_Flash_Write_Sectors(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)
{
	u32 secpos;
	u32 secnum;
	do{
		int i;
		if(WriteAddr%4096!=0)
			break;
		secpos=WriteAddr/4096;
		secnum=NumByteToWrite/4096;
		if(NumByteToWrite%4096)
			secnum++;
		for(i=0;i<secnum;i++)
		{
			BURN_Flash_Erase_Sector(secpos+i);//擦除这个扇区
		}
		BURN_Flash_Write_NoCheck(pBuffer,secpos*4096,NumByteToWrite);//写入整个扇区 
		return TRUE;
	}while(0);
	return FALSE;
}
//写SPI FLASH  
//在指定地址开始写入指定长度的数据
//该函数带擦除操作!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)						
//NumByteToWrite:要写入的字节数(最大65535)   
u8 BURN_Flash_BUFFER[4096];		 
void BURN_Flash_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 
	u32 secpos;
	u16 secoff;
	u16 secremain;	   
 	u16 i;    
	u8 * BURN_Flash_BUF;	  
   	BURN_Flash_BUF=BURN_Flash_BUFFER;	     
 	secpos=WriteAddr/4096;//扇区地址  
	secoff=WriteAddr%4096;//在扇区内的偏移
	secremain=4096-secoff;//扇区剩余空间大小   
 	//printf("ad:%X,nb:%X\r\n",WriteAddr,NumByteToWrite);//测试用
 	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//不大于4096个字节
	while(1) 
	{	
		BURN_Flash_Read(BURN_Flash_BUF,secpos*4096,4096);//读出整个扇区的内容
		for(i=0;i<secremain;i++)//校验数据
		{
			if(BURN_Flash_BUF[secoff+i]!=0XFF)break;//需要擦除  	  
		}
		if(i<secremain)//需要擦除
		{
			BURN_Flash_Erase_Sector(secpos);//擦除这个扇区
			for(i=0;i<secremain;i++)	   //复制
			{
				BURN_Flash_BUF[i+secoff]=pBuffer[i];	  
			}
			BURN_Flash_Write_NoCheck(BURN_Flash_BUF,secpos*4096,4096);//写入整个扇区  

		}else BURN_Flash_Write_NoCheck(pBuffer,WriteAddr,secremain);//写已经擦除了的,直接写入扇区剩余区间. 				   
		if(NumByteToWrite==secremain)break;//写入结束了
		else//写入未结束
		{
			secpos++;//扇区地址增1
			secoff=0;//偏移位置为0 	 

		   	pBuffer+=secremain;  //指针偏移
			WriteAddr+=secremain;//写地址偏移	   
		   	NumByteToWrite-=secremain;				//字节数递减
			if(NumByteToWrite>4096)secremain=4096;	//下一个扇区还是写不完
			else secremain=NumByteToWrite;			//下一个扇区可以写完了
		}	 
	};	 
}
 

extern void HC244_disable(void);	//...
extern FIL *f_bin;
extern const u32 bd_addr_offset[MAX_DEVICE_TYPE][2];
extern u8 bd_addr[6];
u8 *burn_buffer=NULL;
u32 burn_buffer_size=0;
#define SINGLE_BUFFER_SIZE      512
bool burnFLASH(void)
{
	static UINT read_count = 0;
	UINT tmp_addr;
	for(;;){
		if(burn_buffer == NULL) {
			burn_buffer = mymalloc(SRAMIN, SINGLE_BUFFER_SIZE);
			if(burn_buffer==NULL)
				break;
		}
		if(burn_buffer_size==0)
		{
			if(f_lseek(f_bin, 0)!=FR_OK)
				break;
			if(f_read(f_bin, burn_buffer, SINGLE_BUFFER_SIZE, &read_count)!=FR_OK)
				break;
		}
		if((burn_buffer_size <= bd_addr_offset[device_type][0])
			&&((burn_buffer_size+read_count) > bd_addr_offset[device_type][0])) {
			if((burn_buffer_size+read_count) > bd_addr_offset[device_type][1]) {
				memcpy(burn_buffer+(bd_addr_offset[device_type][0]-burn_buffer_size), bd_addr, 6);
			}
			else {
				memcpy(burn_buffer+(bd_addr_offset[device_type][0]-burn_buffer_size), bd_addr, 5+(burn_buffer_size+read_count)-bd_addr_offset[device_type][1]);
			}
		}
		else if(burn_buffer_size > bd_addr_offset[device_type][0]&&burn_buffer_size <= bd_addr_offset[device_type][1]) {
			memcpy(burn_buffer, bd_addr+(burn_buffer_size+5-bd_addr_offset[device_type][1]), bd_addr_offset[device_type][1]-burn_buffer_size+1);
		}

		BURN_Flash_Write(burn_buffer,burn_buffer_size,read_count);
		burn_buffer_size += read_count;
		if(burn_buffer_size >= f_bin->fsize) {
			burn_buffer_size = 0;
			tmp_addr = bd_addr[0] | (bd_addr[1]<<8) | (bd_addr[2]<<16);
			tmp_addr += 1;
			bd_addr[0] = tmp_addr & 0xFF;
			bd_addr[1] = (tmp_addr>>8) & 0xFF;
			bd_addr[2] = (tmp_addr>>16) & 0xFF;
					LED1 = LED_ON;	//...
					LED2 = LED_OFF;	//...
					HC244_disable();	//...
			return TRUE;;
		}
		else {
			if(f_read(f_bin, burn_buffer, SINGLE_BUFFER_SIZE, &read_count)!=FR_OK)
				break;
		}
	}
	return FALSE;
}

extern  uint32_t  ParamInfo_TotalNum;
extern uint32_t  ParamInfo_BeTransferedNum;
extern uint32_t  ParamInfo_LeftNum;	
extern uint32_t  ParamInfo_BaseAddr;
extern uint32_t  ParamInfo_BlueAddr_offset;
#define SPISector_NUM (4096)
static uint8_t SPI_DataBuf[SPISector_NUM];

bool burnFlashbysector(void)
{
#if 0
	static UINT read_count = 0;
	UINT tmp_addr;
	for(;;){
		if(burn_buffer == NULL) {
			burn_buffer = mymalloc(SRAMIN, FLASH_SECTOR_SIZE);
			if(burn_buffer==NULL)
				break;
		}
		if(burn_buffer_size==0)
		{
#if 0
			if(f_lseek(f_bin, 0)!=FR_OK)
				break;
			if(f_read(f_bin, burn_buffer, FLASH_SECTOR_SIZE, &read_count)!=FR_OK)
				break;
#else
			FSMC_SRAM_ReadBuffer(burn_buffer, ParamInfo_BaseAddr, FLASH_SECTOR_SIZE);
#endif
		}
		if((burn_buffer_size <= bd_addr_offset[device_type][0])
			&&((burn_buffer_size+read_count) > bd_addr_offset[device_type][0])) {
			if((burn_buffer_size+read_count) > bd_addr_offset[device_type][1]) {
				memcpy(burn_buffer+(bd_addr_offset[device_type][0]-burn_buffer_size), bd_addr, 6);
			}
			else {
				memcpy(burn_buffer+(bd_addr_offset[device_type][0]-burn_buffer_size), bd_addr, 5+(burn_buffer_size+read_count)-bd_addr_offset[device_type][1]);
			}
		}
		else if(burn_buffer_size > bd_addr_offset[device_type][0]&&burn_buffer_size <= bd_addr_offset[device_type][1]) {
			memcpy(burn_buffer, bd_addr+(burn_buffer_size+5-bd_addr_offset[device_type][1]), bd_addr_offset[device_type][1]-burn_buffer_size+1);
		}

		BURN_Flash_Write_Sectors(burn_buffer,burn_buffer_size,read_count);	
		burn_buffer_size += read_count;
		if(burn_buffer_size >= f_bin->fsize) {
			burn_buffer_size = 0;
			tmp_addr = bd_addr[0] | (bd_addr[1]<<8) | (bd_addr[2]<<16);
			tmp_addr += 1;
			bd_addr[0] = tmp_addr & 0xFF;
			bd_addr[1] = (tmp_addr>>8) & 0xFF;
			bd_addr[2] = (tmp_addr>>16) & 0xFF;
					LED1 = LED_ON;	//...
					LED2 = LED_OFF;	//...
					HC244_disable();	//...
			return TRUE;
		}
		else {
#if 0
			if(f_read(f_bin, burn_buffer, FLASH_SECTOR_SIZE, &read_count)!=FR_OK)
				break;
#else
			
#endif

		}
	}
	return FALSE;
	
#else
	
uint32_t size;
	
ParamInfo_LeftNum = ParamInfo_TotalNum;
	
for(ParamInfo_BeTransferedNum = 0; ParamInfo_BeTransferedNum < ParamInfo_TotalNum; ){
	if(ParamInfo_LeftNum > SPISector_NUM)
		size = SPISector_NUM;
	else
		size = ParamInfo_LeftNum;
	
	FSMC_SRAM_ReadBuffer(SPI_DataBuf, ParamInfo_BaseAddr + ParamInfo_BeTransferedNum, size);
	BURN_Flash_Write_Sectors(SPI_DataBuf,ParamInfo_BeTransferedNum,size);	
	
	ParamInfo_BeTransferedNum += size;
	ParamInfo_LeftNum = ParamInfo_TotalNum - ParamInfo_BeTransferedNum;
}

return TRUE;
	
#endif
}


/*func: Enable_OutVcc
effect: enable the SPX3819, output the vcc
*/
void Enable_OutVcc(void)
{
//GPIOE_CRL -- mode  00  PE3 bit12-bit13
	GPIOE->CRL &= ~(0x03 << 12);
	GPIOE->CRL |=  0x01 <<12;
// CNF3  bit14-bit15   通用推挽输出 00
	GPIOE->CRL &= ~(0x03 << 14);
	GPIOE->CRL |=  0x00 <<14;
	// output 1
	GPIOE->ODR = 0x01<<3;
}

void DISEnable_OutVcc(void)
{
//GPIOE_CRL -- mode  00  PE3 bit12-bit13
	GPIOE->CRL &= ~(0x03 << 12);
	GPIOE->CRL |=  0x01 <<12;
// CNF3  bit14-bit15   通用推挽输出 00
	GPIOE->CRL &= ~(0x03 << 14);
	GPIOE->CRL |=  0x00 <<14;
	// output 1
	GPIOE->ODR = 0x00<<3;
}

/*func: Enable_CMOS
effect: enable the cmos ,output the Reset single
*/
void RESET_Single(void)
{
	power_control_pin_on();
	delay_ms(500);
	power_control_pin_off();
	delay_ms(25000);
	power_control_pin_on();
	delay_ms(500);
}


//拉低
void UP_Reset(void)
{
//GPIOE_CRL -- mode  00  PE2 bit12-bit13
	GPIOE->CRL &= ~(0x03 << 8);
	GPIOE->CRL |=  0x01 <<8;
// CNF3  bit14-bit15   通用推挽输出 00
	GPIOE->CRL &= ~(0x03 << 10);
	GPIOE->CRL |=  0x00 <<10;
	// output 1
	GPIOE->ODR = 0x01<<2;
}

// 不拉低
void LOWER_Reset(void)
{
//GPIOE_CRL -- mode  00  PE2 bit12-bit13
	GPIOE->CRL &= ~(0x03 << 8);
	GPIOE->CRL |=  0x01 <<8;
// CNF3  bit14-bit15   通用推挽输出 00
	GPIOE->CRL &= ~(0x03 << 10);
	GPIOE->CRL |=  0x00 <<10;
	// output 1
	GPIOE->ODR = 0x00<<2;
}

#define ReadNum (512)
uint8_t SPI_ReadData[ReadNum];
uint8_t SRAM_ReadData[ReadNum];

// SPI_DataBuf
bool CheckSPIDownload(u32 total)
{
	uint32_t read_addr;
	uint32_t size = 0;
	uint32_t left = total;
	uint32_t transfer_num = 0;
	
	for(transfer_num = 0; transfer_num < total; ){
		if(left > ReadNum){
			size = ReadNum;
		}	else{
			size = left;
		}
		
		BURN_Flash_Read(SPI_ReadData, transfer_num, size);
		FSMC_SRAM_ReadBuffer(SRAM_ReadData, ParamInfo_BaseAddr + transfer_num, size);
		if(memcmp(SPI_ReadData, SRAM_ReadData, size) != 0){
			return FALSE;
		}
		transfer_num += size;
		left = total - transfer_num;
	}
	
	return TRUE;
}
























#ifndef _BURN_FLASH_H
#define _BURN_FLASH_H	    
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//W25Q64 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/9
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//W25X系列/Q系列芯片列表	   
//W25Q80 ID  0XEF13
//W25Q16 ID  0XEF14
//W25Q32 ID  0XEF15
//W25Q32 ID  0XEF16	
#define W25Q80 	0XEF13 	
#define W25Q16 	0XEF14
#define W25Q32 	0XEF15
#define W25Q64 	0XEF16

extern u16 SPI_FLASH_TYPE;		//定义我们使用的flash芯片型号		   
#define	BURN_FLASH_CS PAout(4)  //选中FLASH	
				 
////////////////////////////////////////////////////////////////////////////
 
//指令表
#define W25X_WriteEnable		0x06 
#define W25X_WriteDisable		0x04 
#define W25X_ReadStatusReg		0x05 
#define W25X_WriteStatusReg		0x01 
#define W25X_ReadData			0x03 
#define W25X_FastReadData		0x0B 
#define W25X_FastReadDual		0x3B 
#define W25X_PageProgram		0x02 
#define W25X_BlockErase			0xD8 
#define W25X_SectorErase		0x20 
#define W25X_ChipErase			0xC7 
#define W25X_PowerDown			0xB9 
#define W25X_ReleasePowerDown	0xAB 
#define W25X_DeviceID			0xAB 
#define W25X_ManufactDeviceID	0x90 
#define W25X_JedecDeviceID		0x9F 
#define W25X_Dummuy 0x00

void BURN_Flash_Init(void);
u16  BURN_Flash_ReadID(void);  	    //读取FLASH ID
u8 BURN_Flash_ReadSR(u8 *byte);        //读取状态寄存器 
void BURN_Flash_Write_SR(u8 sr);  	//写状态寄存器
void BURN_Flash_Write_Enable(void);  //写使能 
void BURN_Flash_Write_Disable(void);	//写保护
void BURN_Flash_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);
void BURN_Flash_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead);   //读取flash
void BURN_Flash_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);//写入flash
void BURN_Flash_Erase_Chip(void);    	  //整片擦除
void BURN_Flash_Erase_Sector(u32 Dst_Addr);//扇区擦除
void BURN_Flash_Wait_Busy(void);           //等待空闲
void BURN_Flash_PowerDown(void);           //进入掉电模式
void BURN_Flash_WAKEUP(void);			  //唤醒

bool burnFLASH(void);
bool burnFlashbysector(void);

void Enable_OutVcc(void);
void DISEnable_OutVcc(void);

void RESET_Single(void);
void UP_Reset(void);
void LOWER_Reset(void);

bool CheckSPIDownload(u32 total);

#endif

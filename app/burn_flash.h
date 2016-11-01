#ifndef _BURN_FLASH_H
#define _BURN_FLASH_H	    
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEKս��STM32������
//W25Q64 ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/9
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//W25Xϵ��/Qϵ��оƬ�б�	   
//W25Q80 ID  0XEF13
//W25Q16 ID  0XEF14
//W25Q32 ID  0XEF15
//W25Q32 ID  0XEF16	
#define W25Q80 	0XEF13 	
#define W25Q16 	0XEF14
#define W25Q32 	0XEF15
#define W25Q64 	0XEF16

extern u16 SPI_FLASH_TYPE;		//��������ʹ�õ�flashоƬ�ͺ�		   
#define	BURN_FLASH_CS PAout(4)  //ѡ��FLASH	
				 
////////////////////////////////////////////////////////////////////////////
 
//ָ���
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

void BURN_Flash_Init(void);
u16  BURN_Flash_ReadID(void);  	    //��ȡFLASH ID
u8 BURN_Flash_ReadSR(u8 *byte);        //��ȡ״̬�Ĵ��� 
void BURN_Flash_Write_SR(u8 sr);  	//д״̬�Ĵ���
void BURN_Flash_Write_Enable(void);  //дʹ�� 
void BURN_Flash_Write_Disable(void);	//д����
void BURN_Flash_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);
void BURN_Flash_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead);   //��ȡflash
void BURN_Flash_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);//д��flash
void BURN_Flash_Erase_Chip(void);    	  //��Ƭ����
void BURN_Flash_Erase_Sector(u32 Dst_Addr);//��������
void BURN_Flash_Wait_Busy(void);           //�ȴ�����
void BURN_Flash_PowerDown(void);           //�������ģʽ
void BURN_Flash_WAKEUP(void);			  //����

bool burnFLASH(void);
bool burnFlashbysector(void);
#endif
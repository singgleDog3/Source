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
u16 BURN_FLASH_TYPE=W25Q64;//Ĭ�Ͼ���25Q64
typedef struct{
	u16 flash_ID;
	u32 flash_capacity;
	u32 flash_sector_size;
	u32 flash_sector_num;
	u32 flash_page_size;
}flash_info_s;
//4KbytesΪһ��Sector
//16������Ϊ1��Block
//W25Q64
//����Ϊ8M�ֽ�,����128��Block,2048��Sector 
													 
//��ʼ��SPI FLASH��IO��
void BURN_Flash_Init(void)
{
	RCC->APB2ENR|=1<<2;  	//PORTAʱ��ʹ�� 
	GPIOA->CRL&=0XFFF0FFFF; 
	GPIOA->CRL|=0X00030000;	//PA4 ���� 	    
	GPIOA->ODR|=1<<4;    	//PA4����
	SPI1_Init();		   	//��ʼ��SPI
	SPI1_SetSpeed(SPI_SPEED_64);//����Ϊ18Mʱ��,����ģʽ
	BURN_FLASH_TYPE=BURN_Flash_ReadID();//��ȡFLASH ID.
}  

//��ȡBURN_Flash��״̬�Ĵ���
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ��
//TB,BP2,BP1,BP0:FLASH����д��������
//WEL:дʹ������
//BUSY:æ���λ(1,æ;0,����)
//Ĭ��:0x00
u8 BURN_Flash_ReadSR(u8 *byte)   
{  
	do{ 
		if(byte==NULL)
			break;
		BURN_FLASH_CS=0;                            //ʹ������   
		if(SPI1_ReadWriteByte(W25X_ReadStatusReg,NULL)==FALSE)//���Ͷ�ȡ״̬�Ĵ�������  	
			break;
		if(SPI1_ReadWriteByte(0Xff,byte)==FALSE)//��ȡһ���ֽ�  
			break;
		BURN_FLASH_CS=1;                            //ȡ��Ƭѡ     
		return TRUE;
	}while(0);
	BURN_FLASH_CS=1;
	return FALSE;
} 
//дBURN_Flash״̬�Ĵ���
//ֻ��SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)����д!!!
void BURN_Flash_Write_SR(u8 sr)   
{   
	BURN_FLASH_CS=0;                            //ʹ������   
	SPI1_ReadWriteByte(W25X_WriteStatusReg,NULL);   //����дȡ״̬�Ĵ�������    
	SPI1_ReadWriteByte(sr,NULL);               //д��һ���ֽ�  
	BURN_FLASH_CS=1;                            //ȡ��Ƭѡ     	      
}   
//��������оƬ		  
//�ȴ�ʱ�䳬��...
void BURN_Flash_Erase_Chip(void)   
{                                   
    BURN_Flash_Write_Enable();                  //SET WEL 
    BURN_Flash_Wait_Busy();   
  	BURN_FLASH_CS=0;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_ChipErase,NULL);        //����Ƭ��������  
	BURN_FLASH_CS=1;                            //ȡ��Ƭѡ     	      
	BURN_Flash_Wait_Busy();   				   //�ȴ�оƬ��������
}   
//����һ������
//Dst_Addr:������ַ ����ʵ����������
//����һ��ɽ��������ʱ��:150ms
void BURN_Flash_Erase_Sector(u32 Dst_Addr)   
{  
	//����falsh�������,������   
 	//printf("fe:%x\r\n",Dst_Addr);	  
 	Dst_Addr*=4096;
    BURN_Flash_Write_Enable();                  //SET WEL 	 
    BURN_Flash_Wait_Busy();   
  	BURN_FLASH_CS=0;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_SectorErase,NULL);      //������������ָ�� 
    SPI1_ReadWriteByte((u8)((Dst_Addr)>>16),NULL);  //����24bit��ַ    
    SPI1_ReadWriteByte((u8)((Dst_Addr)>>8),NULL);   
    SPI1_ReadWriteByte((u8)Dst_Addr,NULL);  
	BURN_FLASH_CS=1;                            //ȡ��Ƭѡ     	      
    BURN_Flash_Wait_Busy();   				   //�ȴ��������
}  
//�ȴ�����
void BURN_Flash_Wait_Busy(void)   
{   
	u8 sr=0x01;
	u8 byte;
	BURN_FLASH_CS=0;                            //ʹ������   
	SPI1_ReadWriteByte(W25X_ReadStatusReg,NULL);//���Ͷ�ȡ״̬�Ĵ�������  	
	do{
		if(SPI1_ReadWriteByte(0Xff,&byte)==TRUE)//��ȡһ���ֽ�  
			sr=byte; 
	}while((sr&0x01)==0x01);   // �ȴ�BUSYλ���
	BURN_FLASH_CS=1;                            //ȡ��Ƭѡ    
}  
//�������ģʽ
void BURN_Flash_PowerDown(void)   
{ 
  	BURN_FLASH_CS=0;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_PowerDown,NULL);        //���͵�������  
	BURN_FLASH_CS=1;                            //ȡ��Ƭѡ     	      
    delay_us(3);                               //�ȴ�TPD  
}   
//����
void BURN_Flash_WAKEUP(void)   
{  
  	BURN_FLASH_CS=0;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_ReleasePowerDown,NULL);   //  send W25X_PowerDown command 0xAB    
	BURN_FLASH_CS=1;                            //ȡ��Ƭѡ     	      
    delay_us(3);                               //�ȴ�TRES1
}  
//BURN_Flashдʹ��	
//��WEL��λ   
void BURN_Flash_Write_Enable(void)   
{
	BURN_FLASH_CS=0;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_WriteEnable,NULL);      //����дʹ��  
	BURN_FLASH_CS=1;                            //ȡ��Ƭѡ     	      
} 
//BURN_Flashд��ֹ	
//��WEL����  
void BURN_Flash_Write_Disable(void)   
{  
	BURN_FLASH_CS=0;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_WriteDisable,NULL);     //����д��ָֹ��    
	BURN_FLASH_CS=1;                            //ȡ��Ƭѡ     	      
} 		
//��ȡоƬID
//����ֵ����:				   
//0XEF13,��ʾоƬ�ͺ�ΪW25Q80  
//0XEF14,��ʾоƬ�ͺ�ΪW25Q16    
//0XEF15,��ʾоƬ�ͺ�ΪW25Q32  
//0XEF16,��ʾоƬ�ͺ�ΪW25Q64   	  
u16 BURN_Flash_ReadID(void)
{
	u8 datal,datah;
	u16 Temp = 0;	  
	BURN_FLASH_CS=0;
	SPI1_ReadWriteByte(0x90,NULL);//���Ͷ�ȡID����	    
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




//��ȡSPI FLASH  
//��ָ����ַ��ʼ��ȡָ�����ȵ�����
//pBuffer:���ݴ洢��
//ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit)
//NumByteToRead:Ҫ��ȡ���ֽ���(���65535)
void BURN_Flash_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead)   
{ 
 	u16 i;   										    
	BURN_FLASH_CS=0;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_FastReadData,NULL);         //���Ͷ�ȡ����   
    SPI1_ReadWriteByte((u8)((ReadAddr)>>16),NULL);  //����24bit��ַ    
    SPI1_ReadWriteByte((u8)((ReadAddr)>>8),NULL);   
    SPI1_ReadWriteByte((u8)ReadAddr,NULL); 
		SPI1_ReadWriteByte(W25X_Dummuy, NULL);
    for(i=0;i<NumByteToRead;i++)
	{ 
        SPI1_ReadWriteByte(0XFF,&pBuffer[i]);   //ѭ������  
    }
	BURN_FLASH_CS=1;  				    	      
}  
//SPI��һҳ(0~65535)��д������256���ֽڵ�����
//��ָ����ַ��ʼд�����256�ֽڵ�����
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���!!!	 
void BURN_Flash_Write_Page(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)
{
 	u16 i;  
    BURN_Flash_Write_Enable();                  //SET WEL 
	BURN_FLASH_CS=0;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_PageProgram,NULL);      //����дҳ����   
    SPI1_ReadWriteByte((u8)((WriteAddr)>>16),NULL); //����24bit��ַ    
    SPI1_ReadWriteByte((u8)((WriteAddr)>>8),NULL);   
    SPI1_ReadWriteByte((u8)WriteAddr,NULL);   
    for(i=0;i<NumByteToWrite;i++)SPI1_ReadWriteByte(pBuffer[i],NULL);//ѭ��д��  
	BURN_FLASH_CS=1;                            //ȡ��Ƭѡ 
	BURN_Flash_Wait_Busy();					   //�ȴ�д�����
} 

//�޼���дSPI FLASH 
//����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
//�����Զ���ҳ���� 
//��ָ����ַ��ʼд��ָ�����ȵ�����,����Ҫȷ����ַ��Խ��!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
//CHECK OK
void BURN_Flash_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 			 		 
	u16 pageremain;	   
	pageremain=256-WriteAddr%256; //��ҳʣ����ֽ���		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//������256���ֽ�
	while(1)
	{	   
		BURN_Flash_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//д�������
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //��ȥ�Ѿ�д���˵��ֽ���
			if(NumByteToWrite>256)pageremain=256; //һ�ο���д��256���ֽ�
			else pageremain=NumByteToWrite; 	  //����256���ֽ���
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
			BURN_Flash_Erase_Sector(secpos+i);//�����������
		}
		BURN_Flash_Write_NoCheck(pBuffer,secpos*4096,NumByteToWrite);//д���������� 
		return TRUE;
	}while(0);
	return FALSE;
}
//дSPI FLASH  
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ú�������������!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)						
//NumByteToWrite:Ҫд����ֽ���(���65535)   
u8 BURN_Flash_BUFFER[4096];		 
void BURN_Flash_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 
	u32 secpos;
	u16 secoff;
	u16 secremain;	   
 	u16 i;    
	u8 * BURN_Flash_BUF;	  
   	BURN_Flash_BUF=BURN_Flash_BUFFER;	     
 	secpos=WriteAddr/4096;//������ַ  
	secoff=WriteAddr%4096;//�������ڵ�ƫ��
	secremain=4096-secoff;//����ʣ��ռ��С   
 	//printf("ad:%X,nb:%X\r\n",WriteAddr,NumByteToWrite);//������
 	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//������4096���ֽ�
	while(1) 
	{	
		BURN_Flash_Read(BURN_Flash_BUF,secpos*4096,4096);//������������������
		for(i=0;i<secremain;i++)//У������
		{
			if(BURN_Flash_BUF[secoff+i]!=0XFF)break;//��Ҫ����  	  
		}
		if(i<secremain)//��Ҫ����
		{
			BURN_Flash_Erase_Sector(secpos);//�����������
			for(i=0;i<secremain;i++)	   //����
			{
				BURN_Flash_BUF[i+secoff]=pBuffer[i];	  
			}
			BURN_Flash_Write_NoCheck(BURN_Flash_BUF,secpos*4096,4096);//д����������  

		}else BURN_Flash_Write_NoCheck(pBuffer,WriteAddr,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������. 				   
		if(NumByteToWrite==secremain)break;//д�������
		else//д��δ����
		{
			secpos++;//������ַ��1
			secoff=0;//ƫ��λ��Ϊ0 	 

		   	pBuffer+=secremain;  //ָ��ƫ��
			WriteAddr+=secremain;//д��ַƫ��	   
		   	NumByteToWrite-=secremain;				//�ֽ����ݼ�
			if(NumByteToWrite>4096)secremain=4096;	//��һ����������д����
			else secremain=NumByteToWrite;			//��һ����������д����
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
// CNF3  bit14-bit15   ͨ��������� 00
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
// CNF3  bit14-bit15   ͨ��������� 00
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


//����
void UP_Reset(void)
{
//GPIOE_CRL -- mode  00  PE2 bit12-bit13
	GPIOE->CRL &= ~(0x03 << 8);
	GPIOE->CRL |=  0x01 <<8;
// CNF3  bit14-bit15   ͨ��������� 00
	GPIOE->CRL &= ~(0x03 << 10);
	GPIOE->CRL |=  0x00 <<10;
	// output 1
	GPIOE->ODR = 0x01<<2;
}

// ������
void LOWER_Reset(void)
{
//GPIOE_CRL -- mode  00  PE2 bit12-bit13
	GPIOE->CRL &= ~(0x03 << 8);
	GPIOE->CRL |=  0x01 <<8;
// CNF3  bit14-bit15   ͨ��������� 00
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
























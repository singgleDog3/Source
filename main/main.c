#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"	 	 
#include "timer.h"
#include "flash.h"
#include "sram.h"
#include "malloc.h"
#include "string.h"
#include "mmc_sd.h"
#include "mass_mal.h"
#include "usb_lib.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "memory.h"	    
#include "usb_bot.h" 
#include "ff.h"
#include "exfuns.h"
#include "exti.h"
#include "power_control.h"
#include "lower.h"

#include "burn_flash.h"
#include "uart2_download.h"
#include "chooseDownload.h"

#include <stdio.h>

u8 bd_addr[6];
FIL *f_bin;
u8 read_value[520];


extern enum lower_state_t lower_state;

//debug flag
extern uint8_t connReq_flag;
uint32_t num = 0;

//����USB ����/����
//enable:0,�Ͽ�
//       1,��������	   
void usb_port_set(u8 enable)
{
    RCC->APB2ENR|=1<<2;    //ʹ��PORTAʱ��	   	 
    if(enable) {
        _SetCNTR(_GetCNTR()&(~(1<<1)));//�˳��ϵ�ģʽ
    }
    else {	  
        _SetCNTR(_GetCNTR()|(1<<1));  // �ϵ�ģʽ
        GPIOA->CRH&=0XFFF00FFF;
        GPIOA->CRH|=0X00033000;
        PAout(12)=0;	    		  
    }
}

// Patch �ļ�����Ϣ
static uint32_t patch_total;
static uint32_t patch_left;
static uint32_t patch_read;
static uint32_t patch_onetime = 256;
static uint32_t SRam3_Addr = 0;
static uint32_t patch_size;

extern uint8_t DataBuf[OneTime_Num];

// param �ļ���Ϣ
static uint32_t param_total;
static uint32_t param_read;
static uint32_t param_left;
static uint32_t param_onetime = 256;
static uint32_t param_size;
static uint32_t param_baseAddr = 0;

int main(void)
{		
extern u8 StartFlag;
extern enum UART2Download_State UART2Doanload_State;
extern uint32_t AckNum;
extern unsigned char usbTransfer_Flag ;
extern u8 TimeOut;
extern uint8_t Fail_Flag ;
extern uint8_t Success_Flag ;
#if 1
    u8 offline_cnt=0;
    u8 tct=0;
    u8 USB_STA;
    u8 Divece_STA;
    DIR root_dir;
    static FILINFO file_info;
    UINT file_num;
    u8 res;
    FIL *f_bdaddr_bin;
		
	
    Stm32_Clock_Init(6);//ϵͳʱ������
    delay_init(72);		//��ʱ��ʼ��
		uart_init(36,115200); //����1��ʼ��


    LED_Init();         //LED��ʼ��
    EXTIX_Init();
		HC244_ctrl_pin_init();	//...
    power_control_pin_init();
		FSMC_SRAM_Init();
#if UART_DOWNLOAD
		TIM3_Int_Init(5000, 48*1000-1);  //36 M
		TIM3_DisEnable();
#endif

    //LED1 = LED_OFF;	//...
    //LED2 = LED_OFF;
		BusyPin_Init();
		
#if LED_ShowState
		EnableBusy();
		LED1 = LED_ON;	//...
		LED2 = LED_ON;
#endif

    SPI_Flash_Init();
    if(SD_Initialize())	{//���SD������
    }
    else {//SD ������														  
        Mass_Memory_Size[0]=(long long)SD_GetSectorCount()*512;//�õ�SD���������ֽڣ�����SD����������4G��ʱ��,��Ҫ�õ�����u32����ʾ
        Mass_Block_Size[0] =512;//��Ϊ������Init����������SD���Ĳ����ֽ�Ϊ512��,��������һ����512���ֽ�.
        Mass_Block_Count[0]=Mass_Memory_Size[0]/Mass_Block_Size[0];
    }
    

    if(SPI_FLASH_TYPE!=W25Q64) {//���SD������
    }
    else {//SPI FLASH ����															  
        Mass_Memory_Size[1]=1024*1024*6;//ǰ6M�ֽ�
        Mass_Block_Size[1] =512;//��Ϊ������Init����������SD���Ĳ����ֽ�Ϊ512��,��������һ����512���ֽ�.
        Mass_Block_Count[1]=Mass_Memory_Size[1]/Mass_Block_Size[1];
    }

    if(SPI_FLASH_TYPE!=0xEF12) {//���SD������
    }
    else {//SPI FLASH ����															  
        Mass_Memory_Size[1]=1024*512;//512K
        Mass_Block_Size[1] =512;//��Ϊ������Init����������SD���Ĳ����ֽ�Ϊ512��,��������һ����512���ֽ�.
        Mass_Block_Count[1]=Mass_Memory_Size[1]/Mass_Block_Size[1];
    }

    delay_ms(1000);
    //printf("error.\r\n");
		
    mem_init(SRAMIN);
    exfuns_init();
    f_mount(0, fs[0]);
    f_mount(1, fs[1]);
#if 1
    delay_ms(1800);
    usb_port_set(0); 	//USB�ȶϿ�
    delay_ms(300);
    usb_port_set(1);	//USB�ٴ�����
    //USB����
    USB_Interrupts_Config();
    Set_USBClock();
    USB_Init();
    delay_ms(1800);
#endif
    if(f_opendir(&root_dir, "1:")==FR_OK) {
        file_info.lfsize = _MAX_LFN*2+1;
	    file_info.lfname = mymalloc(SRAMIN, file_info.lfsize);
        while(1) {
            res = f_readdir(&root_dir, &file_info);
            if((res != FR_OK) ||(file_info.fname[0] == 0)) {
                break;
            }
        }
    }

		
#if 1
    f_bin = (FIL *)mymalloc(SRAMIN,sizeof(FIL));
		
// ��patchweb�ļ����ص�SRAM��
    if(f_open(f_bin, (const TCHAR*)"1:/PATCH.bin", FA_READ) == FR_OK) {
			patch_total = f_bin->fsize;
			patch_left = patch_total;
			patch_read = 0;
			for(; patch_read < patch_total; ){
				if(patch_left > patch_onetime){
						patch_size = patch_onetime;
				}else{
					patch_size = patch_left;		
				}
				f_read(f_bin, DataBuf, patch_onetime, &file_num);
				FSMC_SRAM_WriteBuffer(DataBuf, patch_read , patch_size);
				patch_read+= patch_size;
				patch_left = patch_total - patch_read;
			}
        f_close(f_bin);
#if LED_ShowState
				LED1 = LED_OFF;	//...
#endif
    }
		
    if(f_open(f_bin, (const TCHAR*)"1:/hid.bin", FA_READ) == FR_OK) {
        device_type = HID_D_F_DEVICE;
    }
    else {
        if(f_open(f_bin, (const TCHAR*)"1:/flash.bin", FA_READ) == FR_OK) {
							device_type = AUDIO_DEVICE_FLASH;
					
							param_baseAddr = patch_total;
							param_total = f_bin->fsize;
							param_left = param_total;
							param_read = 0;
							for(; param_read < param_total;){
								if(param_left > param_onetime){
									param_size = param_onetime;
								}else{
									param_size = param_left;
								}
								f_read( f_bin, DataBuf, param_size, &file_num);
								FSMC_SRAM_WriteBuffer(DataBuf, param_baseAddr + param_read,param_size);
								param_read += param_size;
								param_left = param_total - param_read;
							}
							f_close(f_bin);
#if LED_ShowState
								LED2 = LED_OFF;
#endif
        }
        else {
            if(f_open(f_bin, (const TCHAR*)"1:/eeprom.bin", FA_READ) == FR_OK) {
                device_type = AUDIO_DEVICE_EEPROM;
            }
            else {
                myfree(SRAMIN, (void *)f_bin);
                f_bin = NULL;
            }
        }
    }


     lower_assign_opcode();

#endif

//    LED1 = LED_ON;	//...
//    LED2 = LED_OFF;	//...
#endif
		
#if UART_DOWNLOAD
InitStartPin();
while(1){	
	
	
	if(StartFlag == 1){
		
			DisEnable_USB();
			NGPin_Init();
			OKPin_Init();
			BusyPin_Init();
			EnableBusy();
			uart2_ChangeBaudRate(115200);
		// if load the patch or param_bin file to SRAM Falied; send NG single
		if(patch_total == 0 | param_total == 0){
				while(1){
					NG_Single();
				}
			}
			ParamInfo_Init(param_total, patch_total);
			PatchInfo_Init(patch_total);
		
			// ��fr3180 
			Enable_OutVcc();
			RESET_Single();
		  TIM3_Enable();
		
			//��fr3180 ��������,����״̬��
			UART2Doanload_State = ReadFlashData_ACK;
			AckNum_Clear();
#if LED_ShowState		
			LED1 = LED_ON;
			LED2 = LED_ON;
#endif
			uart2_SuccessConnACK();
			delay_ms(10);
			uart2_tryReadFlash();
			
			while(StartFlag); //�ȴ���¼���̵Ľ���
			DISEnable_OutVcc();
			TIM3_DisEnable();
			DisEnableBusy();
			Enable_USB();
	}
	
		if(TimeOut == 1){
#if LED_ShowState
			LED1 = LED_ON;
			LED2 = LED_ON;
			
			LED1 = LED_OFF;
#endif
			TimeOut = 0;
			NG_Single();
		}
		if(Fail_Flag == 1){
#if LED_ShowState
			LED1 = LED_ON;
			LED2 = LED_ON;
			
			LED2 = LED_OFF;
#endif
			Fail_Flag = 0;
		}
		if(Success_Flag == 1){
#if LED_ShowState
			LED1 = LED_ON;
			LED2 = LED_ON;
			
			LED1 = LED_OFF;
			LED2 = LED_OFF;
#endif
			Success_Flag = 0;
		}
	 delay_ms(1);				  
}



#endif


#if SPI_DOWNLOAD
	InitStartPin();
    while(1) {
			
if( StartFlag == 1){
	if(lower_get_state() != LOWER_DISABLE) {
			lower_uart_reset();
	}
	
			//DisEnable_USB();
			NGPin_Init();
			OKPin_Init();
			BusyPin_Init();
			EnableBusy();
	// if load the  param_bin file to SRAM Falied; send NG single
			if(patch_total == 0 ){
				while(1){
					NG_Single();
				}
			}
	
			ParamInfo_Init(param_total, patch_total);
			PatchInfo_Init(patch_total);
		
			// ��fr3180 
			Enable_OutVcc();
			RESET_Single();
#if LED_ShowState	
			LED1 = LED_ON;
			LED2 = LED_ON;
#endif
	
			lower_set_state(LOWER_IDLE);
			BURN_Flash_Init();
			BURN_Flash_ReadID();
		if(burnFlashbysector() ==TRUE)
		{
			if(CheckSPIDownload(param_total) == TRUE){
#if LED_ShowState
				LED1 = LED_OFF;
				LED2 = LED_OFF;
				delay_ms(500);
#endif
				
				DisEnableBusy();
				OK_Single();
				delay_ms(100);
				EnableBusy();
				
			}else{
#if LED_ShowState
				LED1 = LED_OFF;
				LED2 = LED_OFF;
				delay_ms(500);
#endif
				DisEnableBusy();
				NG_Single();
				delay_ms(100);
				EnableBusy();
			}			
		}	
		StartFlag = 0;
		DISEnable_OutVcc();
		TIM3_DisEnable();
		DisEnableBusy();
		Enable_USB();
}

#if 0
if(key1_pressed == 1) {
	key1_pressed = 0;
	if(lower_get_state() != LOWER_DISABLE) {
			lower_uart_reset();
	}
	ParamInfo_Init(param_total, patch_total);
	PatchInfo_Init(patch_total);

	
	HC244_enable();
	Enable_OutVcc();	
	RESET_Single();							

	LED1 = LED_OFF;
	LED2 = LED_OFF;
	lower_set_state(LOWER_IDLE);
	power_control_pin_off();
	delay_ms(60);	//...
	power_control_pin_on();
	BURN_Flash_Init();
	//burnFLASH();
	if(burnFlashbysector()!=TRUE)
	{
	LED1 = LED_OFF;
	LED2 = LED_ON;				
	}
	HC244_enable();
	BURN_Flash_Read(read_value, 0x0000 , 250);
	HC244_disable();
	StartFlag = 0;
}

        if(key2_pressed) {
            key2_pressed = 0;
            if(lower_get_state() != LOWER_DISABLE) {
                lower_uart_reset();
            }
			HC244_enable();
            LED1 = LED_OFF;
            LED2 = LED_OFF;
            lower_set_state(LOWER_IDLE);
            power_control_pin_off();
            delay_ms(60);	//...
            power_control_pin_on();
#if 0            
			low_fix_trans_bug();
#else
			BURN_Flash_Init();
			//burnFLASH();
			if(burnFlashbysector()!=TRUE)
			{
				LED1 = LED_OFF;
				LED2 = LED_ON;				
			}
#endif
        }
        
        if(key3_pressed) {
            if(lower_get_state() == LOWER_DISABLE) {
                key3_pressed = 0;
                if(f_bin) {
                    f_close(f_bin);
                }
                f_bdaddr_bin = (FIL *)mymalloc(SRAMIN,sizeof(FIL));
                if(f_open(f_bdaddr_bin, (const TCHAR*)"1:/bd_addr.bin", FA_WRITE) == FR_OK) {
                    f_write(f_bdaddr_bin, bd_addr, 6, &file_num);
                    f_close(f_bdaddr_bin);
                }
                myfree(SRAMIN, f_bdaddr_bin);
                if(f_bin) {
                    switch(device_type) {
                        case AUDIO_DEVICE_EEPROM:
                            f_open(f_bin, (const TCHAR*)"1:/eeprom.bin", FA_READ);
                            break;
                        case AUDIO_DEVICE_FLASH:
                            f_open(f_bin, (const TCHAR*)"1:/flash.bin", FA_READ);
                            break;
                        case HID_D_F_DEVICE:
                        case HID_E_G_DEVICE:
                            f_open(f_bin, (const TCHAR*)"1:/hid.bin", FA_READ);
                            break;
                    }
                }
            }
        }
#endif
				
				delay_ms(1);
#if 0
				  
        if(USB_STA!=USB_STATUS_REG) {//״̬�ı��� 
            if(USB_STATUS_REG&0x01) {//����д
                //��ʾUSB����д������	 
            }
            if(USB_STATUS_REG&0x02) {//���ڶ�
                //��ʾUSB���ڶ�������  		 
            }	 										  
            if(USB_STATUS_REG&0x04) {//��ʾд�����
            }
            else {//�����ʾ	  
            }
            if(USB_STATUS_REG&0x08) {//��ʾ��������
            }
            else {//�����ʾ    
            }
            USB_STA=USB_STATUS_REG;//��¼����״̬
        }
        if(Divece_STA!=bDeviceState) 
        {
            if(bDeviceState==CONFIGURED) {//��ʾUSB�����Ѿ�����
            }
            else {//��ʾUSB���γ���
            }
            Divece_STA=bDeviceState;
        }
        tct++;
        if(tct==200)
        {
            tct=0;
            //LED0=!LED0;//��ʾϵͳ������
            if(USB_STATUS_REG&0x10) {
                offline_cnt=0;//USB������,�����offline������
                bDeviceState=CONFIGURED;
            }else {//û�еõ���ѯ 
                offline_cnt++;  
                if(offline_cnt>10)
                    bDeviceState=UNCONNECTED;//2s��û�յ����߱��,����USB���γ���
								
            }
            USB_STATUS_REG=0;
        }

#endif
		
	    }; 	
#endif

		
}


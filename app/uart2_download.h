#ifndef _UART2DOWNLOAD_H
#define _UART2DOWNLOAD_H

#include "lower.h"

#define FALSE              (0)
#define TRUE               (1)
#define READING            (2)
#define WriteData_End      (3)
#define WriteData_NotEnd   (4)

#define OneTime_Num (513)

typedef unsigned int uint32_t;
typedef unsigned short uin16_t;
typedef unsigned char uint8_t;

typedef unsigned long int UL_int;

//ÉÕÐ´×´Ì¬
enum UART2Download_State	{
	NoCore,
	
	Connect_ACK,
	ReadFlashData_ACK,
	PatchWrite_ACK,
	RepaceFunc_Ack,
	Redefine_Ack,
	ChangeBaudRate_ACK,
	ChangeBaudRateSuccess_ACK,
	Erasesure_ACK,
	
	ParamWrite_ACK,
	ParamWriteOver_ACK,
	
	ReadParamFile_ACK,
};


void AckNum_Clear(void);

void OKPin_Init(void);
void NGPin_Init(void);
void NG_Single(void);
void OK_Single(void);

void IntToStr(uint8_t *str, uint32_t num);
void PatchInfo_Init(uint32_t total);
void ParamInfo_Init(uint32_t param_total, uint32_t patch_total);
void ReadInfo_Init(void);

void My_strcpy(char *str1, char *str2, uint32_t num);

UL_int StrToULInt(uint8_t *str);
void ULintToStr(UL_int addrNum, uint8_t *str);
void EnableBusy(void);
void DisEnableBusy(void);
void BusyPin_Init(void);
#endif


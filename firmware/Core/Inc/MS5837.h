#ifndef __MS5837_H_
#define __MS5837_H_



#include "stdio.h"
#include "stdint.h"
#define MS5837_30BA_WriteCommand     0xEC
#define MS5837_30BA_ReadCommand      0xED

#define MS5837_30BA_ResetCommand     0x1E                //复位
#define	MS5837_30BA_PROM_RD 	       0xA0                //PROM读取,{0XA0,0XA2,0XA4,0XA8,0XAA,0XAC,0XAE}
#define MS5837_30BA_ADC_RD           0x00                //ADC读取              

#define MS5837_30BA_OSR256					 0x40
#define MS5837_30BA_OSR512					 0x42
#define MS5837_30BA_OSR1024					 0x44
#define MS5837_30BA_OSR2048					 0x46
#define MS5837_30BA_OSR4096					 0x48
#define	MS5837_30BA_D2_OSR_8192  0x54 // 0x58 	//                //16.44ms转换时间
#define	MS5837_30BA_D1_OSR_8192   0x44// 0x48 //	                //16.44ms转换时间

typedef struct
{
	uint32_t Depth;
	int32_t Temp;
	int32_t Pressure;
	
	uint32_t D1_Pres;
	uint32_t D2_Temp;
	
	uint16_t Cal_C[7];                    //用于存放PROM中的6组数据1-6

	int32_t Atmosphere_Pressure;          //大气压基准值 (单位与Pressure一致)

}MS5837_data;

typedef struct
{
	float depth;
	float temperature;
	float pressure;
	
}MS5837_result;

extern MS5837_data MS5837_30BA_data;
extern MS5837_result MS5837_30BA_result;

unsigned char MS5837_30BA_Crc4(void);
void MS5837Init(void);
void MS5837_ReInit(void);
void MS5837Read(void);
float MS5837DataGet(void);
float MS5837_GetDepth(void);   // 返回深度值 (单位: 米)

#endif

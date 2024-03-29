#ifndef _AS5047P_H__
#define _AS5047P_H__

#include "IQmathLib.h"
#include <math.h>
#include "BLDC_MOTOR.h"
#include "spi.h"
#include "FOC_Math.h"

// AS5047p 地址
#define NOP 0x0000
#define ERRFL 0x0001
#define PROG 0x0003
#define DIAAGC 0x3FFC
#define MAG 0x3FFD
#define ANGLEUNC 0x3FFE
#define ANGLECOM 0x3FFF

#define ZPOSM 0x0016
#define ZPOSL 0x0017
#define SETTINGS1 0x0018
#define SETTINGS2 0x0019

#define FREQ_SAM (60) // 采样频率 60Hz

uint16_t SPI_ReadWrite_OneByte(uint16_t _txdata);
uint16_t SPI_ReadWrite_OneByte(uint16_t _txdata);
uint16_t AS5047_read(uint16_t add);
void Read_Angle(foc_data *motor);
void GetMotor_Speed(foc_data *motor);

#endif
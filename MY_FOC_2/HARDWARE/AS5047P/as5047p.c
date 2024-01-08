#include "as5047p.h"
#include "main.h"

// 计算奇偶函数
uint16_t Parity_bit_Calculate(uint16_t data_2_cal)
{
	uint16_t parity_bit_value=0;
	while(data_2_cal != 0)
	{
		parity_bit_value ^= data_2_cal; 
		data_2_cal >>=1;
	}
	return (parity_bit_value & 0x1); 
}

// SPI发送读取函数
uint16_t SPI_ReadWrite_OneByte(uint16_t _txdata)
{
	HAL_GPIO_WritePin(SPI_CSN_GPIO_Port, SPI_CSN_Pin, GPIO_PIN_RESET);
	uint16_t rxdata;
	if(HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&_txdata, (uint8_t *)&rxdata,1,1000) !=HAL_OK)
	rxdata=0;
    HAL_GPIO_WritePin(SPI_CSN_GPIO_Port, SPI_CSN_Pin, GPIO_PIN_SET);
	return rxdata;
}

// 速度计算
void GetMotor_Speed(foc_data *motor)
{
    static float mangle = 0;
    static float last_mangle = 0;
    static float last_speed = 0;

    mangle = motor->angle;
    motor->speed = (mangle - last_mangle)/360 *60 * FREQ_SAM;  // delta_seta/ t_sam * 60 /360   du/s -> rpm =>60Hz sample Freq
    if(motor->speed > 1000 || motor->speed < -1000) 
        motor->speed = last_speed;

    last_mangle = mangle;
    last_speed = motor->speed;
}

// 读取as5047p角度
uint16_t AS5047_read(uint16_t add)
{
	uint16_t data;
	add |= 0x4000;	// 读指令 bit14 置1
	if(Parity_bit_Calculate(add)==1) add=add|0x8000; //如果前15位 1的个数位偶数，则Bit15 置1
	SPI_ReadWrite_OneByte(add);		//发送一条指令，不管读回的数据
	data=SPI_ReadWrite_OneByte(NOP|0x4000); //发送一条空指令，读取上一次指令返回的数据。
	data &=0x3fff;
	return data;
}

// 角度(弧度)获取
void Read_Angle(foc_data *motor)
{
    static uint32_t data ;
    static float angle;
    static float angle_el;

    data = AS5047_read(ANGLEUNC);
    angle=((float)data*360.0f)/16384.0f-motor->angle_Offect;  // 转换为角度

    // 判别角度范围，调整至0-360°
    while(angle<0) 
        angle+=360.0f;

    angle=360-angle;
    angle_el=angle*(float)motor->pole_pairs;

    while(angle_el>360) 
        angle_el-=360.0f;
    
    motor->angle=angle;
    motor->angle_el=angle_el;
    motor->angle_eRadian=angle_el*(float)FOC_EANGLE_TO_ERADIN; // 转换为弧度 rad
}
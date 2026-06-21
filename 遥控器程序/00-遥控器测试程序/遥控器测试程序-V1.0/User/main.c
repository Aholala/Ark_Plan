#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Key.h"
#include "AD.h"
#include "NRF24L01.h"
#include "Timer.h"

uint16_t AD_LH, AD_LV, AD_RH, AD_RV;
int8_t LH, LV, RH, RV;

uint8_t KEY0, KEY1;
uint8_t KeyNum;

uint8_t SendFlag;
uint8_t SuccessRatio;

uint8_t Flag;

uint8_t Mode = 0;

#define AD_DEAD			100

int8_t DataProcess(uint16_t ADValue);
uint8_t CalculateSuccessRatio(uint8_t SendFlag);

int main(void)
{
	/*初始化*/
	OLED_Init();
	Key_Init();
	AD_Init();
	NRF24L01_Init();
	Timer_Init();
	
	/*显示初始界面*/
	OLED_Clear();
	OLED_ShowString(0, 0,  "   [江协科技]   ", OLED_8X16);
	OLED_ShowString(4, 16, " 遥控器测试程序 ", OLED_8X16);
	OLED_ShowString(0, 32, "      V1.0      ", OLED_8X16);
	OLED_ShowString(0, 48, "            K10>", OLED_8X16);
	OLED_Update();
	
	/*按K10键继续*/
	while (Key_Check(KEY_10, KEY_SINGLE) == 0);
	Key_Clear();

	OLED_Clear();
	
	while (1)
	{
		KeyNum = 0;
		
		/*读取左右摇杆的AD值*/
		AD_LH = AD_GetValue(ADC_Channel_0);
		AD_LV = AD_GetValue(ADC_Channel_1);
		AD_RH = AD_GetValue(ADC_Channel_2);
		AD_RV = AD_GetValue(ADC_Channel_3);
		
		/*对AD值进行数据处理，得到摇杆数据*/
		LH = DataProcess(AD_LH);
		LV = DataProcess(AD_LV);
		RH = DataProcess(AD_RH);
		RV = DataProcess(AD_RV);
		
		/*读取按键*/
		if (Key_Check(KEY_1, KEY_SINGLE))
		{
			KEY0 |= 0x01;
			KeyNum = 1;
		}
		if (Key_Check(KEY_2, KEY_SINGLE))
		{
			KEY0 |= 0x02;
			KeyNum = 2;
		}
		if (Key_Check(KEY_3, KEY_SINGLE))
		{
			KEY0 |= 0x04;
			KeyNum = 3;
		}
		if (Key_Check(KEY_4, KEY_SINGLE))
		{
			KEY0 |= 0x08;
			KeyNum = 4;
		}
		if (Key_Check(KEY_5, KEY_SINGLE))
		{
			KEY0 |= 0x10;
			KeyNum = 5;
		}
		if (Key_Check(KEY_6, KEY_SINGLE))
		{
			KEY0 |= 0x20;
			KeyNum = 6;
		}
		if (Key_Check(KEY_7, KEY_SINGLE))
		{
			KEY0 |= 0x40;
			KeyNum = 7;
		}
		if (Key_Check(KEY_8, KEY_SINGLE))
		{
			KEY0 |= 0x80;
			KeyNum = 8;
		}
		if (Key_Check(KEY_9, KEY_SINGLE))
		{
			KEY1 |= 0x01;
			KeyNum = 9;
		}
		if (Key_Check(KEY_10, KEY_SINGLE))
		{
			KEY1 |= 0x02;
			KeyNum = 10;
		}
		if (Key_Check(KEY_11, KEY_SINGLE))
		{
			KEY1 |= 0x04;
			KeyNum = 11;
		}
		if (Key_Check(KEY_12, KEY_SINGLE))
		{
			KEY1 |= 0x08;
			KeyNum = 12;
		}
		
		/*定时器中，每隔100ms，置一次Flag*/
		/*即每隔100ms，发送一次数据*/
		if (Flag)
		{
			Flag = 0;
			
			/*填充遥控器数据*/
			NRF24L01_TxPacket[0] = Mode;
			NRF24L01_TxPacket[1] = LH;
			NRF24L01_TxPacket[2] = LV;
			NRF24L01_TxPacket[3] = RH;
			NRF24L01_TxPacket[4] = RV;
			NRF24L01_TxPacket[5] = KEY0;
			NRF24L01_TxPacket[6] = KEY1;
			
			/*发送遥控器数据*/
			SendFlag = NRF24L01_Send();
			
			/*发送成功后，清除按键，确保按键被成功发出*/
			if (SendFlag == 1)
			{
				KEY0 = 0;
				KEY1 = 0;
			}
			
			/*计算发送成功率，用于显示信号值*/
			SuccessRatio = CalculateSuccessRatio(SendFlag);
		}
		
		/*按键按下，在屏幕右上角闪烁此按键值*/
		if (KeyNum)
		{
			OLED_Printf(104, 0, OLED_8X16, "K%d", KeyNum);
			OLED_UpdateArea(104, 0, 24, 16);
			Delay_ms(100);
			OLED_ClearArea(104, 0, 24, 16);
			OLED_UpdateArea(104, 0, 24, 16);
		}
		
		/*K9键按下，切换显示模式*/
		if (KeyNum == 9)
		{
			Mode = !Mode;
			OLED_Clear();
		}
		
		/*模式0时，屏幕显示摇杆数据*/
		if (Mode == 0)
		{
			OLED_Printf(0, 16, OLED_8X16, "LH:%+04d", LH);
			OLED_Printf(0, 32, OLED_8X16, "LV:%+04d", LV);
			OLED_Printf(72, 16, OLED_8X16, "RH:%+04d", RH);
			OLED_Printf(72, 32, OLED_8X16, "RV:%+04d", RV);
		}
		/*模式1时，屏幕显示平衡车返回的状态信息*/
		else if (Mode == 1)
		{
			if (NRF24L01_Receive() == 1)
			{
				uint8_t SpeedLevel = NRF24L01_RxPacket[1];
				int8_t PWML = NRF24L01_RxPacket[2];
				int8_t PWMR = NRF24L01_RxPacket[3];
				float Angle = *(float *)&NRF24L01_RxPacket[4];
				float SpeedLeft = *(float *)&NRF24L01_RxPacket[8];
				float SpeedRight = *(float *)&NRF24L01_RxPacket[12];
				
				OLED_Clear();
				OLED_Printf(0, 21, OLED_6X8, "Angle:%+06.1f", Angle);
				OLED_DrawLine(0, 36, 127, 36);
				OLED_Printf(0, 40, OLED_6X8, "PWML:%+05d PWMR:%+05d", PWML, PWMR);
				OLED_Printf(0, 48, OLED_6X8, "SpdL:%+05.1f SpdR:%+05.1f", SpeedLeft, SpeedRight);
				OLED_Printf(0, 56, OLED_6X8, "SpdLevel:%1d", SpeedLevel);
				
			}
		}
		
		/*根据发送成功率，在屏幕左上角显示对应的信号值*/
		if (SuccessRatio >= 9)
		{
			OLED_ShowImage(0, 0, 16, 16, Signal_3);
		}
		else if (SuccessRatio >= 5)
		{
			OLED_ShowImage(0, 0, 16, 16, Signal_2);
		}
		else if (SuccessRatio >= 1)
		{
			OLED_ShowImage(0, 0, 16, 16, Signal_1);
		}
		else
		{
			OLED_ShowImage(0, 0, 16, 16, Signal_0);
		}
		
		OLED_Update();
	}
}

int8_t DataProcess(uint16_t ADValue)		//摇杆AD数据处理函数
{
	/*输入ADValue，范围：0 ~ 4095*/
	int32_t Value;;
	
	/*减去中心AD值，范围：-2048 ~ 2047*/
	Value = ADValue - 2048;
	
	/*摇杆回中时，设定一个死区区间，确保回中时摇杆值为0*/
	/*调整后，范围：-(2048-AD_DEAD) ~ 2047-AD_DEAD*/
	if (Value > AD_DEAD)
	{
		Value -= AD_DEAD;
	}
	else if (Value < -AD_DEAD)
	{
		Value += AD_DEAD;
	}
	else
	{
		Value = 0;
	}
	
	/*对值进行缩放，得到最终摇杆数据*/
	/*缩放后，范围：-100 ~ 100*/
	Value = Value * 101 / (2049 - AD_DEAD);
	
	return Value;
}

uint8_t CalculateSuccessRatio(uint8_t SendFlag)		//计算发送成功率
{
	static uint8_t SendFlagArray[10];
	static uint8_t p;
	uint8_t i, SuccessCount;
	
	/*取最近10次的SendFlag，统计发送成功的次数*/
	SendFlagArray[p] = SendFlag;
	p ++;
	p %= 10;
	
	SuccessCount = 0;
	for (i = 0; i < 10; i ++)
	{
		if (SendFlagArray[i] == 1)
		{
			SuccessCount ++;
		}
	}
	
	/*返回最近10次发送成功的次数*/
	return SuccessCount;
}

void TIM1_UP_IRQHandler(void)
{
	static uint16_t Count;
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		/*定时器定时中断，每隔1ms自动执行一次*/
		
		/*驱动非阻塞式按键*/
		Key_Tick();
		
		Count ++;
		if (Count >= 100)
		{
			Count = 0;
			
			Flag = 1;		//100ms置一次Flag
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

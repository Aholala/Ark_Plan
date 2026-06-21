#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "Key.h"
#include "AD.h"
#include "NRF24L01.h"

/*OLED测试*/
//int main(void)
//{
//	OLED_Init();
//	
//	OLED_Printf(0, 0, OLED_8X16, "Hello World!");
//	OLED_Printf(0, 16, OLED_6X8, "Hello World!");
//	OLED_Update();
//	
//	while (1)
//	{
//		
//	}
//}

/*定时中断和非阻塞式按键测试*/
//uint16_t Count;
//uint8_t KeyNum, Num;

//int main(void)
//{
//	OLED_Init();
//	Key_Init();
//	
//	Timer_Init();
//	
//	while (1)
//	{
//		KeyNum = Key_GetNum();
//		if (KeyNum)
//		{
//			Num = KeyNum;
//		}
//		
//		OLED_Printf(0, 0, OLED_8X16, "Count:%05d", Count);
//		OLED_Printf(0, 16, OLED_8X16, "Num:%03d", Num);
//		OLED_Update();
//	}
//}

//void TIM1_UP_IRQHandler(void)
//{
//	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
//	{
//		Count ++;
//		
//		Key_Tick();
//		
//		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
//	}
//}

/*摇杆测试*/
//int main(void)
//{
//	OLED_Init();
//	AD_Init();
//	
//	while (1)
//	{
//		OLED_Printf(0, 0, OLED_8X16, "AD0:%04d", AD_GetValue(ADC_Channel_0));
//		OLED_Printf(0, 16, OLED_8X16, "AD1:%04d", AD_GetValue(ADC_Channel_1));
//		OLED_Printf(0, 32, OLED_8X16, "AD2:%04d", AD_GetValue(ADC_Channel_2));
//		OLED_Printf(0, 48, OLED_8X16, "AD3:%04d", AD_GetValue(ADC_Channel_3));
//		OLED_Update();
//	}
//}

/*NRF24L01测试*/
uint8_t KeyNum;

int main(void)
{
	OLED_Init();
	NRF24L01_Init();
	Key_Init();
	
	Timer_Init();
	
	NRF24L01_WriteReg(NRF24L01_SETUP_RETR, 0xA5);
	uint8_t TestData = NRF24L01_ReadReg(NRF24L01_SETUP_RETR);
	
	OLED_Printf(0, 0, OLED_8X16, "TestData:%02X", TestData);
	OLED_Update();
	
	while (1)
	{
		KeyNum = Key_GetNum();
		if (KeyNum == 1)
		{
			NRF24L01_TxPacket[0] ++;
			NRF24L01_TxPacket[1] += 2;
			NRF24L01_TxPacket[2] += 3;
			NRF24L01_TxPacket[3] += 4;
			
			NRF24L01_Send();
			
			OLED_Printf(0, 16, OLED_8X16, "T:%02X %02X %02X %02X", 
				NRF24L01_TxPacket[0], NRF24L01_TxPacket[1], NRF24L01_TxPacket[2], NRF24L01_TxPacket[3]);
			OLED_Update();
		}
		
		if (NRF24L01_Receive() == 1)
		{
			OLED_Printf(0, 32, OLED_8X16, "R:%02X %02X %02X %02X", 
				NRF24L01_RxPacket[0], NRF24L01_RxPacket[1], NRF24L01_RxPacket[2], NRF24L01_RxPacket[3]);
			OLED_Update();
		}
	}
}

void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		Key_Tick();
		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

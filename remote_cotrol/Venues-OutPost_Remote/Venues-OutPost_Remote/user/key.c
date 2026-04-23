//使用实例
/*
 *
 *
extern struct Bkeys bkeys[];

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM6)
//	key_serv_long();
//	key_serv_long_2();
    key_serv_double();
}

if(bkeys[4].short_flag == 1)
    {
        sprintf(text,"       key4 D                 ");
      LCD_DisplayStringLine(Line4, (uint8_t *)text);
        bkeys[4].short_flag = 0;
    }
    if(bkeys[4].long_flag == 1)
    {
        sprintf(text,"       key4 D %d                 ",temp);
      LCD_DisplayStringLine(Line4, (uint8_t *)text);
    temp++;
    }
    if(bkeys[4].double_flag == 1)
    {
        sprintf(text,"       key4 Double                 ");
      LCD_DisplayStringLine(Line4, (uint8_t *)text);
        bkeys[4].double_flag = 0;
    }
*/
#include "key.h"

struct Bkeys bkeys[6] = {0,0};

uint8_t key_read()
{
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12)==0) return 1; //paw
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)==0) return 2;
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==1) return 3;
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==1) return 4;
    else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==1) return 5;
	else return 0;
	/*LB PA12
	  K1 PB0 
	  K2 PB1 
	  K3 PA1 
	  RB PB4*/
}

void key_serv()
{
	uint8_t key_sta = key_read();
	if(key_sta != 0) 
	{
		bkeys[key_sta].age++;
		if(bkeys[key_sta].age>10 && bkeys[key_sta].press==0) {
		    bkeys[key_sta].short_flag=1;
		    bkeys[key_sta].press=1;
		}
	}
	else 
	{
		for(int i=0;i<6;i++) {
		    bkeys[i].age=0;
		    bkeys[i].press=0;
		}
	}
}

//长按自增
void key_serv_long()
{
	uint8_t key_sta = key_read();
	if(key_sta != 0) 
	{
		bkeys[key_sta].age++;
		if(bkeys[key_sta].age==10)bkeys[key_sta].press=1;
	}
	else 
	{
		for(int i=0;i<5;i++) 
		{
			if(bkeys[i].press==1 && bkeys[i].long_flag==0)
			bkeys[i].short_flag=1;
			
			bkeys[i].press=0;
			bkeys[i].age=0;
			bkeys[i].long_flag=0;
		}
	}
	if(bkeys[key_sta].age>69) bkeys[key_sta].long_flag=1;
}

//1s反应
void key_serv_long_2()
{
	uint8_t key_sta = key_read();
	if(key_sta != 0) 
	{
		bkeys[key_sta].age++;
		if(bkeys[key_sta].age>1)bkeys[key_sta].press=1;
	}
	else 
	{
		for(int i=0;i<5;i++) 
		{
			if(bkeys[i].age>100) bkeys[i].long_flag=1;
			if(bkeys[i].press==1 && bkeys[i].long_flag==0)
			bkeys[i].short_flag=1;
			
			bkeys[i].press=0;
			bkeys[i].age=0;
		}
	}
}

void key_serv_double()
{
	uint8_t key_sta = key_read();
	if(key_sta != 0) 
	{
		bkeys[key_sta].age++;
		if(bkeys[key_sta].age>1)bkeys[key_sta].press=1;
	}
	else 
	{
		for(int i=0;i<5;i++) 
		{
			if(bkeys[i].press==1 && bkeys[i].double_ageEN==1) 
			{
				bkeys[i].double_flag=1;
				bkeys[i].press=0;
				bkeys[i].double_ageEN=0;
				bkeys[i].double_age=0;
			}
			if(bkeys[i].press==1 && bkeys[i].long_flag==0) bkeys[i].double_ageEN=1;
			if(bkeys[i].double_ageEN==1) bkeys[i].double_age++;
			if(bkeys[i].double_ageEN==1 && bkeys[i].double_age>100)
			{
				bkeys[i].short_flag=1;
				bkeys[i].double_age=0;
				bkeys[i].double_ageEN=0;
			}
			
			bkeys[i].press=0;
			bkeys[i].age=0;
			bkeys[i].long_flag=0;
		}
	}
	if(bkeys[key_sta].age>200) bkeys[key_sta].long_flag=1;
}

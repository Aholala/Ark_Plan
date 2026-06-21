/**
 * @file bsp_oled.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 板级支持包（BSP）OLED显示屏驱动头文件
 * @version 1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) 2026
 *
 * 本头文件定义了OLED显示屏（通常为SSD1306驱动，128x64分辨率）的驱动接口。
 * 提供屏幕初始化、显存更新、清屏、字符/字符串/数字显示、绘图（点、线、矩形、圆形等）
 * 以及格式化输出（OLED_Printf）等功能。支持两种字体：6x8和8x16。
 */

#ifndef __OLED_H
#define __OLED_H

#include "bsp_oled_data.h" /* 字库、图片数据等（外部定义） */
#include "main.h"          /* HAL库及硬件定义 */

/**
 * @brief 字体大小宏定义
 * @note  用于指定显示字符/字符串时的字体尺寸
 */
#define OLED_8X16 8U /**< 8x16点阵字体（宽8像素，高16像素） */
#define OLED_6X8 6U  /**< 6x8点阵字体（宽6像素，高8像素） */

/*------------------------ 初始化与基本操作 ------------------------*/

/**
 * @brief 初始化OLED显示屏
 * @retval 无
 * @note  配置I2C/SPI接口，发送初始化命令序列，清空显存
 */
void OLED_Init(void);

/**
 * @brief 将显存数据刷新到屏幕（全屏更新）
 * @retval 无
 * @note  将内部GRAM全部写入SSD1306的GDDRAM
 */
void OLED_Update(void);

/**
 * @brief 更新指定区域的显存到屏幕（部分更新）
 * @param X     起始X坐标（列，0~127）
 * @param Y     起始Y坐标（行，0~63）
 * @param Width 区域宽度（像素）
 * @param Height 区域高度（像素）
 * @note  仅更新指定矩形区域，可提高刷新效率
 */
void OLED_UpdateArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height);

/**
 * @brief 清空整个显存（屏幕变黑/白取决于取反设置）
 * @retval 无
 * @note  不清除屏幕，仅将内部显存清零，需调用OLED_Update()生效
 */
void OLED_Clear(void);

/**
 * @brief 清空指定矩形区域的显存
 * @param X     起始X坐标
 * @param Y     起始Y坐标
 * @param Width 区域宽度
 * @param Height 区域高度
 * @retval 无
 */
void OLED_ClearArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height);

/**
 * @brief 全局反色（黑白反转）
 * @retval 无
 * @note  将显存所有像素取反，并立即更新屏幕
 */
void OLED_Reverse(void);

/**
 * @brief 指定区域反色
 * @param X     起始X坐标
 * @param Y     起始Y坐标
 * @param Width 区域宽度
 * @param Height 区域高度
 * @retval 无
 */
void OLED_ReverseArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height);

/*------------------------ 字符与字符串显示 ------------------------*/

/**
 * @brief 在指定位置显示单个字符
 * @param X        X坐标（像素）
 * @param Y        Y坐标（像素）
 * @param Char     待显示的字符
 * @param FontSize 字体大小（OLED_8X16 或 OLED_6X8）
 * @retval 无
 */
void OLED_ShowChar(int16_t X, int16_t Y, char Char, uint8_t FontSize);

/**
 * @brief 在指定位置显示字符串
 * @param X        X坐标（像素）
 * @param Y        Y坐标（像素）
 * @param String   待显示的字符串指针
 * @param FontSize 字体大小（OLED_8X16 或 OLED_6X8）
 * @retval 无
 */
void OLED_ShowString(int16_t X, int16_t Y, char *String, uint8_t FontSize);

/**
 * @brief 显示无符号整数（十进制）
 * @param X        X坐标
 * @param Y        Y坐标
 * @param Number   待显示的数字
 * @param Length   显示位数（不足补零）
 * @param FontSize 字体大小
 * @retval 无
 */
void OLED_ShowNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length,
                  uint8_t FontSize);

/**
 * @brief 显示有符号整数（十进制，带正负号）
 * @param X        X坐标
 * @param Y        Y坐标
 * @param Number   待显示的数字（可为负数）
 * @param Length   显示位数（符号位不计入，不足补零）
 * @param FontSize 字体大小
 * @retval 无
 */
void OLED_ShowSignedNum(int16_t X, int16_t Y, int32_t Number, uint8_t Length,
                        uint8_t FontSize);

/**
 * @brief 显示十六进制数（无前缀）
 * @param X        X坐标
 * @param Y        Y坐标
 * @param Number   待显示的数字
 * @param Length   显示位数（不足补零）
 * @param FontSize 字体大小
 * @retval 无
 */
void OLED_ShowHexNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length,
                     uint8_t FontSize);

/**
 * @brief 显示二进制数（无前缀）
 * @param X        X坐标
 * @param Y        Y坐标
 * @param Number   待显示的数字
 * @param Length   显示位数（不足补零）
 * @param FontSize 字体大小
 * @retval 无
 */
void OLED_ShowBinNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length,
                     uint8_t FontSize);

/**
 * @brief 显示浮点数（带小数）
 * @param X          X坐标
 * @param Y          Y坐标
 * @param Number     待显示的浮点数
 * @param IntLength  整数部分位数（不足补零）
 * @param FraLength  小数部分位数
 * @param FontSize   字体大小
 * @retval 无
 */
void OLED_ShowFloatNum(int16_t X, int16_t Y, double Number, uint8_t IntLength,
                       uint8_t FraLength, uint8_t FontSize);

/**
 * @brief 显示图像（位图）
 * @param X      X坐标
 * @param Y      Y坐标
 * @param Width  图像宽度（像素）
 * @param Height 图像高度（像素）
 * @param Image  图像数据数组（按行存储，每个字节代表8个像素）
 * @retval 无
 */
void OLED_ShowImage(int16_t X, int16_t Y, uint8_t Width, uint8_t Height,
                    const uint8_t *Image);

/**
 * @brief 格式化输出（类似于printf，支持整数、浮点数、字符串等）
 * @param X        X坐标
 * @param Y        Y坐标
 * @param FontSize 字体大小
 * @param format   格式化字符串（如 "Value:%d"）
 * @param ...      可变参数
 * @retval 无
 * @note  支持 %d, %u, %x, %f, %s 等常用格式，具体实现请参考源码
 */
void OLED_Printf(int16_t X, int16_t Y, uint8_t FontSize, char *format, ...);

/*------------------------ 绘图函数 ------------------------*/

/**
 * @brief 画一个像素点（置1）
 * @param X X坐标
 * @param Y Y坐标
 * @retval 无
 */
void OLED_DrawPoint(int16_t X, int16_t Y);

/**
 * @brief 获取某像素点的当前值（0或1）
 * @param X X坐标
 * @param Y Y坐标
 * @return uint8_t 0表示空白，1表示点亮
 */
uint8_t OLED_GetPoint(int16_t X, int16_t Y);

/**
 * @brief 画一条直线（Bresenham算法）
 * @param X0 起点X坐标
 * @param Y0 起点Y坐标
 * @param X1 终点X坐标
 * @param Y1 终点Y坐标
 * @retval 无
 */
void OLED_DrawLine(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1);

/**
 * @brief 画矩形（可填充）
 * @param X         左上角X坐标
 * @param Y         左上角Y坐标
 * @param Width     矩形宽度
 * @param Height    矩形高度
 * @param IsFilled  是否填充（1填充，0仅边框）
 * @retval 无
 */
void OLED_DrawRectangle(int16_t X, int16_t Y, uint8_t Width, uint8_t Height,
                        uint8_t IsFilled);

/**
 * @brief 画三角形（可填充）
 * @param X0        第一个顶点X坐标
 * @param Y0        第一个顶点Y坐标
 * @param X1        第二个顶点X坐标
 * @param Y1        第二个顶点Y坐标
 * @param X2        第三个顶点X坐标
 * @param Y2        第三个顶点Y坐标
 * @param IsFilled  是否填充
 * @retval 无
 */
void OLED_DrawTriangle(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1,
                       int16_t X2, int16_t Y2, uint8_t IsFilled);

/**
 * @brief 画圆（可填充）
 * @param X         圆心X坐标
 * @param Y         圆心Y坐标
 * @param Radius    半径
 * @param IsFilled  是否填充
 * @retval 无
 */
void OLED_DrawCircle(int16_t X, int16_t Y, uint8_t Radius, uint8_t IsFilled);

/**
 * @brief 画椭圆（可填充）
 * @param X         圆心X坐标
 * @param Y         圆心Y坐标
 * @param A         水平半轴
 * @param B         垂直半轴
 * @param IsFilled  是否填充
 * @retval 无
 */
void OLED_DrawEllipse(int16_t X, int16_t Y, uint8_t A, uint8_t B,
                      uint8_t IsFilled);

/**
 * @brief 画圆弧（可填充成扇形）
 * @param X          圆心X坐标
 * @param Y          圆心Y坐标
 * @param Radius     半径
 * @param StartAngle 起始角度（度，0~360，0为右侧水平）
 * @param EndAngle   结束角度（度）
 * @param IsFilled   是否填充（1为扇形，0为弧线）
 * @retval 无
 */
void OLED_DrawArc(int16_t X, int16_t Y, uint8_t Radius, int16_t StartAngle,
                  int16_t EndAngle, uint8_t IsFilled);

#endif /* __OLED_H */
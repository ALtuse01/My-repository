/****************************************************************************/
/*                                                                          */
/*              广州创龙电子科技有限公司                                    */
/*                                                                          */
/*              Copyright 2014 Tronlong All rights reserved                 */
/*                                                                          */
/****************************************************************************/
/****************************************************************************/
/*                                                                          */
/*              按键中断测试                                                */
/*                                                                          */
/*              2014年4月20日                                               */
/*                                                                          */
/****************************************************************************/
#include "TL6748.h"                 // 创龙 DSP6748 开发板相关声明

#include "hw_types.h"               // 宏命令
#include "hw_syscfg0_C6748.h"       // 系统配置模块寄存器
#include "soc_C6748.h"              // DSP C6748 外设寄存器

#include "psc.h"                    // 电源与睡眠控制宏及设备抽象层函数声明
#include "gpio.h"                   // 通用输入输出口宏及设备抽象层函数声明

#include "interrupt.h"              // DSP C6748 中断相关应用程序接口函数声明及系统事件号定义

/****************************************************************************/
/*                                                                          */
/*              宏定义                                                      */
/*                                                                          */
/****************************************************************************/
// 软件断点
#define SW_BREAKPOINT   asm(" SWBP 0 ");

// 中断相关寄存器
cregister unsigned int IER,IFR,CSR,ICR,ISTP,ISR;

/****************************************************************************/
/*                                                                          */
/*              全局变量                                                    */
/*                                                                          */
/****************************************************************************/
unsigned char Flag=0;

/****************************************************************************/
/*                                                                          */
/*              函数声明                                                    */
/*                                                                          */
/****************************************************************************/
// 外设使能配置
void PSCInit(void);

// GPIO 管脚复用配置
void GPIOBankPinMuxSet();
// GPIO 管脚初始化
void GPIOBankPinInit();
// GPIO 管脚中断初始化
void GPIOBankPinInterruptInit(void);

// DSP 中断初始化
void InterruptInit(void);
// 中断服务函数
void USER0KEYIsr(void);
void USER1KEYIsr(void);

/****************************************************************************/
/*                                                                          */
/*              主函数                                                      */
/*                                                                          */
/****************************************************************************/
int main(void)
{
	// 外设使能配置
	PSCInit();
	
	// GPIO 管脚复用配置
	GPIOBankPinMuxSet();

	// GPIO 管脚初始化
	GPIOBankPinInit();

	// DSP 中断初始化
	InterruptInit();

	// GPIO 管脚中断初始化
	GPIOBankPinInterruptInit();

	unsigned int i;

	// 主循环
	for(;;)
	{
		// 亦可以使用查询法查询中断状态

		if(Flag)
		{
			// 核心板 LED
			for(i=0x00FFFFFF;i>0;i--);                          // 延时
			GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
			GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);

			for(i=0x00FFFFFF;i>0;i--);                          // 延时
			GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
			GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
		}
	}
}

/****************************************************************************/
/*                                                                          */
/*              PSC 初始化                                                  */
/*                                                                          */
/****************************************************************************/
void PSCInit(void)
{
	// 使能 GPIO 模块
	// 对相应外设模块的使能也可以在 BootLoader 中完成
    PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_GPIO, PSC_POWERDOMAIN_ALWAYS_ON, PSC_MDCTL_NEXT_ENABLE);
}

/****************************************************************************/
/*                                                                          */
/*              GPIO 管脚复用配置                                           */
/*                                                                          */
/****************************************************************************/
void GPIOBankPinMuxSet(void)
{
	// 配置相应的 GPIO 口功能为普通输入输出口
	// 核心板 LED
	GPIOBank6Pin12PinMuxSetup();
	GPIOBank6Pin13PinMuxSetup();

	// 底板按键
	GPIOBank0Pin6PinMuxSetup();
	GPIOBank6Pin1PinMuxSetup();
}

/****************************************************************************/
/*                                                                          */
/*              GPIO 管脚初始化                                             */
/*                                                                          */
/****************************************************************************/
void GPIOBankPinInit(void)
{
	// 配置 LED 对应管脚为输出管脚
    // OMAPL138 及 DSP C6748 共有 144 个 GPIO
	// 以下为各组 GPIO BANK 起始管脚对应值
    // 范围 1-144
	// GPIO0[0] 1
    // GPIO1[0] 17
	// GPIO2[0] 33
    // GPIO3[0] 49
	// GPIO4[0] 65
    // GPIO5[0] 81
	// GPIO6[0] 97
	// GPIO7[0] 113
	// GPIO8[0] 129

	// 核心板 LED
    GPIODirModeSet(SOC_GPIO_0_REGS, 109, GPIO_DIR_OUTPUT);  // GPIO6[12]
    GPIODirModeSet(SOC_GPIO_0_REGS, 110, GPIO_DIR_OUTPUT);  // GPIO6[13]

	// 底板按键
    GPIODirModeSet(SOC_GPIO_0_REGS, 7, GPIO_DIR_INPUT);     // USER0 KEY GPIO0[6]
    GPIODirModeSet(SOC_GPIO_0_REGS, 98, GPIO_DIR_INPUT);    // USER1 KEY GPIO6[1]
}

/****************************************************************************/
/*                                                                          */
/*              GPIO 管脚中断初始化                                         */
/*                                                                          */
/****************************************************************************/
void GPIOBankPinInterruptInit(void)
{
    // 底板按键中断
    // 配置 USER0 KEY GPIO0[6] 为下降沿触发
    GPIOIntTypeSet(SOC_GPIO_0_REGS, 7, GPIO_INT_TYPE_FALLEDGE);
    // 配置 USER1 KEY GPIO6[1] 为上升沿及下降沿触发
    GPIOIntTypeSet(SOC_GPIO_0_REGS, 98, GPIO_INT_TYPE_BOTHEDGE);

    // 使能 GPIO BANK 中断
    GPIOBankIntEnable(SOC_GPIO_0_REGS, 0);                  // USER0 KEY GPIO0
    GPIOBankIntEnable(SOC_GPIO_0_REGS, 6);                  // USER1 KEY GPIO6

	// 注册中断服务函数
	IntRegister(C674X_MASK_INT4, USER0KEYIsr);
	IntRegister(C674X_MASK_INT5, USER1KEYIsr);

	// 映射中断到 DSP 可屏蔽中断
	IntEventMap(C674X_MASK_INT4, SYS_INT_GPIO_B0INT);
	IntEventMap(C674X_MASK_INT5, SYS_INT_GPIO_B6INT);

	// 使能 DSP 可屏蔽中断
	IntEnable(C674X_MASK_INT4);
	IntEnable(C674X_MASK_INT5);
}

/****************************************************************************/
/*                                                                          */
/*              DSP 中断初始化                                              */
/*                                                                          */
/****************************************************************************/
void InterruptInit(void)
{
	// 初始化 DSP 中断控制器
	IntDSPINTCInit();

	// 使能 DSP 全局中断
	IntGlobalEnable();
}

/****************************************************************************/
/*                                                                          */
/*              中断服务函数                                                */
/*                                                                          */
/****************************************************************************/
void USER0KEYIsr(void)
{
	// 软件断点 方便调试
	//SW_BREAKPOINT;

    // 禁用 GPIO BANK 0 中断
    GPIOBankIntDisable(SOC_GPIO_0_REGS, 0);

    // 清除 GPIO BANK 0 中断状态
    IntEventClear(SYS_INT_GPIO_B0INT);

    // 清除 GPIO0[6] 中断状态
    GPIOPinIntClear(SOC_GPIO_0_REGS, 7);

    Flag=0;
	
	// 使能 GPIO BANK 0 中断
    GPIOBankIntEnable(SOC_GPIO_0_REGS, 0);
}

void USER1KEYIsr(void)
{
	// 软件断点 方便调试
	//SW_BREAKPOINT;

    // 禁用 GPIO BANK 6 中断
    GPIOBankIntDisable(SOC_GPIO_0_REGS, 6);

    // 清除 GPIO BANK 6 中断状态
    IntEventClear(SYS_INT_GPIO_B6INT);

    // 清除 GPIO6[1] 中断状态
    GPIOPinIntClear(SOC_GPIO_0_REGS, 98);

    Flag=1;
	
	// 使能 GPIO BANK 0 中断
    GPIOBankIntEnable(SOC_GPIO_0_REGS, 6);
}

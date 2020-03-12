/****************************************************************************/
/*                                                                          */
/*              ���ݴ������ӿƼ����޹�˾                                    */
/*                                                                          */
/*              Copyright 2014 Tronlong All rights reserved                 */
/*                                                                          */
/****************************************************************************/
/****************************************************************************/
/*                                                                          */
/*              �����жϲ���                                                */
/*                                                                          */
/*              2014��4��20��                                               */
/*                                                                          */
/****************************************************************************/
#include "TL6748.h"                 // ���� DSP6748 �������������

#include "hw_types.h"               // ������
#include "hw_syscfg0_C6748.h"       // ϵͳ����ģ��Ĵ���
#include "soc_C6748.h"              // DSP C6748 ����Ĵ���

#include "psc.h"                    // ��Դ��˯�߿��ƺ꼰�豸����㺯������
#include "gpio.h"                   // ͨ����������ں꼰�豸����㺯������

#include "interrupt.h"              // DSP C6748 �ж����Ӧ�ó���ӿں���������ϵͳ�¼��Ŷ���

/****************************************************************************/
/*                                                                          */
/*              �궨��                                                      */
/*                                                                          */
/****************************************************************************/
// ����ϵ�
#define SW_BREAKPOINT   asm(" SWBP 0 ");

// �ж���ؼĴ���
cregister unsigned int IER,IFR,CSR,ICR,ISTP,ISR;

/****************************************************************************/
/*                                                                          */
/*              ȫ�ֱ���                                                    */
/*                                                                          */
/****************************************************************************/
unsigned char Flag=0;

/****************************************************************************/
/*                                                                          */
/*              ��������                                                    */
/*                                                                          */
/****************************************************************************/
// ����ʹ������
void PSCInit(void);

// GPIO �ܽŸ�������
void GPIOBankPinMuxSet();
// GPIO �ܽų�ʼ��
void GPIOBankPinInit();
// GPIO �ܽ��жϳ�ʼ��
void GPIOBankPinInterruptInit(void);

// DSP �жϳ�ʼ��
void InterruptInit(void);
// �жϷ�����
void USER0KEYIsr(void);
void USER1KEYIsr(void);

/****************************************************************************/
/*                                                                          */
/*              ������                                                      */
/*                                                                          */
/****************************************************************************/
int main(void)
{
	// ����ʹ������
	PSCInit();
	
	// GPIO �ܽŸ�������
	GPIOBankPinMuxSet();

	// GPIO �ܽų�ʼ��
	GPIOBankPinInit();

	// DSP �жϳ�ʼ��
	InterruptInit();

	// GPIO �ܽ��жϳ�ʼ��
	GPIOBankPinInterruptInit();

	unsigned int i;

	// ��ѭ��
	for(;;)
	{
		// �����ʹ�ò�ѯ����ѯ�ж�״̬

		if(Flag)
		{
			// ���İ� LED
			for(i=0x00FFFFFF;i>0;i--);                          // ��ʱ
			GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
			GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);

			for(i=0x00FFFFFF;i>0;i--);                          // ��ʱ
			GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
			GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
		}
	}
}

/****************************************************************************/
/*                                                                          */
/*              PSC ��ʼ��                                                  */
/*                                                                          */
/****************************************************************************/
void PSCInit(void)
{
	// ʹ�� GPIO ģ��
	// ����Ӧ����ģ���ʹ��Ҳ������ BootLoader �����
    PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_GPIO, PSC_POWERDOMAIN_ALWAYS_ON, PSC_MDCTL_NEXT_ENABLE);
}

/****************************************************************************/
/*                                                                          */
/*              GPIO �ܽŸ�������                                           */
/*                                                                          */
/****************************************************************************/
void GPIOBankPinMuxSet(void)
{
	// ������Ӧ�� GPIO �ڹ���Ϊ��ͨ���������
	// ���İ� LED
	GPIOBank6Pin12PinMuxSetup();
	GPIOBank6Pin13PinMuxSetup();

	// �װ尴��
	GPIOBank0Pin6PinMuxSetup();
	GPIOBank6Pin1PinMuxSetup();
}

/****************************************************************************/
/*                                                                          */
/*              GPIO �ܽų�ʼ��                                             */
/*                                                                          */
/****************************************************************************/
void GPIOBankPinInit(void)
{
	// ���� LED ��Ӧ�ܽ�Ϊ����ܽ�
    // OMAPL138 �� DSP C6748 ���� 144 �� GPIO
	// ����Ϊ���� GPIO BANK ��ʼ�ܽŶ�Ӧֵ
    // ��Χ 1-144
	// GPIO0[0] 1
    // GPIO1[0] 17
	// GPIO2[0] 33
    // GPIO3[0] 49
	// GPIO4[0] 65
    // GPIO5[0] 81
	// GPIO6[0] 97
	// GPIO7[0] 113
	// GPIO8[0] 129

	// ���İ� LED
    GPIODirModeSet(SOC_GPIO_0_REGS, 109, GPIO_DIR_OUTPUT);  // GPIO6[12]
    GPIODirModeSet(SOC_GPIO_0_REGS, 110, GPIO_DIR_OUTPUT);  // GPIO6[13]

	// �װ尴��
    GPIODirModeSet(SOC_GPIO_0_REGS, 7, GPIO_DIR_INPUT);     // USER0 KEY GPIO0[6]
    GPIODirModeSet(SOC_GPIO_0_REGS, 98, GPIO_DIR_INPUT);    // USER1 KEY GPIO6[1]
}

/****************************************************************************/
/*                                                                          */
/*              GPIO �ܽ��жϳ�ʼ��                                         */
/*                                                                          */
/****************************************************************************/
void GPIOBankPinInterruptInit(void)
{
    // �װ尴���ж�
    // ���� USER0 KEY GPIO0[6] Ϊ�½��ش���
    GPIOIntTypeSet(SOC_GPIO_0_REGS, 7, GPIO_INT_TYPE_FALLEDGE);
    // ���� USER1 KEY GPIO6[1] Ϊ�����ؼ��½��ش���
    GPIOIntTypeSet(SOC_GPIO_0_REGS, 98, GPIO_INT_TYPE_BOTHEDGE);

    // ʹ�� GPIO BANK �ж�
    GPIOBankIntEnable(SOC_GPIO_0_REGS, 0);                  // USER0 KEY GPIO0
    GPIOBankIntEnable(SOC_GPIO_0_REGS, 6);                  // USER1 KEY GPIO6

	// ע���жϷ�����
	IntRegister(C674X_MASK_INT4, USER0KEYIsr);
	IntRegister(C674X_MASK_INT5, USER1KEYIsr);

	// ӳ���жϵ� DSP �������ж�
	IntEventMap(C674X_MASK_INT4, SYS_INT_GPIO_B0INT);
	IntEventMap(C674X_MASK_INT5, SYS_INT_GPIO_B6INT);

	// ʹ�� DSP �������ж�
	IntEnable(C674X_MASK_INT4);
	IntEnable(C674X_MASK_INT5);
}

/****************************************************************************/
/*                                                                          */
/*              DSP �жϳ�ʼ��                                              */
/*                                                                          */
/****************************************************************************/
void InterruptInit(void)
{
	// ��ʼ�� DSP �жϿ�����
	IntDSPINTCInit();

	// ʹ�� DSP ȫ���ж�
	IntGlobalEnable();
}

/****************************************************************************/
/*                                                                          */
/*              �жϷ�����                                                */
/*                                                                          */
/****************************************************************************/
void USER0KEYIsr(void)
{
	// ����ϵ� �������
	//SW_BREAKPOINT;

    // ���� GPIO BANK 0 �ж�
    GPIOBankIntDisable(SOC_GPIO_0_REGS, 0);

    // ��� GPIO BANK 0 �ж�״̬
    IntEventClear(SYS_INT_GPIO_B0INT);

    // ��� GPIO0[6] �ж�״̬
    GPIOPinIntClear(SOC_GPIO_0_REGS, 7);

    Flag=0;
	
	// ʹ�� GPIO BANK 0 �ж�
    GPIOBankIntEnable(SOC_GPIO_0_REGS, 0);
}

void USER1KEYIsr(void)
{
	// ����ϵ� �������
	//SW_BREAKPOINT;

    // ���� GPIO BANK 6 �ж�
    GPIOBankIntDisable(SOC_GPIO_0_REGS, 6);

    // ��� GPIO BANK 6 �ж�״̬
    IntEventClear(SYS_INT_GPIO_B6INT);

    // ��� GPIO6[1] �ж�״̬
    GPIOPinIntClear(SOC_GPIO_0_REGS, 98);

    Flag=1;
	
	// ʹ�� GPIO BANK 0 �ж�
    GPIOBankIntEnable(SOC_GPIO_0_REGS, 6);
}

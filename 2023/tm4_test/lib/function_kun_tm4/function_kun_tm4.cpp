#include "function_kun_tm4.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/tm4c123gh6pm.h"				//Register Definitions
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
// #include "uartstdio.h"
#include "driverlib/systick.h"
#include "driverlib/pin_map.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_qei.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/qei.h"


// char rxBuffer[256];

// void UARTIntHandler(void)
// {
// 	uint32_t ui32Status;
// 	ui32Status = UARTIntStatus(UART1_BASE, true); //get interrupt status
// 	static int32_t cnt;
// 	UARTIntClear(UART1_BASE, ui32Status); 				//clear the asserted interrupts
// 	while(UARTCharsAvail(UART1_BASE)) 						//loop while there are chars
// 	{
// 		rxBuffer[cnt++] = UARTCharGetNonBlocking(UART1_BASE);
// 		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); //blink LED
// 		delay_ms(0.1);
// 		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0); //turn off LED
// 	}
// 	cnt = 0;
// 	UARTprintf("%s\r\n", rxBuffer);
// 	memset(rxBuffer, 0x00, sizeof(rxBuffer));
// }

// void QEI_Init(void)
// {
//     const uint32_t MaxPos = 4000;

// 	// 使能外设
// 	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
// 	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0)){}
// 	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
// 	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI1));

// 	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
// 	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
// 	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

// 	// Unlock PD7
// 	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
// 	HWREG(GPIO_PORTD_BASE + GPIO_O_CR)  |= 0x80;
// 	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0x00;

// 	// QEI1: PC5, PC6
// 	GPIOPinConfigure(GPIO_PC5_PHA1);
// 	GPIOPinTypeQEI(GPIO_PORTC_BASE , GPIO_PIN_5);
// 	GPIOPinConfigure(GPIO_PC6_PHB1);
// 	GPIOPinTypeQEI(GPIO_PORTC_BASE , GPIO_PIN_6);
// 	QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_RESET_IDX |QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), MaxPos - 1);
// 	QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1, SysCtlClockGet()/100);
// 	QEIVelocityEnable(QEI1_BASE);
// 	QEIEnable(QEI1_BASE);
// 	// QEI2: PD6, PD7
// 	GPIOPinConfigure(GPIO_PD6_PHA0);
// 	GPIOPinTypeQEI(GPIO_PORTD_BASE , GPIO_PIN_6);
// 	GPIOPinConfigure(GPIO_PD7_PHB0);
// 	GPIOPinTypeQEI(GPIO_PORTD_BASE , GPIO_PIN_7);
// 	QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_RESET_IDX |QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), MaxPos - 1);
// 	QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, SysCtlClockGet()/100);
// 	QEIVelocityEnable(QEI0_BASE);
// 	QEIEnable(QEI0_BASE);
// }

void function_kun_tm4:: UART1_Init()
{
	//使能外设
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	//配置复用功能
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	//分配UART信号
	GPIOPinTypeUART(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1);
	//串口参数设置
	UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);	//使用16MHz内部高精度振荡器(PIOSC)作为UART模块时钟
	// UARTStdioConfig(1,115200, 16000000);				//UART编号、波特率、UART时钟频率（频率要和上一行设的一致）
	//FIFO配置，
	UARTFIFOLevelSet(UART1_BASE,UART_FIFO_TX4_8,UART_FIFO_RX2_8);	//FIFO填入4Byte时触发中断
	UARTFIFOEnable(UART1_BASE);
	//中断使能
	IntMasterEnable(); 			//enable processor interrupts
	IntEnable(INT_UART1); 		//enable the UART interrupt
	UARTIntEnable(UART1_BASE, UART_INT_RX);
	//注册中断服务函数
	UARTIntRegister(UART1_BASE,UARTIntHandler);

    UARTEnable(UART1_BASE);
}

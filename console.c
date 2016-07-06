#include <stdio.h>

#include "define.h"
#include "console.h"
#include "FreeRTOS.h"
#include "task.h"

#define xDelay10  ((TickType_t)10 / portTICK_PERIOD_MS)
#define xDelay100  ((TickType_t)100 / portTICK_PERIOD_MS)

#define IER_RBR         (0x01<<0)
#define IER_THRE        (0x01<<1)
#define IER_RLS         (0x01<<2)
#define IER_ABEO        (0x01<<8)
#define IER_ABTO        (0x01<<9)

#define IIR_PEND        0x01
#define IIR_RLS         0x03
#define IIR_RDA         0x02
#define IIR_CTI         0x06
#define IIR_THRE        0x01
#define IIR_ABEO        (0x01<<8)
#define IIR_ABTO        (0x01<<9)

#define LSR_RDR         (0x01<<0)
#define LSR_OE          (0x01<<1)
#define LSR_PE          (0x01<<2)
#define LSR_FE          (0x01<<3)
#define LSR_BI          (0x01<<4)
#define LSR_THRE        (0x01<<5)
#define LSR_TEMT        (0x01<<6)
#define LSR_RXFE        (0x01<<7)

xQueueHandle consoleTxQueue;
xQueueHandle consoleRxQueue;

/* UART transmit-only interrupt handler for ring buffers */
void UART1_TXIntHandler()
{
	BaseType_t result = pdFAIL;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	uint8_t ch;

	/* Fill FIFO until full or until TX ring buffer is empty */
	while ((LPC_UART1->LSR & LSR_THRE) && ((result = xQueueReceiveFromISR(consoleTxQueue, &ch, &xHigherPriorityTaskWoken)) == pdPASS))
		LPC_UART1->THR = ch;
	if (result != pdPASS)
		LPC_UART1->IER &= ~IER_THRE;	/* Disable UART THRE interrupt */		
}

void UART1_IRQHandler(void)
{
  uint8_t IIRValue, LSRValue;
  uint8_t ch = ch;

	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  IIRValue = LPC_UART1->IIR;
  IIRValue >>= 1;            /* skip pending bit in IIR */
  IIRValue &= 0x07;            /* check bit 1~3, interrupt identification */

  if ( IIRValue == IIR_RLS )        /* Receive Line Status */
  {
    LSRValue = LPC_UART1->LSR;
    /* Receive Line Status */
    if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) )
    {
      /* There are errors or break interrupt */
      /* Read LSR will clear the interrupt */
      ch = LPC_UART1->RBR;        /* Dummy read on RX to clear interrupt, then bail out */
      return;
    }
    if ( LSRValue & LSR_RDR )    /* Receive Data Ready */
    {
      /* If no error on RLS, normal ready, save into the data buffer. */
      /* Note: read RBR will clear the interrupt */
			ch = LPC_UART1->RBR;
			xQueueSendFromISR(consoleRxQueue, &ch, &xHigherPriorityTaskWoken);
    }
  }
  else if ( IIRValue == IIR_RDA )    /* Receive Data Available */
  {
    /* Receive Data Available */
		ch = LPC_UART1->RBR;
		xQueueSendFromISR(consoleRxQueue, &ch, &xHigherPriorityTaskWoken);
  }
  else if ( IIRValue == IIR_THRE )    /* THRE, transmit holding register empty */
  {
    UART1_TXIntHandler();
  }

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	return;
}

void task_Console(void *pvParameters)
{
  uint32_t Fdiv;
  uint32_t pclkdiv, pclk;

  consoleTxQueue = xQueueCreate(256, sizeof(char));
  consoleRxQueue = xQueueCreate(128, sizeof(char));

	LPC_SC->PCONP |= (PCONP_PCGPIO | PCONP_PCUART1); // power up GPIO and UART1

	LPC_GPIO0->FIODIR |= (1<<15);							// pin 5 auf JST GHR-08V-S
	LPC_GPIO0->FIODIR &= ~(1<<16);						// pin 2 auf JST GHR-08V-S

	LPC_PINCON->PINSEL0 &= ~((uint32_t)3 << 30);
	LPC_PINCON->PINSEL0 |= ((uint32_t)1 << 30);	// TXD1
	LPC_PINCON->PINSEL1 &= ~(3 << 0);
	LPC_PINCON->PINSEL1 |= (1 << 0);						// RXD1

	/* By default, the PCLKSELx value is zero, thus, the PCLK for
	all the peripherals is 1/4 of the SystemFrequency. */
	/* Bit 8,9 are for UART1 */
	pclkdiv = (LPC_SC->PCLKSEL0 >> 8) & 0x03;
	switch ( pclkdiv )
	{
		case 0x00:
		default:
			pclk = SystemCoreClock/4;
			break;
		case 0x01:
			pclk = SystemCoreClock;
			break;
		case 0x02:
			pclk = SystemCoreClock/2;
			break;
		case 0x03:
			pclk = SystemCoreClock/8;
			break;
	}

	LPC_UART1->LCR = 0x83;        /* 8 bits, no Parity, 1 Stop bit */
	Fdiv = ( pclk / 16 ) / 115200 ;    /*baud rate */
	LPC_UART1->DLM = Fdiv / 256;
	LPC_UART1->DLL = Fdiv % 256;
	LPC_UART1->LCR = 0x03;        /* DLAB = 0 */
	LPC_UART1->FCR = 0x07;        /* Enable and reset TX and RX FIFO. */

  LPC_UART1->IER = IER_RBR | IER_RLS;    /* Enable UART1 interrupts */

	portENTER_CRITICAL();
	NVIC_SetPriority(UART1_IRQn, 5);
	NVIC_EnableIRQ(UART1_IRQn);
	portEXIT_CRITICAL();

	printf("Project Landlord Console\r\n");
	
	for (;;)
	{
		uint8_t ch;
		
		while (xQueueReceive(consoleRxQueue, &ch, (TickType_t)xDelay10) == pdPASS)
			xQueueSend(consoleTxQueue, (char*)&ch, (TickType_t)0);
	}
}

#pragma import(__use_no_semihosting_swi)

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f)
{
	BaseType_t result = pdPASS;
	LPC_UART1->IER &= ~IER_THRE;	/* Disable UART THRE interrupt */
	result = xQueueSend(consoleTxQueue, (char*)&ch, (TickType_t)0);
	UART1_TXIntHandler();
	LPC_UART1->IER |= IER_THRE;
	if (result == pdPASS)
		return (ch);
  return -1;
}

void _ttywrch(int ch)
{
	LPC_UART1->IER &= ~IER_THRE;	/* Disable UART THRE interrupt */
	xQueueSend(consoleTxQueue, (char*)&ch, (TickType_t)0);
	UART1_TXIntHandler();
	LPC_UART1->IER |= IER_THRE;
	return;
}

int fgetc(FILE *f)
{
	char _ch;
	if (xQueueReceive(consoleRxQueue, &_ch, (TickType_t)0) != pdPASS)
		return -1;
	else
		return _ch;
}

void _sys_exit(int return_code)
{
label:  goto label;  /* endless loop */
}

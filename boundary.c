#include "boundary.h"
#include "common.h"
#include "FreeRTOS.h"

// Something fishy going on with include files, compiles in wrong order?? Does not load xxx_hal.h 
// boundary.c:86:18: error: 'BOUNDARY_TX' undeclared (first use in this function); did you mean 'BOUNDARY_H'?

#define BOUNDARY_TX        (GPIO_TYPE(PORT_0, PIN_10, FUNC_1))
#define BOUNDARY_RX        (GPIO_TYPE(PORT_0, PIN_11, FUNC_1))
#define BOUNDARY_ENABLE    (GPIO_TYPE(PORT_4, PIN_16, FUNC_0))

#define buflen 128
volatile char txbuf[buflen];
volatile uint8_t txpos;

void UART2_IRQHandler(void) {
	volatile uint8_t ch;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	while ( LPC_UART2->LSR & (1<<0) ) { // while Receive FIFO Not Empty
		ch = LPC_UART2->RBR;
		if (ch == 0x56) { // Ascii "V"
			if (txpos == 36) {
				txbuf[txpos++] = 0;
				xQueueSendFromISR(xBoundaryMsgQueue, &txbuf, &xHigherPriorityTaskWoken);
			}
			txpos=0;
		}
		if (txpos >= sizeof(txbuf)) txpos = 0;
		txbuf[txpos++] = ch;
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	return;
}


void boundary_Task(void *pvParameters) {
    //boundary_Init();

    LPC_SC->PCONP |= PCONP_PCUART2;

    GPIO_PIN_FNC(BOUNDARY_TX);
    GPIO_PIN_FNC(BOUNDARY_RX);
// Behöver vi sätta direction manuellt? tror inte det
	GPIO_DIR_OUT(BOUNDARY_TX);
	GPIO_DIR_IN(BOUNDARY_RX);

	// Exit stm32 flash/sleep mode(?)
	GPIO_DIR_OUT(BOUNDARY_ENABLE);
	GPIO_SET_PIN(BOUNDARY_ENABLE); 

    // Baud 115132
    LPC_UART2->LCR = (3<<0) | (1 << 7); // 8-bit character length || Enable access to Divisor Latches.
    LPC_UART2->DLM = 0; // (Fdiv >> 0x08) & 0xFF;
	LPC_UART2->DLL = 19; // Fdiv & 0xFF;
    LPC_UART2->FDR = (5 << 0) | (7 << 4); // MULVAL || DIVADDVAL
	LPC_UART2->FCR = 1; // FIFO _needs_ to be enabled for interrupts to work correctly. 
	LPC_UART2->LCR = (3<<0); // 8-bit character length

    // https://www.keil.com/dd/docs/datashts/philips/lpc177x_lpc178x_um.pdf page 460
	portENTER_CRITICAL();
	NVIC_EnableIRQ(UART2_IRQn);
	NVIC_SetPriority(UART2_IRQn, 5);
	portEXIT_CRITICAL();
    //HAL_NVIC_EnableIRQ(USART2_IRQn);
	
    LPC_UART2->IER = (1 << 2) | (1 << 0); // Enable the RX line status interrupts. || Enable the RDA interrupts.

 /*   for (;;) {
		vTaskDelay(xDelay1000);
    }
*/
	vTaskDelete( NULL );
}
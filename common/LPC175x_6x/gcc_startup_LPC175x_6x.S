/* File: startup_lpc11xx.S, derived from startup_ARMCM0.S
 * Purpose: startup file for Cortex-M0 devices. Should use with
 *   GCC for ARM Embedded Processors
 * Version: V1.4
 * Date: 09 July 2012
 *
 * Copyright (c) 2011, 2012, ARM Limited
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the ARM Limited nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ARM LIMITED BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
    .syntax unified
    .arch armv6-m

    .section .stack
    .align 3
#ifdef __STACK_SIZE
    .equ    Stack_Size, __STACK_SIZE
#else
    .equ    Stack_Size, 0xc00
#endif
    .globl    __StackTop
    .globl    __StackLimit
__StackLimit:
    .space    Stack_Size
    .size __StackLimit, . - __StackLimit
__StackTop:
    .size __StackTop, . - __StackTop

    .section .heap
    .align 3
#ifdef __HEAP_SIZE
    .equ    Heap_Size, __HEAP_SIZE
#else
    .equ    Heap_Size, 0
#endif
    .globl    __HeapBase
    .globl    __HeapLimit
__HeapBase:
    .if    Heap_Size
    .space    Heap_Size
    .endif
    .size __HeapBase, . - __HeapBase
__HeapLimit:
    .size __HeapLimit, . - __HeapLimit

    .section .isr_vector
    .align 2
    .globl __isr_vector
__isr_vector:
    .long    __StackTop            /* Top of Stack */
    .long    Reset_Handler         /* Reset Handler */
    .long    NMI_Handler           /* NMI Handler */
    .long    HardFault_Handler     /* Hard Fault Handler */
    .long    MemManage_Handler     /* MPU Fault Handler */
    .long    BusFault_Handler      /* Bus Fault Handler */
    .long    UsageFault_Handler    /* Usage Fault Handler  */
    .long    __valid_user_code_checksum     /* Checksum of vector 0-6, Calculated in lpc17xx.ld*/
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    SVC_Handler           /* SVCall Handler */
    .long    DebugMon_Handler      /* Debug Monitor Handler */
    .long    0                     /* Reserved */
    .long    PendSV_Handler        /* PendSV Handler */
    .long    SysTick_Handler       /* SysTick Handler */

    /* External Interrupts                IRQ: */
    .long   WDT_IRQHandler              /*  0: Watchdog Timer               */
    .long   TIMER0_IRQHandler           /*  1: Timer0                       */
    .long   TIMER1_IRQHandler           /*  2: Timer1                       */
    .long   TIMER2_IRQHandler           /*  3: Timer2                       */
    .long   TIMER3_IRQHandler           /*  4: Timer3                       */
    .long   UART0_IRQHandler            /*  5: UART0                        */
    .long   UART1_IRQHandler            /*  6: UART1                        */
    .long   UART2_IRQHandler            /*  7: UART2                        */
    .long   UART3_IRQHandler            /*  8: UART3                        */
    .long   PWM1_IRQHandler             /*  9: PWM1                         */
    .long   I2C0_IRQHandler             /* 10: I2C0                         */
    .long   I2C1_IRQHandler             /* 11: I2C1                         */
    .long   I2C2_IRQHandler             /* 12: I2C2                         */
    .long   SPI_IRQHandler              /* 13: SPI                          */
    .long   SSP0_IRQHandler             /* 14: SSP0                         */
    .long   SSP1_IRQHandler             /* 15: SSP1                         */
    .long   PLL0_IRQHandler             /* 16: PLL0 Lock (Main PLL)         */
    .long   RTC_IRQHandler              /* 17: Real Time Clock              */
    .long   EINT0_IRQHandler            /* 18: External Interrupt 0         */
    .long   EINT1_IRQHandler            /* 19: External Interrupt 1         */
    .long   EINT2_IRQHandler            /* 20: External Interrupt 2         */
    .long   EINT3_IRQHandler            /* 21: External Interrupt 3         */
    .long   ADC_IRQHandler              /* 22: A/D Converter                */
    .long   BOD_IRQHandler              /* 23: Brown-Out Detect             */
    .long   USB_IRQHandler              /* 24: USB                          */
    .long   CAN_IRQHandler              /* 25: CAN                          */
    .long   DMA_IRQHandler              /* 26: General Purpose DMA          */
    .long   I2S_IRQHandler              /* 27: I2S                          */
    .long   ENET_IRQHandler             /* 28: Ethernet                     */
    .long   RIT_IRQHandler              /* 29: Repetitive Interrupt Timer   */
    .long   MCPWM_IRQHandler            /* 30: Motor Control PWM            */
    .long   QEI_IRQHandler              /* 31: Quadrature Encoder Interface */
    .long   PLL1_IRQHandler             /* 32: PLL1 Lock (USB PLL)          */
    .long   USBActivity_IRQHandler      /* 33: USB Activity                 */
    .long   CANActivity_IRQHandler      /* 34: CAN Activity                 */

    .size    __isr_vector, . - __isr_vector

    .section .text.Reset_Handler
    .thumb
    .thumb_func
    .align 2
    .globl    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
/*     Loop to copy data from read only memory to RAM. The ranges
 *      of copy from/to are specified by following symbols evaluated in
 *      linker script.
 *      __etext: End of code section, i.e., begin of data sections to copy from.
 *      __data_start__/__data_end__: RAM address range that data should be
 *      copied to. Both must be aligned to 4 bytes boundary.  */

    ldr    r1, =__etext
    ldr    r2, =__data_start__
    ldr    r3, =__data_end__

    subs    r3, r2
    ble    .Lflash_to_ram_loop_end

    movs    r4, 0
.Lflash_to_ram_loop:
    ldr    r0, [r1,r4]
    str    r0, [r2,r4]
    adds    r4, 4
    cmp    r4, r3
    blt    .Lflash_to_ram_loop
.Lflash_to_ram_loop_end:

    ldr    r0, =SystemInit
    blx    r0
    ldr    r0, =_start
    bx    r0
    .pool
    .size Reset_Handler, . - Reset_Handler

    .text
/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro  IRQ handler
    .align 1
    .thumb_func
    .weak   \handler
    .type    \handler, %function
\handler :
/*    b    .
    .size    \handler, . - \handler*/
    .endm

    IRQ    NMI_Handler
    IRQ    HardFault_Handler
    IRQ    MemManage_Handler
    IRQ    BusFault_Handler
    IRQ    UsageFault_Handler
    IRQ    DebugMon_Handler
    IRQ    SVC_Handler
    IRQ    PendSV_Handler
    IRQ    SysTick_Handler

    IRQ     WDT_IRQHandler
    IRQ     TIMER0_IRQHandler
    IRQ     TIMER1_IRQHandler
    IRQ     TIMER2_IRQHandler
    IRQ     TIMER3_IRQHandler
    IRQ     UART0_IRQHandler
    IRQ     UART1_IRQHandler
    IRQ     UART2_IRQHandler
    IRQ     UART3_IRQHandler
    IRQ     PWM1_IRQHandler
    IRQ     I2C0_IRQHandler
    IRQ     I2C1_IRQHandler
    IRQ     I2C2_IRQHandler
    IRQ     SPI_IRQHandler
    IRQ     SSP0_IRQHandler
    IRQ     SSP1_IRQHandler
    IRQ     PLL0_IRQHandler
    IRQ     RTC_IRQHandler
    IRQ     EINT0_IRQHandler
    IRQ     EINT1_IRQHandler
    IRQ     EINT2_IRQHandler
    IRQ     EINT3_IRQHandler
    IRQ     ADC_IRQHandler
    IRQ     BOD_IRQHandler
    IRQ     USB_IRQHandler
    IRQ     CAN_IRQHandler
    IRQ     DMA_IRQHandler
    IRQ     I2S_IRQHandler
    IRQ     ENET_IRQHandler
    IRQ     RIT_IRQHandler
    IRQ     MCPWM_IRQHandler
    IRQ     QEI_IRQHandler
    IRQ     PLL1_IRQHandler
    IRQ         USBActivity_IRQHandler
    IRQ         CANActivity_IRQHandler

/* IRQ Handlers */

    .globl  Default_Handler
    .type   Default_Handler, %function

/*
Default_Handler:
    B       .
    .size   Default_Handler, . - Default_Handler
*/

Default_Handler:
  /* Load the address of the interrupt control register into r3. */
  ldr r3, NVIC_INT_CTRL_CONST
  /* Load the value of the interrupt control register into r2 from the address held in r3. */
  ldr r2, [r3, #0]
  /* The interrupt number is in the least significant byte - clear all other bits. */
  uxtb r2, r2
Infinite_Loop:
  /* Now sit in an infinite loop - the number of the executing interrupt is held in r2. */
  b  Infinite_Loop
  .size  Default_Handler, .-Default_Handler

.align 4
/* The address of the NVIC interrupt control register. */
NVIC_INT_CTRL_CONST: .word 0xe000ed04

    .end

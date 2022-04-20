/**************************************************************************/
/* Configure memory regions */
MEMORY
{
/* TODO: make if-statement here */
  /*FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 512k*/ /* Overwrites bootloader, not flashable over USB! */

  FLASH (rx) : ORIGIN = 0x00009000, LENGTH = 476k /* INCLUDES BOOTLOADER */
  /*RAM  (rwx) : ORIGIN = 0x10000000, LENGTH =  32k*/ /* NO SPACE FOR Interrupt Vector Table */

  RAM (rwx) : ORIGIN = 0x100000C8, LENGTH = (32K - 0xC8) /* Interrupt Vector Table in RAM 0xC8 length */
  
  USB_RAM(rwx) : ORIGIN = 0x2007C000, LENGTH = 16K
  ETH_RAM(rwx) : ORIGIN = 0x20080000, LENGTH = 16K
/*  CAN_RAM(rwx) : ORIGIN = 0x40038000, LENGTH = 2K*/

}

/* Config Libraries */
GROUP(libgcc.a libc.a libm.a libnosys.a)

/* Linker script to place sections and symbol values. Should be used together
 * with other linker script that defines memory regions FLASH and RAM.
 * It references following symbols, which must be defined in code:
 *   Reset_Handler : Entry of reset handler
 *
 * It defines following symbols, which code can use without definition:
 *   __exidx_start
 *   __exidx_end
 *   __etext
 *   __data_start__
 *   __preinit_array_start
 *   __preinit_array_end
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
 *   __data_end__
 *   __bss_start__
 *   __bss_end__
 *   __end__
 *   end
 *   __HeapLimit
 *   __StackLimit
 *   __StackTop
 *   __stack
 */
ENTRY(Reset_Handler)

SECTIONS
{
	.text :
	{
		KEEP(*(.isr_vector))
		*(.text*)

		KEEP(*(.init))
		KEEP(*(.fini))

		/* .ctors */
		*crtbegin.o(.ctors)
		*crtbegin?.o(.ctors)
		*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
		*(SORT(.ctors.*))
		*(.ctors)

		/* .dtors */
 		*crtbegin.o(.dtors)
 		*crtbegin?.o(.dtors)
 		*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
 		*(SORT(.dtors.*))
 		*(.dtors)

		*(.rodata*)

		KEEP(*(.eh_frame*))
	} > FLASH

	.ARM.extab :
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)
	} > FLASH

	__exidx_start = .;
	.ARM.exidx :
	{
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} > FLASH
	__exidx_end = .;

	__etext = .;

	.data : AT (__etext)
	{
		__data_start__ = .;
		*(vtable)
		*(.data*)

		. = ALIGN(4);
		/* preinit data */
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP(*(.preinit_array))
		PROVIDE_HIDDEN (__preinit_array_end = .);

		. = ALIGN(4);
		/* init data */
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP(*(SORT(.init_array.*)))
		KEEP(*(.init_array))
		PROVIDE_HIDDEN (__init_array_end = .);


		. = ALIGN(4);
		/* finit data */
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP(*(SORT(.fini_array.*)))
		KEEP(*(.fini_array))
		PROVIDE_HIDDEN (__fini_array_end = .);

		KEEP(*(.jcr*))
		. = ALIGN(4);
		/* All data end */
		__data_end__ = .;

	} > RAM

	.bss :
	{
		. = ALIGN(4);
		__bss_start__ = .;
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		__bss_end__ = .;
	} > RAM

	.heap (COPY):
	{
		__end__ = .;
		end = __end__;
		*(.heap*)
		__HeapLimit = .;
	} > RAM

	/* .stack_dummy section doesn't contains any symbols. It is only
	 * used for linker to calculate size of stack sections, and assign
	 * values to stack symbols later */
	.stack_dummy (COPY):
	{
		*(.stack*)
	} > RAM

	/* Set stack top to end of RAM, and stack limit move down by
	 * size of stack_dummy section */
	__StackTop = ORIGIN(RAM) + LENGTH(RAM);
	__StackLimit = __StackTop - SIZEOF(.stack_dummy);
	PROVIDE(__stack = __StackTop);

	/* Check if data + heap + stack exceeds RAM limit */
	ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")

	.AHBSRAM0 (NOLOAD):
    {
        Image$$RW_IRAM2$$Base = . ;
        *(AHBSRAM0)
        Image$$RW_IRAM2$$ZI$$Limit = .;
    } > USB_RAM

    .AHBSRAM1 (NOLOAD):
    {
        Image$$RW_IRAM3$$Base = . ;
        *(AHBSRAM1)
        Image$$RW_IRAM3$$ZI$$Limit = .;
    } > ETH_RAM

 /*   .CANSRAM0 (NOLOAD):
    {
        Image$$RW_IRAM4$$Base = . ;
        *(AHBSRAM1)
        Image$$RW_IRAM4$$ZI$$Limit = .;
    } > CAN_RAM */
    

}
/**************************************************************************/

/* Calculate __valid_user_code_checksum. */

PROVIDE(__valid_user_code_checksum = 0
             - (__StackTop 
             + (Reset_Handler + 1) 
             + (NMI_Handler + 1) 
             + (HardFault_Handler + 1)
             + ( DEFINED(MemManage_Handler) ? MemManage_Handler + 1 : 0 )   /* MemManage_Handler may not be defined */ 
             + ( DEFINED(BusFault_Handler) ? BusFault_Handler + 1 : 0 )     /* BusFault_Handler may not be defined */ 
             + ( DEFINED(UsageFault_Handler) ? UsageFault_Handler + 1 : 0 ) /* UsageFault_Handler may not be defined */ 
             ) );

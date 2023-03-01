ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
/* end of 10K RAM */
_estack = 0x20002800;

/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0;
_Min_Stack_Size = 0x200;

MEMORY {
	FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 32K
	RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 10K
}

/* Define output sections */
SECTIONS {
	.isr_vector : {
		. = ALIGN(4);
		KEEP(*(.isr_vector)) /* Startup code */
		. = ALIGN(4);
	} >FLASH

	.text : {
		. = ALIGN(4);
		*(.text .text*)
		*(.rodata .rodata*)
		*(.glue_7)
		*(.glue_7t)
		KEEP (*(.init))
		KEEP (*(.fini))
		. = ALIGN(4);
		_etext = .;
	} >FLASH


	/*
	.ARM.extab : {
		*(.ARM.extab* .gnu.linkonce.armextab.*)
	} >FLASH

	.ARM : {
		__exidx_start = .;
		*(.ARM.exidx*)
		__exidx_end = .;
	} >FLASH

	.ARM.attributes : { *(.ARM.attributes) } > FLASH
	*/

	_sidata = .;

	.data : AT ( _sidata ) {
		. = ALIGN(4);
		_sdata = .;
		*(.data .data*)
		. = ALIGN(4);
		_edata = .;
	} >RAM

	. = ALIGN(4);
	.bss : {
		_sbss = .;
		__bss_start__ = _sbss;
		*(.bss .bss*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
		__bss_end__ = _ebss;
	} >RAM

	PROVIDE ( end = _ebss );
	PROVIDE ( _end = _ebss );

	._user_heap_stack : {
		. = ALIGN(4);
		. = . + _Min_Heap_Size;
		. = . + _Min_Stack_Size;
		. = ALIGN(4);
	} >RAM

	/DISCARD/ : {
		libc.a ( * )
		libm.a ( * )
		libgcc.a ( * )
	}
}

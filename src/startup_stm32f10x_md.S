	.syntax unified
	.cpu cortex-m3
	.fpu softvfp
	.thumb

	.global g_pfnVectors
	.global Default_Handler

	.word _sidata
	.word _sdata
	.word _edata
	.word _sbss
	.word _ebss

	.equ BootRAM, 0xF108F85F

	.section .text.Reset_Handler
	.weak Reset_Handler
	.type Reset_Handler, %function

Reset_Handler:
	movs r1, #0
	b LoopCopyDataInit

CopyDataInit:
	ldr r3, =_sidata
	ldr r3, [r3, r1]
	str r3, [r0, r1]
	adds r1, r1, #4

LoopCopyDataInit:
	ldr r0, =_sdata
	ldr r3, =_edata
	adds r2, r0, r1
	cmp r2, r3
	bcc CopyDataInit
	ldr r2, =_sbss
	b LoopFillZerobss

FillZerobss:
	movs r3, #0
	str r3, [r2], #4

LoopFillZerobss:
	ldr r3, = _ebss
	cmp r2, r3
	bcc FillZerobss

	bl SystemInit
	bl main
	bx lr

	.size Reset_Handler, .-Reset_Handler

	.section .text.Default_Handler,"ax",%progbits

	.macro IRQ handler
	.word \handler
	.weak \handler
	.set \handler, Default_Handler
	.endm

Default_Handler:
Infinite_Loop:
	b Infinite_Loop
	.size Default_Handler, .-Default_Handler

	.section .isr_vector,"a",%progbits
	.type g_pfnVectors, %object
	.size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
	.word _estack
	.word Reset_Handler
	IRQ NMI_Handler
	IRQ HardFault_Handler
	IRQ MemManage_Handler
	IRQ BusFault_Handler
	IRQ UsageFault_Handler
	.word 0
	.word 0
	.word 0
	.word 0
	IRQ SVC_Handler
	IRQ DebugMon_Handler
	.word 0
	IRQ PendSV_Handler
	IRQ SysTick_Handler
	IRQ WWDG_IRQHandler
	IRQ PVD_IRQHandler
	IRQ TAMPER_IRQHandler
	IRQ RTC_IRQHandler
	IRQ FLASH_IRQHandler
	IRQ RCC_IRQHandler
	IRQ EXTI0_IRQHandler
	IRQ EXTI1_IRQHandler
	IRQ EXTI2_IRQHandler
	IRQ EXTI3_IRQHandler
	IRQ EXTI4_IRQHandler
	IRQ DMA1_Channel1_IRQHandler
	IRQ DMA1_Channel2_IRQHandler
	IRQ DMA1_Channel3_IRQHandler
	IRQ DMA1_Channel4_IRQHandler
	IRQ DMA1_Channel5_IRQHandler
	IRQ DMA1_Channel6_IRQHandler
	IRQ DMA1_Channel7_IRQHandler
	IRQ ADC1_2_IRQHandler
	IRQ USB_HP_CAN1_TX_IRQHandler
	IRQ USB_LP_CAN1_RX0_IRQHandler
	IRQ CAN1_RX1_IRQHandler
	IRQ CAN1_SCE_IRQHandler
	IRQ EXTI9_5_IRQHandler
	IRQ TIM1_BRK_IRQHandler
	IRQ TIM1_UP_IRQHandler
	IRQ TIM1_TRG_COM_IRQHandler
	IRQ TIM1_CC_IRQHandler
	IRQ TIM2_IRQHandler
	IRQ TIM3_IRQHandler
	IRQ TIM4_IRQHandler
	IRQ I2C1_EV_IRQHandler
	IRQ I2C1_ER_IRQHandler
	IRQ I2C2_EV_IRQHandler
	IRQ I2C2_ER_IRQHandler
	IRQ SPI1_IRQHandler
	IRQ SPI2_IRQHandler
	IRQ USART1_IRQHandler
	IRQ USART2_IRQHandler
	IRQ USART3_IRQHandler
	IRQ EXTI15_10_IRQHandler
	IRQ RTCAlarm_IRQHandler
	IRQ USBWakeUp_IRQHandler
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	/// @0x108. This is for boot in RAM mode for
	/// STM32F10x Medium Density devices.
	.word BootRAM


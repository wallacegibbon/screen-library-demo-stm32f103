#include "stm32f10x.h"

#if defined (STM32F10X_LD_VL) || (defined STM32F10X_MD_VL) || (defined STM32F10X_HD_VL)

#define SYSCLK_FREQ_24MHz 24000000

#else

// #define SYSCLK_FREQ_24MHz 24000000
// #define SYSCLK_FREQ_36MHz 36000000
// #define SYSCLK_FREQ_48MHz 48000000
// #define SYSCLK_FREQ_56MHz 56000000
#define SYSCLK_FREQ_72MHz 72000000

#endif

#if defined (STM32F10X_HD) || (defined STM32F10X_XL) || (defined STM32F10X_HD_VL)

// #define DATA_IN_ExtSRAM

#endif

/// Vector Table base offset field.  This value must be a multiple of 0x200.
#define VECT_TAB_OFFSET 0x0

///  Clock Definitions
#ifdef SYSCLK_FREQ_HSE
uint32_t SystemCoreClock = SYSCLK_FREQ_HSE;
#elif defined SYSCLK_FREQ_24MHz
uint32_t SystemCoreClock = SYSCLK_FREQ_24MHz;
#elif defined SYSCLK_FREQ_36MHz
uint32_t SystemCoreClock = SYSCLK_FREQ_36MHz;
#elif defined SYSCLK_FREQ_48MHz
uint32_t SystemCoreClock = SYSCLK_FREQ_48MHz;
#elif defined SYSCLK_FREQ_56MHz
uint32_t SystemCoreClock = SYSCLK_FREQ_56MHz;
#elif defined SYSCLK_FREQ_72MHz
uint32_t SystemCoreClock = SYSCLK_FREQ_72MHz;
#else
uint32_t SystemCoreClock = HSI_VALUE;
#endif

__I uint8_t AHBPrescTable[16] = {
	0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9
};

static void SetSysClock();

#ifdef SYSCLK_FREQ_HSE
static void SetSysClockToHSE();
#elif defined SYSCLK_FREQ_24MHz
static void SetSysClockTo24();
#elif defined SYSCLK_FREQ_36MHz
static void SetSysClockTo36();
#elif defined SYSCLK_FREQ_48MHz
static void SetSysClockTo48();
#elif defined SYSCLK_FREQ_56MHz
static void SetSysClockTo56();
#elif defined SYSCLK_FREQ_72MHz
static void SetSysClockTo72();
#endif

#ifdef DATA_IN_ExtSRAM
static void SystemInit_ExtMemCtl();
#endif

void SystemInit() {
	/// Reset the RCC clock configuration to the default reset state
	/// (for debug purpose)
	/// Set HSION bit
	RCC->CR |= 0x00000001;

	/// Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits
#ifndef STM32F10X_CL
	RCC->CFGR &= 0xF8FF0000;
#else
	RCC->CFGR &= 0xF0FF0000;
#endif

	/// Reset HSEON, CSSON and PLLON bits
	RCC->CR &= 0xFEF6FFFF;

	/// Reset HSEBYP bit
	RCC->CR &= 0xFFFBFFFF;

	/// Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits
	RCC->CFGR &= 0xFF80FFFF;

#ifdef STM32F10X_CL
	/// Reset PLL2ON and PLL3ON bits
	RCC->CR &= 0xEBFFFFFF;

	/// Disable all interrupts and clear pending bits
	RCC->CIR = 0x00FF0000;

	/// Reset CFGR2 register
	RCC->CFGR2 = 0x00000000;
#elif defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || (defined STM32F10X_HD_VL)
	/// Disable all interrupts and clear pending bits
	RCC->CIR = 0x009F0000;

	/// Reset CFGR2 register
	RCC->CFGR2 = 0x00000000;
#else
	/// Disable all interrupts and clear pending bits
	RCC->CIR = 0x009F0000;
#endif

#if defined (STM32F10X_HD) || (defined STM32F10X_XL) || (defined STM32F10X_HD_VL)
#ifdef DATA_IN_ExtSRAM
	SystemInit_ExtMemCtl();
#endif
#endif

	/// Configure the System clock frequency,
	/// HCLK, PCLK2 and PCLK1 prescalers
	/// Configure the Flash Latency cycles and enable prefetch buffer
	SetSysClock();

#ifdef VECT_TAB_SRAM
	/// Vector Table Relocation in Internal SRAM.
	SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET;
#else
	/// Vector Table Relocation in Internal FLASH.
	SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET;
#endif
}

void SystemCoreClockUpdate() {
	uint32_t tmp = 0, pllmull = 0, pllsource = 0;

#ifdef  STM32F10X_CL
	uint32_t prediv1source = 0, prediv1factor = 0, prediv2factor = 0, pll2mull = 0;
#endif

#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || (defined STM32F10X_HD_VL)
	uint32_t prediv1factor = 0;
#endif

	tmp = RCC->CFGR & RCC_CFGR_SWS;

	switch (tmp) {
	case 0x00:
		SystemCoreClock = HSI_VALUE;
		break;
	case 0x04:
		SystemCoreClock = HSE_VALUE;
		break;
	case 0x08:
		pllmull = RCC->CFGR & RCC_CFGR_PLLMULL;
		pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;

#ifndef STM32F10X_CL
		pllmull = (pllmull >> 18) + 2;

		if (pllsource == 0x00) {
			/// HSI oscillator clock divided by 2 selected
			/// as PLL clock entry
			SystemCoreClock = (HSI_VALUE >> 1) * pllmull;
		} else {
#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || (defined STM32F10X_HD_VL)
			prediv1factor = (RCC->CFGR2 & RCC_CFGR2_PREDIV1) + 1;
			/// HSE oscillator clock selected as PREDIV1 clock entry
			SystemCoreClock = (HSE_VALUE / prediv1factor) * pllmull;
#else
			/// HSE selected as PLL clock entry
			if ((RCC->CFGR & RCC_CFGR_PLLXTPRE) != RESET) {
				/// HSE oscillator clock divided by 2
				SystemCoreClock = (HSE_VALUE >> 1) * pllmull;
			} else {
				SystemCoreClock = HSE_VALUE * pllmull;
			}
#endif
		}
#else
		pllmull = pllmull >> 18;

		if (pllmull != 0x0D) {
			pllmull += 2;
		} else {
			/// PLL multiplication factor = PLL input clock * 6.5
			pllmull = 13 / 2;
		}

		if (pllsource == 0x00) {
			/// HSI oscillator clock divided by 2 selected
			/// as PLL clock entry
			SystemCoreClock = (HSI_VALUE >> 1) * pllmull;
			break;
		}
		
		/// PREDIV1 selected as PLL clock entry
		/// Get PREDIV1 clock source and division factor
		prediv1source = RCC->CFGR2 & RCC_CFGR2_PREDIV1SRC;
		prediv1factor = (RCC->CFGR2 & RCC_CFGR2_PREDIV1) + 1;

		if (prediv1source == 0) {
			/// HSE oscillator clock selected as PREDIV1 clock entry
			SystemCoreClock = (HSE_VALUE / prediv1factor) * pllmull;
			break;
		}

		/// PLL2 clock selected as PREDIV1 clock entry

		/// Get PREDIV2 division factor and PLL2 multiplication factor
		prediv2factor = ((RCC->CFGR2 & RCC_CFGR2_PREDIV2) >> 4) + 1;
		pll2mull = ((RCC->CFGR2 & RCC_CFGR2_PLL2MUL) >> 8) + 2;
		SystemCoreClock =
			(((HSE_VALUE / prediv2factor) * pll2mull) / prediv1factor) * pllmull;
#endif
		break;

	default:
		SystemCoreClock = HSI_VALUE;
		break;
	}

	/// Compute HCLK clock frequency ----------------
	/// Get HCLK prescaler
	tmp = AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> 4];
	/// HCLK clock frequency
	SystemCoreClock >>= tmp;
}

static void SetSysClock() {
#ifdef SYSCLK_FREQ_HSE
	SetSysClockToHSE();
#elif defined SYSCLK_FREQ_24MHz
	SetSysClockTo24();
#elif defined SYSCLK_FREQ_36MHz
	SetSysClockTo36();
#elif defined SYSCLK_FREQ_48MHz
	SetSysClockTo48();
#elif defined SYSCLK_FREQ_56MHz
	SetSysClockTo56();
#elif defined SYSCLK_FREQ_72MHz
	SetSysClockTo72();
#endif

	/// If none of the define above is enabled,
	/// the HSI is used as System clock source (default after reset)
}

#ifdef DATA_IN_ExtSRAM

/// Setup the external memory controller. 
/// Called in startup_stm32f10x_xx.s/.c before jump to main.
void SystemInit_ExtMemCtl() {
	/// Enable FSMC clock
	RCC->AHBENR = 0x00000114;

	/// Enable GPIOD, GPIOE, GPIOF and GPIOG clocks
	RCC->APB2ENR = 0x000001E0;

/* ---------------  SRAM Data lines, NOE and NWE configuration ---------------*/
/*----------------  SRAM Address lines configuration -------------------------*/
/*----------------  NOE and NWE configuration --------------------------------*/
/*----------------  NE3 configuration ----------------------------------------*/
/*----------------  NBL0, NBL1 configuration ---------------------------------*/

	GPIOD->CRL = 0x44BB44BB;
	GPIOD->CRH = 0xBBBBBBBB;

	GPIOE->CRL = 0xB44444BB;
	GPIOE->CRH = 0xBBBBBBBB;

	GPIOF->CRL = 0x44BBBBBB;
	GPIOF->CRH = 0xBBBB4444;

	GPIOG->CRL = 0x44BBBBBB;
	GPIOG->CRH = 0x44444B44;

/*----------------  FSMC Configuration ---------------------------------------*/
/*----------------  Enable FSMC Bank1_SRAM Bank ------------------------------*/

	FSMC_Bank1->BTCR[4] = 0x00001011;
	FSMC_Bank1->BTCR[5] = 0x00000200;
}
#endif

#ifdef SYSCLK_FREQ_HSE

static void SetSysClockToHSE() {
	__IO uint32_t StartUpCounter = 0, HSEStatus = 0;

	RCC->CR |= RCC_CR_HSEON;

	/// Wait till HSE is ready and if Time out is reached exit
	while ((HSEStatus == 0) && (StartUpCounter++ != HSE_STARTUP_TIMEOUT))
		HSEStatus = RCC->CR & RCC_CR_HSERDY;

	if ((RCC->CR & RCC_CR_HSERDY) != RESET)
		HSEStatus = 0x01;
	else
		HSEStatus = 0x00;

	if (HSEStatus != 0x01) {
		/// ...
		return;
	}

#if !defined STM32F10X_LD_VL && !defined STM32F10X_MD_VL && !defined STM32F10X_HD_VL
	/// Enable Prefetch Buffer
	FLASH->ACR |= FLASH_ACR_PRFTBE;

	/// Flash 0 wait state
	FLASH->ACR &= ~FLASH_ACR_LATENCY;

#ifndef STM32F10X_CL
	FLASH->ACR |= FLASH_ACR_LATENCY_0;
#else
	if (HSE_VALUE <= 24000000)
		FLASH->ACR |= FLASH_ACR_LATENCY_0;
	else
		FLASH->ACR |= FLASH_ACR_LATENCY_1;
#endif
#endif
	/// HCLK = SYSCLK
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	/// PCLK2 = HCLK
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

	/// PCLK1 = HCLK
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;

	/// Select HSE as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_HSE;

	/// Wait till HSE is used as system clock source
	while ((RCC->CFGR & RCC_CFGR_SWS) != 0x04);
}

#elif defined SYSCLK_FREQ_24MHz

static void SetSysClockTo24() {
	__IO uint32_t StartUpCounter = 0, HSEStatus = 0;

	RCC->CR |= RCC_CR_HSEON;

	/// Wait till HSE is ready and if Time out is reached exit
	while ((HSEStatus == 0) && (StartUpCounter++ != HSE_STARTUP_TIMEOUT))
		HSEStatus = RCC->CR & RCC_CR_HSERDY;

	if ((RCC->CR & RCC_CR_HSERDY) != RESET)
		HSEStatus = 0x01;
	else
		HSEStatus = 0x00;

	if (HSEStatus != 0x01) {
		/// ...
		return;
	}

#if !defined STM32F10X_LD_VL && !defined STM32F10X_MD_VL && !defined STM32F10X_HD_VL
	/// Enable Prefetch Buffer
	FLASH->ACR |= FLASH_ACR_PRFTBE;

	/// Flash 0 wait state
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_0;
#endif

	/// HCLK = SYSCLK
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	/// PCLK2 = HCLK
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

	/// PCLK1 = HCLK
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;

#ifdef STM32F10X_CL
	/// PLL configuration: PLLCLK = PREDIV1 * 6 = 24 MHz
	RCC->CFGR &=
		~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);

	RCC->CFGR |=
		RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLSRC_PREDIV1 | RCC_CFGR_PLLMULL6;

	/// PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz
	/// PREDIV1 configuration: PREDIV1CLK = PLL2 / 10 = 4 MHz
	RCC->CFGR2 &=
		~(RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL2MUL | RCC_CFGR2_PREDIV1 | RCC_CFGR2_PREDIV1SRC);

	RCC->CFGR2 |=
		RCC_CFGR2_PREDIV2_DIV5 | RCC_CFGR2_PLL2MUL8 | RCC_CFGR2_PREDIV1SRC_PLL2 | RCC_CFGR2_PREDIV1_DIV10;

	/// Enable PLL2
	RCC->CR |= RCC_CR_PLL2ON;
	/// Wait till PLL2 is ready
	while ((RCC->CR & RCC_CR_PLL2RDY) == 0);

#elif defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)

	/// PLL configuration:  = (HSE / 2) * 6 = 24 MHz
	RCC->CFGR &=
		~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));

	RCC->CFGR |=
		RCC_CFGR_PLLSRC_PREDIV1 | RCC_CFGR_PLLXTPRE_PREDIV1_Div2 | RCC_CFGR_PLLMULL6;
#else
	///  PLL configuration: = (HSE / 2) * 6 = 24 MHz
	RCC->CFGR &=
		~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);

	RCC->CFGR |=
		RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLXTPRE_HSE_Div2 | RCC_CFGR_PLLMULL6;
#endif

	RCC->CR |= RCC_CR_PLLON;

	/// Wait till PLL is ready
	while ((RCC->CR & RCC_CR_PLLRDY) == 0);

	/// Select PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/// Wait till PLL is used as system clock source
	while ((RCC->CFGR & RCC_CFGR_SWS) != 0x08);
}

#elif defined SYSCLK_FREQ_36MHz

static void SetSysClockTo36() {
	__IO uint32_t StartUpCounter = 0, HSEStatus = 0;

	RCC->CR |= RCC_CR_HSEON;

	/// Wait till HSE is ready and if Time out is reached exit
	while ((HSEStatus == 0) && (StartUpCounter++ != HSE_STARTUP_TIMEOUT))
		HSEStatus = RCC->CR & RCC_CR_HSERDY;

	if ((RCC->CR & RCC_CR_HSERDY) != RESET)
		HSEStatus = 0x01;
	else
		HSEStatus = 0x00;

	if (HSEStatus != 0x01) {
		/// ...
		return;
	}

	/// Enable Prefetch Buffer
	FLASH->ACR |= FLASH_ACR_PRFTBE;

	/// Flash 1 wait state
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_1;

	/// HCLK = SYSCLK
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	/// PCLK2 = HCLK
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

	/// PCLK1 = HCLK
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;

#ifdef STM32F10X_CL

	/// PLL configuration: PLLCLK = PREDIV1 * 9 = 36 MHz
	RCC->CFGR &=
		~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);

	RCC->CFGR |=
		RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLSRC_PREDIV1 | RCC_CFGR_PLLMULL9;

	/// PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz
	/// PREDIV1 configuration: PREDIV1CLK = PLL2 / 10 = 4 MHz

	RCC->CFGR2 &=
		~(RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL2MUL | RCC_CFGR2_PREDIV1 | RCC_CFGR2_PREDIV1SRC);

	RCC->CFGR2 |=
		RCC_CFGR2_PREDIV2_DIV5 | RCC_CFGR2_PLL2MUL8 | RCC_CFGR2_PREDIV1SRC_PLL2 | RCC_CFGR2_PREDIV1_DIV10;

	/// Enable PLL2
	RCC->CR |= RCC_CR_PLL2ON;
	/// Wait till PLL2 is ready
	while ((RCC->CR & RCC_CR_PLL2RDY) == 0);

#else
	///  PLL configuration: PLLCLK = (HSE / 2) * 9 = 36 MHz
	RCC->CFGR &=
		~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));

	RCC->CFGR |=
		RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLXTPRE_HSE_Div2 | RCC_CFGR_PLLMULL9;
#endif

	/// Enable PLL
	RCC->CR |= RCC_CR_PLLON;

	/// Wait till PLL is ready
	while ((RCC->CR & RCC_CR_PLLRDY) == 0);

	/// Select PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/// Wait till PLL is used as system clock source
	while ((RCC->CFGR & RCC_CFGR_SWS) != 0x08);
}

#elif defined SYSCLK_FREQ_48MHz

static void SetSysClockTo48() {
	__IO uint32_t StartUpCounter = 0, HSEStatus = 0;

	RCC->CR |= RCC_CR_HSEON;

	/// Wait till HSE is ready and if Time out is reached exit
	while ((HSEStatus == 0) && (StartUpCounter++ != HSE_STARTUP_TIMEOUT))
		HSEStatus = RCC->CR & RCC_CR_HSERDY;

	if ((RCC->CR & RCC_CR_HSERDY) != RESET)
		HSEStatus = 0x01;
	else
		HSEStatus = 0x00;

	if (HSEStatus != 0x01) {
		/// ...
		return;
	}

	/// Enable Prefetch Buffer
	FLASH->ACR |= FLASH_ACR_PRFTBE;

	/// Flash 1 wait state
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_1;

	/// HCLK = SYSCLK
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	/// PCLK2 = HCLK
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

	/// PCLK1 = HCLK
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

#ifdef STM32F10X_CL
	/// PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz
	/// PREDIV1 configuration: PREDIV1CLK = PLL2 / 5 = 8 MHz

	RCC->CFGR2 &=
		~(RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL2MUL | RCC_CFGR2_PREDIV1 | RCC_CFGR2_PREDIV1SRC);

	RCC->CFGR2 |=
		RCC_CFGR2_PREDIV2_DIV5 | RCC_CFGR2_PLL2MUL8 | RCC_CFGR2_PREDIV1SRC_PLL2 | RCC_CFGR2_PREDIV1_DIV5;

	/// Enable PLL2
	RCC->CR |= RCC_CR_PLL2ON;
	/// Wait till PLL2 is ready
	while ((RCC->CR & RCC_CR_PLL2RDY) == 0);

	/// PLL configuration: PLLCLK = PREDIV1 * 6 = 48 MHz
	RCC->CFGR &= ~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
	RCC->CFGR |= RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLSRC_PREDIV1 | RCC_CFGR_PLLMULL6;
#else
	///  PLL configuration: PLLCLK = HSE * 6 = 48 MHz
	RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL6;
#endif

	/// Enable PLL
	RCC->CR |= RCC_CR_PLLON;

	/// Wait till PLL is ready
	while ((RCC->CR & RCC_CR_PLLRDY) == 0);

	/// Select PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/// Wait till PLL is used as system clock source
	while ((RCC->CFGR & RCC_CFGR_SWS) != 0x08);
}

#elif defined SYSCLK_FREQ_56MHz
/**
  * @brief  Sets System clock frequency to 56MHz and configure HCLK, PCLK2 
  *         and PCLK1 prescalers. 
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
static void SetSysClockTo56() {
	__IO uint32_t StartUpCounter = 0, HSEStatus = 0;

	/// Enable HSE
	RCC->CR |= RCC_CR_HSEON;

	/// Wait till HSE is ready and if Time out is reached exit
	while ((HSEStatus == 0) && (StartUpCounter++ != HSE_STARTUP_TIMEOUT))
		HSEStatus = RCC->CR & RCC_CR_HSERDY;

	if ((RCC->CR & RCC_CR_HSERDY) != RESET)
		HSEStatus = 0x01;
	else
		HSEStatus = 0x00;

	if (HSEStatus != 0x01) {
		/// ...
		return;
	}

	/// Enable Prefetch Buffer
	FLASH->ACR |= FLASH_ACR_PRFTBE;

	/// Flash 2 wait state
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_2;

	/// HCLK = SYSCLK
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	/// PCLK2 = HCLK
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

	/// PCLK1 = HCLK
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

#ifdef STM32F10X_CL
	/// PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz
	/// PREDIV1 configuration: PREDIV1CLK = PLL2 / 5 = 8 MHz

	RCC->CFGR2 &=
		~(RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL2MUL | RCC_CFGR2_PREDIV1 | RCC_CFGR2_PREDIV1SRC);

	RCC->CFGR2 |=
		RCC_CFGR2_PREDIV2_DIV5 | RCC_CFGR2_PLL2MUL8 | RCC_CFGR2_PREDIV1SRC_PLL2 | RCC_CFGR2_PREDIV1_DIV5;

	/// Enable PLL2
	RCC->CR |= RCC_CR_PLL2ON;
	/// Wait till PLL2 is ready
	while ((RCC->CR & RCC_CR_PLL2RDY) == 0);

	/// PLL configuration: PLLCLK = PREDIV1 * 7 = 56 MHz
	RCC->CFGR &= ~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
	RCC->CFGR |= RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLSRC_PREDIV1 | RCC_CFGR_PLLMULL7;

#else
	/// PLL configuration: PLLCLK = HSE * 7 = 56 MHz
	RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL7;

#endif

	/// Enable PLL
	RCC->CR |= RCC_CR_PLLON;

	/// Wait till PLL is ready
	while ((RCC->CR & RCC_CR_PLLRDY) == 0);

	/// Select PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/// Wait till PLL is used as system clock source
	while ((RCC->CFGR & RCC_CFGR_SWS) != 0x08);
}

#elif defined SYSCLK_FREQ_72MHz

/// This function should be used only after reset.
static void SetSysClockTo72() {
	__IO uint32_t StartUpCounter = 0, HSEStatus = 0;

	RCC->CR |= RCC_CR_HSEON;

	while ((HSEStatus == 0) && (StartUpCounter++ != HSE_STARTUP_TIMEOUT))
		HSEStatus = RCC->CR & RCC_CR_HSERDY;

	if ((RCC->CR & RCC_CR_HSERDY) != RESET)
		HSEStatus = 0x01;
	else
		HSEStatus = 0x00;

	if (HSEStatus != 0x01) {
		/// ...
		return;
	}

	/// Enable Prefetch Buffer
	FLASH->ACR |= FLASH_ACR_PRFTBE;

	/// Flash 2 wait state
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_2;

	/// HCLK = SYSCLK
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	/// PCLK2 = HCLK
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

	/// PCLK1 = HCLK
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

#ifdef STM32F10X_CL
	/// PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz
	/// PREDIV1 configuration: PREDIV1CLK = PLL2 / 5 = 8 MHz

	RCC->CFGR2 &= ~(RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL2MUL | RCC_CFGR2_PREDIV1 | RCC_CFGR2_PREDIV1SRC);
	RCC->CFGR2 |= RCC_CFGR2_PREDIV2_DIV5 | RCC_CFGR2_PLL2MUL8 | RCC_CFGR2_PREDIV1SRC_PLL2 | RCC_CFGR2_PREDIV1_DIV5;

	/// Enable PLL2
	RCC->CR |= RCC_CR_PLL2ON;
	/// Wait till PLL2 is ready
	while ((RCC->CR & RCC_CR_PLL2RDY) == 0);

	/// PLL configuration: PLLCLK = PREDIV1 * 9 = 72 MHz
	RCC->CFGR &= ~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
	RCC->CFGR |= RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLSRC_PREDIV1 | RCC_CFGR_PLLMULL9;
#else
	/// PLL configuration: PLLCLK = HSE * 9 = 72 MHz
	RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);

	RCC->CFGR |= RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9;
#endif
	/// Enable PLL
	RCC->CR |= RCC_CR_PLLON;

	/// Wait till PLL is ready
	while ((RCC->CR & RCC_CR_PLLRDY) == 0);

	/// Select PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/// Wait till PLL is used as system clock source
	while ((RCC->CFGR & RCC_CFGR_SWS) != 0x08);
}

#endif


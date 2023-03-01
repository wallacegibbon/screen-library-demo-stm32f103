#include <stdint.h>

#if defined (__CC_ARM)
#define __ASM __asm
#define __INLINE __inline

#elif defined (__ICCARM__)
#define __ASM __asm
#define __INLINE inline

#elif defined  (__GNUC__)
#define __ASM __asm
#define __INLINE inline

#elif defined (__TASKING__)
#define __ASM __asm
#define __INLINE inline

#endif

uint32_t __get_PSP(void) __attribute__((naked));
uint32_t __get_PSP(void) {
	uint32_t result = 0;

	__ASM volatile (
		"MRS %0, psp\n\t"
		"MOV r0, %0 \n\t" "BX  lr     \n\t":"=r" (result)
	);

	return result;
}

void __set_PSP(uint32_t topOfProcStack) __attribute__((naked));
void __set_PSP(uint32_t topOfProcStack) {
	__ASM volatile (
		"MSR psp, %0\n\t"
		"BX  lr     \n\t"::"r" (topOfProcStack)
	);
}

uint32_t __get_MSP(void) __attribute__((naked));
uint32_t __get_MSP(void) {
	uint32_t result = 0;

	__ASM volatile (
		"MRS %0, msp\n\t"
		"MOV r0, %0 \n\t" "BX  lr     \n\t":"=r" (result)
	);
	return result;
}

void __set_MSP(uint32_t topOfMainStack) __attribute__((naked));
void __set_MSP(uint32_t topOfMainStack) {
	__ASM volatile (
		"MSR msp, %0\n\t"
		"BX  lr     \n\t"::"r" (topOfMainStack)
	);
}

uint32_t __get_BASEPRI(void) {
	uint32_t result = 0;

	__ASM volatile ("MRS %0, basepri_max":"=r" (result));
	return result;
}

void __set_BASEPRI(uint32_t value) {
	__ASM volatile ("MSR basepri, %0"::"r" (value));
}

uint32_t __get_PRIMASK(void) {
	uint32_t result = 0;

	__ASM volatile ("MRS %0, primask":"=r" (result));
	return result;
}

void __set_PRIMASK(uint32_t priMask) {
	__ASM volatile ("MSR primask, %0"::"r" (priMask));
}

uint32_t __get_FAULTMASK(void) {
	uint32_t result = 0;

	__ASM volatile ("MRS %0, faultmask":"=r" (result));
	return result;
}

void __set_FAULTMASK(uint32_t faultMask) {
	__ASM volatile ("MSR faultmask, %0"::"r" (faultMask));
}

uint32_t __get_CONTROL(void) {
	uint32_t result = 0;

	__ASM volatile ("MRS %0, control":"=r" (result));
	return result;
}

void __set_CONTROL(uint32_t control) {
	__ASM volatile ("MSR control, %0"::"r" (control));
}

uint32_t __REV(uint32_t value) {
	uint32_t result = 0;

	__ASM volatile ("rev %0, %1":"=r" (result):"r"(value));
	return result;
}

uint32_t __REV16(uint16_t value) {
	uint32_t result = 0;

	__ASM volatile ("rev16 %0, %1":"=r" (result):"r"(value));
	return result;
}

int32_t __REVSH(int16_t value) {
	uint32_t result = 0;

	__ASM volatile ("revsh %0, %1":"=r" (result):"r"(value));
	return result;
}

uint32_t __RBIT(uint32_t value) {
	uint32_t result = 0;

	__ASM volatile ("rbit %0, %1":"=r" (result):"r"(value));
	return result;
}

uint8_t __LDREXB(uint8_t * addr) {
	uint8_t result = 0;

	__ASM volatile ("ldrexb %0, [%1]":"=r" (result):"r"(addr));
	return result;
}

uint16_t __LDREXH(uint16_t * addr) {
	uint16_t result = 0;

	__ASM volatile ("ldrexh %0, [%1]":"=r" (result):"r"(addr));
	return result;
}

uint32_t __LDREXW(uint32_t * addr) {
	uint32_t result = 0;

	__ASM volatile ("ldrex %0, [%1]":"=r" (result):"r"(addr));
	return result;
}

uint32_t __STREXB(uint8_t value, uint8_t * addr) {
	uint32_t result = 0;

	__ASM volatile (
		"strexb %0, %2, [%1]":"=&r" (result):"r"(addr),
		"r"(value)
	);

	return result;
}

uint32_t __STREXH(uint16_t value, uint16_t * addr) {
	uint32_t result = 0;

	__ASM volatile (
		"strexh %0, %2, [%1]":"=&r" (result):"r"(addr),
		"r"(value)
	);

	return result;
}

uint32_t __STREXW(uint32_t value, uint32_t * addr) {
	uint32_t result = 0;

	__ASM volatile (
		"strex %0, %2, [%1]":"=r" (result):"r"(addr),
		"r"(value)
	);

	return result;
}


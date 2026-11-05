/**
 * @file plc_sdb_expansion.c
 * @brief PLC Operand Write Primitives (Byte/Word/DWord), 96 MHz HSE+PLL Clock
 *        Initialization, System Data Block (SDB) Parser, Retentive Flash
 *        Restoration, Program Block Indexer, and Expansion Module Bus
 *        Enumerator (0x0803b100 .. 0x0803dfdd).
 *
 * Note: The tail of FUN_0803aee6 (0x0803aee6 .. 0x0803b423) spans across the
 *       0x0803b100 boundary; the complete FUN_0803aee6 function is defined in
 *       src/plc_int_dispatch.c alongside its sibling SBR wrappers.
 *
 * Target MCU: APM32E103VET6 / STM32F103VET6 (Cortex-M3 @ 96 MHz)
 * Board:      SIMATIC S7-200 CPU 226 Clone (6ES7 216-2BD23-0XB0)
 */

#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * Hardware Register & Memory Access Macros
 * ========================================================================== */
#define REG8(addr)          (*(volatile uint8_t  *)(uintptr_t)(addr))
#define REG16(addr)         (*(volatile uint16_t *)(uintptr_t)(addr))
#define REG32(addr)         (*(volatile uint32_t *)(uintptr_t)(addr))

#define RAM8(addr)          (*(volatile uint8_t  *)(uintptr_t)(addr))
#define RAM16(addr)         (*(volatile uint16_t *)(uintptr_t)(addr))
#define RAM32(addr)         (*(volatile uint32_t *)(uintptr_t)(addr))
#define RAM_S16(addr)       (*(volatile int16_t  *)(uintptr_t)(addr))
#define RAM_S32(addr)       (*(volatile int32_t  *)(uintptr_t)(addr))
#define RAM_PTR8(addr)      ((volatile uint8_t   *)(uintptr_t)(addr))
#define RAM_PTR16(addr)     ((volatile uint16_t  *)(uintptr_t)(addr))
#define RAM_PTR32(addr)     ((volatile uint32_t  *)(uintptr_t)(addr))

#define FLASH_PTR8(addr)    ((const uint8_t      *)(uintptr_t)(addr))
#define FLASH_PTR16(addr)   ((const uint16_t     *)(uintptr_t)(addr))

/* RCC & Flash Registers */
#define RCC_CR              REG32(0x40021000UL)
#define RCC_CFGR            REG32(0x40021004UL)
#define RCC_CIR             REG32(0x40021008UL)
#define FLASH_ACR           REG32(0x40022000UL)

/* GPIO & Bit-Band Registers for Expansion Bus */
#define GPIOC_BASE          ((volatile uint32_t *)0x40011000UL)
#define GPIOC_CRH           REG32(0x40011004UL)
#define BITBAND_GPIOC_ODR_PC8   REG32(0x422201A0UL)
#define BITBAND_GPIOC_ODR_PC9   REG32(0x422201A4UL)
#define BITBAND_GPIOC_ODR_PC10  REG32(0x422201A8UL)
#define BITBAND_GPIOC_ODR_PC11  REG32(0x422201ACUL)

/* Timers */
#define TIM2_CR1            REG16(0x40000000UL)
#define TIM2_DIER           REG16(0x4000000CUL)
#define TIM2_SR             REG16(0x40000010UL)
#define TIM2_CNT            REG16(0x40000024UL)

#define TIM3_CR1            REG16(0x40000400UL)
#define TIM3_CNT            REG16(0x40000424UL)

#define TIM4_CR1            REG16(0x40000800UL)
#define TIM4_CNT            REG16(0x40000824UL)

#define TIM5_CR1            REG16(0x40000C00UL)
#define TIM5_DIER           REG16(0x40000C0CUL)
#define TIM5_SR             REG16(0x40000C10UL)
#define TIM5_CNT            REG16(0x40000C24UL)

#define TIM6_CR1            REG16(0x40001000UL)
#define TIM6_DIER           REG16(0x4000100CUL)
#define TIM6_SR             REG16(0x40001010UL)
#define TIM6_CNT            REG16(0x40001024UL)

#define TIM7_CR1            REG16(0x40001400UL)
#define TIM7_DIER           REG16(0x4000140CUL)
#define TIM7_SR             REG16(0x40001410UL)
#define TIM7_CNT            REG16(0x40001424UL)

#define TIM1_CR1            REG16(0x40012C00UL)
#define TIM1_DIER           REG16(0x40012C0CUL)
#define TIM1_SR             REG16(0x40012C10UL)
#define TIM1_CNT            REG16(0x40012C24UL)
#define TIM1_BDTR           REG16(0x40012C44UL)

#define TIM8_CR1            REG16(0x40013400UL)
#define TIM8_DIER           REG16(0x4001340CUL)
#define TIM8_SR             REG16(0x40013410UL)
#define TIM8_CNT            REG16(0x40013424UL)
#define TIM8_BDTR           REG16(0x40013444UL)

/* External declarations */
extern uint32_t FUN_08001070(void);
extern void     FUN_080043b8(uint32_t delay_loops);
extern void     FUN_0800547c(void);
extern void     FUN_08012a22(uint32_t *exti_init);
extern uint32_t FUN_08012aac(uint16_t cmd_word, uint8_t delay_loops);
extern uint32_t FUN_08013346(uint8_t cmd_byte, uint8_t delay_loops);
extern void     FUN_08014526(volatile uint32_t *gpio_base, uint16_t pins);
extern void     FUN_0801452a(volatile uint32_t *gpio_base, uint16_t pins);
extern uint32_t FUN_08014972(uint32_t val);
extern uint32_t FUN_080149ea(uint32_t val);
extern uint32_t FUN_08014a5a(uint32_t val);
extern uint32_t FUN_08014e46(uint32_t val);
extern uint32_t FUN_08036ca0(const uint8_t *buf, uint32_t len);
extern void     FUN_0803e06c(uint32_t port0_baud_code, uint32_t port1_baud_code);
extern uint32_t FUN_08040ab4(uint16_t start_pos, uint16_t end_pos);
extern void     FUN_08040dd2(void);
extern void     FUN_080421b4(void);

/* ==========================================================================
 * Function: FUN_0803b424 (PLC_WriteByteOperand) @ 0x0803b424 (750 bytes)
 *
 * Writes an 8-bit value `val` to the PLC operand encoded at the current
 * instruction stream position `0x200001CC` (Direct or Indirect `*VDxx`/`*LDxx`/`*ACx`):
 *   - Supports V, I, Q, M, SM, AC0..AC3, S, and L memory areas.
 *   - Sets `SMB4` indirect addressing error bit (`0x10`) if pointer is out of range.
 * ========================================================================== */
void FUN_0803b424(uint8_t val)
{
    uint32_t pos = RAM8(0x200001CCUL);
    uint8_t  op_mode = RAM_PTR8(0x20000496UL)[pos + 2U];

    if (op_mode == 2U) {
        uint8_t area = RAM_PTR8(0x20000496UL)[pos + 3U];
        if (area == 1U) {
            uint16_t off = ((uint16_t)RAM_PTR8(0x20000496UL)[pos + 4U] << 8) |
                           RAM_PTR8(0x20000496UL)[pos + 5U];
            RAM_PTR8(0x2000108CUL)[off] = val;
            RAM8(0x200001CCUL) = (uint8_t)(pos + 4U);
            return;
        }
        if (area == 5U) {
            uint16_t off = ((uint16_t)RAM_PTR8(0x20000496UL)[pos + 4U] << 8) |
                           RAM_PTR8(0x20000496UL)[pos + 5U];
            RAM_PTR8(0x20005F28UL)[off] = val;
            RAM8(0x200001CCUL) = (uint8_t)(pos + 4U);
            return;
        }

        uint8_t sub = RAM_PTR8(0x20000496UL)[pos + 4U];
        switch (area) {
        case 2U:  RAM_PTR8(0x20005ED8UL)[sub] = val; break;
        case 3U:  RAM_PTR8(0x20005EE8UL)[sub] = val; break;
        case 4U:  RAM_PTR8(0x20005EF8UL)[sub] = val; break;
        case 12U:
            RAM_PTR32(0x20005F18UL)[sub] =
                (RAM_PTR32(0x20005F18UL)[sub] & 0xFFFFFF00UL) | val;
            break;
        case 15U: RAM_PTR8(0x20005EB8UL)[sub] = val; break;
        case 0x70U: RAM_PTR8(0x20006110UL)[sub] = val; break;
        default:  break;
        }
        RAM8(0x200001CCUL) = (uint8_t)(pos + 3U);
        return;
    }

    if (op_mode == 3U) {
        uint8_t ptr_area = RAM_PTR8(0x20000496UL)[pos + 3U];
        uint32_t ptr_val = 0;
        if (ptr_area == 1U) {
            uint32_t v_off = ((uint32_t)RAM_PTR8(0x20000496UL)[pos + 4U] << 8) |
                             RAM_PTR8(0x20000496UL)[pos + 5U];
            ptr_val = ((uint32_t)RAM_PTR8(0x2000108CUL)[v_off] << 24) |
                      ((uint32_t)RAM_PTR8(0x2000108DUL)[v_off] << 16) |
                      ((uint32_t)RAM_PTR8(0x2000108EUL)[v_off] << 8)  |
                      RAM_PTR8(0x2000108FUL)[v_off];
            RAM8(0x200001CCUL) = (uint8_t)(pos + 4U);
        } else if (ptr_area == 0x0CU) {
            uint8_t ac_idx = RAM_PTR8(0x20000496UL)[pos + 4U];
            if (ac_idx != 0) {
                ptr_val = RAM_PTR32(0x20005F18UL)[ac_idx];
            }
            RAM8(0x200001CCUL) = (uint8_t)(pos + 3U);
        } else if (ptr_area == 0x70U) {
            uint8_t l_off = RAM_PTR8(0x20000496UL)[pos + 4U];
            ptr_val = ((uint32_t)RAM_PTR8(0x20006110UL)[l_off] << 24) |
                      ((uint32_t)RAM_PTR8(0x20006111UL)[l_off] << 16) |
                      ((uint32_t)RAM_PTR8(0x20006112UL)[l_off] << 8)  |
                      RAM_PTR8(0x20006113UL)[l_off];
            RAM8(0x200001CCUL) = (uint8_t)(pos + 3U);
        }

        if (ptr_val >= 0x08005000UL) {
            if (ptr_val < 0x0800A000UL) {
                RAM_PTR8(0x2000108CUL)[ptr_val - 0x08005000UL] = val;
                return;
            }
        } else {
            switch ((ptr_val >> 8) & 0xFFU) {
            case 0U:
                if (ptr_val < 0x08000010UL) {
                    RAM_PTR8(0x20005ED8UL)[ptr_val & 0xFFU] = val;
                    return;
                }
                break;
            case 1U:
                if (ptr_val < 0x08000110UL) {
                    RAM_PTR8(0x20005EE8UL)[(ptr_val - 0x08000100UL) & 0xFFFFU] = val;
                    return;
                }
                break;
            case 2U:
                if (ptr_val < 0x08000220UL) {
                    RAM_PTR8(0x20005EF8UL)[(ptr_val - 0x08000200UL) & 0xFFFFU] = val;
                    return;
                }
                break;
            case 3U:
            case 4U:
                if (ptr_val >= 0x0800031EUL && ptr_val < 0x0800042CUL) {
                    RAM_PTR8(0x20005F28UL)[(ptr_val - 0x08000300UL) & 0xFFFFU] = val;
                    return;
                }
                break;
            case 11U:
                if (ptr_val < 0x08000B20UL) {
                    RAM_PTR8(0x20005EB8UL)[(ptr_val - 0x08000B00UL) & 0xFFFFU] = val;
                    return;
                }
                break;
            default:
                break;
            }
        }
        RAM8(0x20005F2CUL) |= 0x10U;
    }
}

/* ==========================================================================
 * Function: FUN_0803b712 (PLC_WriteDWordOperand) @ 0x0803b712 (1044 bytes)
 *
 * Writes a 32-bit value `val` (DWord / DInt / Real) to the PLC operand encoded
 * at `0x200001CC` (big-endian byte order in V, I, Q, M, SM, S, L; native
 * 32-bit word in AC0..AC3).
 * ========================================================================== */
void FUN_0803b712(uint32_t val)
{
    uint32_t pos = RAM8(0x200001CCUL);
    uint8_t  op_mode = RAM_PTR8(0x20000496UL)[pos + 2U];

    uint8_t b0 = (uint8_t)(val >> 24);
    uint8_t b1 = (uint8_t)(val >> 16);
    uint8_t b2 = (uint8_t)(val >> 8);
    uint8_t b3 = (uint8_t)val;

    if (op_mode == 2U) {
        uint8_t area = RAM_PTR8(0x20000496UL)[pos + 3U];
        if (area == 1U) {
            uint32_t off = ((uint32_t)RAM_PTR8(0x20000496UL)[pos + 4U] << 8) |
                           RAM_PTR8(0x20000496UL)[pos + 5U];
            RAM_PTR8(0x2000108CUL)[off] = b0;
            RAM_PTR8(0x2000108DUL)[off] = b1;
            RAM_PTR8(0x2000108EUL)[off] = b2;
            RAM_PTR8(0x2000108FUL)[off] = b3;
            RAM8(0x200001CCUL) = (uint8_t)(pos + 4U);
            return;
        }
        if (area == 5U) {
            uint32_t off = ((uint32_t)RAM_PTR8(0x20000496UL)[pos + 4U] << 8) |
                           RAM_PTR8(0x20000496UL)[pos + 5U];
            RAM_PTR8(0x20005F28UL)[off] = b0;
            RAM_PTR8(0x20005F29UL)[off] = b1;
            RAM_PTR8(0x20005F2AUL)[off] = b2;
            RAM_PTR8(0x20005F2BUL)[off] = b3;
            RAM8(0x200001CCUL) = (uint8_t)(pos + 4U);
            return;
        }

        uint32_t sub = RAM_PTR8(0x20000496UL)[pos + 4U];
        switch (area) {
        case 2U:
            RAM_PTR8(0x20005ED8UL)[sub] = b0;
            RAM_PTR8(0x20005ED9UL)[sub] = b1;
            RAM_PTR8(0x20005EDAUL)[sub] = b2;
            RAM_PTR8(0x20005EDBUL)[sub] = b3;
            break;
        case 3U:
            RAM_PTR8(0x20005EE8UL)[sub] = b0;
            RAM_PTR8(0x20005EE9UL)[sub] = b1;
            RAM_PTR8(0x20005EEAUL)[sub] = b2;
            RAM_PTR8(0x20005EEBUL)[sub] = b3;
            break;
        case 4U:
            RAM_PTR8(0x20005EF8UL)[sub] = b0;
            RAM_PTR8(0x20005EF9UL)[sub] = b1;
            RAM_PTR8(0x20005EFAUL)[sub] = b2;
            RAM_PTR8(0x20005EFBUL)[sub] = b3;
            break;
        case 12U:
            RAM_PTR32(0x20005F18UL)[sub] = val;
            break;
        case 15U:
            RAM_PTR8(0x20005EB8UL)[sub] = b0;
            RAM_PTR8(0x20005EB9UL)[sub] = b1;
            RAM_PTR8(0x20005EBAUL)[sub] = b2;
            RAM_PTR8(0x20005EBBUL)[sub] = b3;
            break;
        case 0x70U:
            RAM_PTR8(0x20006110UL)[sub] = b0;
            RAM_PTR8(0x20006111UL)[sub] = b1;
            RAM_PTR8(0x20006112UL)[sub] = b2;
            RAM_PTR8(0x20006113UL)[sub] = b3;
            break;
        default:
            break;
        }
        RAM8(0x200001CCUL) = (uint8_t)(pos + 3U);
        return;
    }

    if (op_mode == 3U) {
        uint8_t ptr_area = RAM_PTR8(0x20000496UL)[pos + 3U];
        uint32_t ptr_val = 0;
        if (ptr_area == 1U) {
            uint32_t v_off = ((uint32_t)RAM_PTR8(0x20000496UL)[pos + 4U] << 8) |
                             RAM_PTR8(0x20000496UL)[pos + 5U];
            ptr_val = ((uint32_t)RAM_PTR8(0x2000108CUL)[v_off] << 24) |
                      ((uint32_t)RAM_PTR8(0x2000108DUL)[v_off] << 16) |
                      ((uint32_t)RAM_PTR8(0x2000108EUL)[v_off] << 8)  |
                      RAM_PTR8(0x2000108FUL)[v_off];
            RAM8(0x200001CCUL) = (uint8_t)(pos + 4U);
        } else if (ptr_area == 0x0CU) {
            uint8_t ac_idx = RAM_PTR8(0x20000496UL)[pos + 4U];
            if (ac_idx != 0) {
                ptr_val = RAM_PTR32(0x20005F18UL)[ac_idx];
            }
            RAM8(0x200001CCUL) = (uint8_t)(pos + 3U);
        } else if (ptr_area == 0x70U) {
            uint8_t l_off = RAM_PTR8(0x20000496UL)[pos + 4U];
            ptr_val = ((uint32_t)RAM_PTR8(0x20006110UL)[l_off] << 24) |
                      ((uint32_t)RAM_PTR8(0x20006111UL)[l_off] << 16) |
                      ((uint32_t)RAM_PTR8(0x20006112UL)[l_off] << 8)  |
                      RAM_PTR8(0x20006113UL)[l_off];
            RAM8(0x200001CCUL) = (uint8_t)(pos + 3U);
        }

        if (ptr_val >= 0x08005000UL) {
            if (ptr_val < 0x08009FFDUL) {
                uint32_t off = ptr_val - 0x08005000UL;
                RAM_PTR8(0x2000108CUL)[off] = b0;
                RAM_PTR8(0x2000108DUL)[off] = b1;
                RAM_PTR8(0x2000108EUL)[off] = b2;
                RAM_PTR8(0x2000108FUL)[off] = b3;
                return;
            }
        } else {
            uint32_t sub = ptr_val & 0xFFU;
            switch ((ptr_val >> 8) & 0xFFU) {
            case 0U:
                if (ptr_val < 0x0800000DUL) {
                    RAM_PTR8(0x20005ED8UL)[sub] = b0;
                    RAM_PTR8(0x20005ED9UL)[sub] = b1;
                    RAM_PTR8(0x20005EDAUL)[sub] = b2;
                    RAM_PTR8(0x20005EDBUL)[sub] = b3;
                    return;
                }
                break;
            case 1U:
                if (ptr_val < 0x0800010DUL) {
                    uint32_t off = (ptr_val - 0x08000100UL) & 0xFFFFU;
                    RAM_PTR8(0x20005EE8UL)[off] = b0;
                    RAM_PTR8(0x20005EE9UL)[off] = b1;
                    RAM_PTR8(0x20005EEAUL)[off] = b2;
                    RAM_PTR8(0x20005EEBUL)[off] = b3;
                    return;
                }
                break;
            case 2U:
                if (ptr_val < 0x0800021DUL) {
                    uint32_t off = (ptr_val - 0x08000200UL) & 0xFFFFU;
                    RAM_PTR8(0x20005EF8UL)[off] = b0;
                    RAM_PTR8(0x20005EF9UL)[off] = b1;
                    RAM_PTR8(0x20005EFAUL)[off] = b2;
                    RAM_PTR8(0x20005EFBUL)[off] = b3;
                    return;
                }
                break;
            case 3U:
            case 4U:
                if (ptr_val >= 0x0800031EUL && ptr_val < 0x08000429UL) {
                    uint32_t off = (ptr_val - 0x08000300UL) & 0xFFFFU;
                    RAM_PTR8(0x20005F28UL)[off] = b0;
                    RAM_PTR8(0x20005F29UL)[off] = b1;
                    RAM_PTR8(0x20005F2AUL)[off] = b2;
                    RAM_PTR8(0x20005F2BUL)[off] = b3;
                    return;
                }
                break;
            case 11U:
                if (ptr_val < 0x08000B1DUL) {
                    uint32_t off = (ptr_val - 0x08000B00UL) & 0xFFFFU;
                    RAM_PTR8(0x20005EB8UL)[off] = b0;
                    RAM_PTR8(0x20005EB9UL)[off] = b1;
                    RAM_PTR8(0x20005EBAUL)[off] = b2;
                    RAM_PTR8(0x20005EBBUL)[off] = b3;
                    return;
                }
                break;
            default:
                break;
            }
        }
        RAM8(0x20005F2CUL) |= 0x10U;
    }
}

/* ==========================================================================
 * Function: FUN_0803bb26 (SystemInit_HSE_PLL_96MHz) @ 0x0803bb26 (172 bytes)
 *
 * Resets RCC configuration and initializes the APM32E103 / STM32F103 clock tree:
 *   - Enables 8 MHz HSE crystal oscillator and waits for HSERDY
 *   - Enables Flash Prefetch Buffer and sets 3 Wait States (FLASH_ACR)
 *   - Configures AHB = SYSCLK, APB2 = HCLK, APB1 = HCLK/2, PLLSRC = HSE,
 *     PLLMUL = x12 (RCC_CFGR |= 0x00290000 -> 8 MHz * 12 = 96 MHz)
 *   - Enables PLL, waits for PLLRDY, and switches SYSCLK to PLL
 * ========================================================================== */
void FUN_0803bb26(void)
{
    RCC_CR   |= 0x00000001UL; /* HSION = 1 */
    RCC_CFGR &= 0xF8FF0000UL;
    RCC_CR   &= 0xFEF6FFFFUL; /* Clear HSEON, CSSON, PLLON */
    RCC_CR   &= 0xFFFBFFFFUL; /* Clear HSEBYP */
    RCC_CFGR &= 0xFF80FFFFUL;
    RCC_CIR   = 0x009F0000UL; /* Clear all RCC interrupt flags */

    /* Enable HSE */
    RCC_CR |= 0x00010000UL;
    uint32_t startup_cnt = 0;
    uint32_t hse_rdy = 0;
    do {
        hse_rdy = RCC_CR & 0x00020000UL;
        startup_cnt++;
    } while (hse_rdy == 0 && startup_cnt != 0x0500UL);

    if ((RCC_CR & 0x00020000UL) != 0) {
        /* Enable Prefetch Buffer (0x10) and set 3 Flash Wait States (0x03) */
        FLASH_ACR |= 0x00000010UL;
        FLASH_ACR  = (FLASH_ACR & 0xFFFFFFF8UL) | 0x00000003UL;

        /* HCLK = SYSCLK, PCLK2 = HCLK, PCLK1 = HCLK/2 (0x0400) */
        RCC_CFGR |= 0x00000000UL;
        RCC_CFGR |= 0x00000000UL;
        RCC_CFGR |= 0x00000400UL;

        /* PLLSRC = HSE (0x010000), PLLMUL = x12 (0x280000 -> 0x290000) */
        RCC_CFGR = (RCC_CFGR & 0xFFC0FFFFUL) | 0x00290000UL;

        /* Enable PLL and wait for lock */
        RCC_CR |= 0x01000000UL;
        while ((RCC_CR & 0x02000000UL) == 0) {
        }

        /* Select PLL as system clock source (SW = 10) */
        RCC_CFGR = (RCC_CFGR & 0xFFFFFFFCUL) | 0x00000002UL;
        while ((RCC_CFGR & 0x0000000CUL) != 0x00000008UL) {
        }
    } else {
        /* HSE failed to start: trap in infinite loop */
        while (true) {
        }
    }
}

/* ==========================================================================
 * Function: FUN_0803bbd2 (PLC_WriteWordOperand) @ 0x0803bbd2 (850 bytes)
 *
 * Writes a 16-bit value `val` to the PLC operand encoded at `0x200001CC`
 * (supports V, I, Q, M, SM, T, C, AQ, AC0..AC3, S, and L memory areas).
 * ========================================================================== */
void FUN_0803bbd2(uint16_t val)
{
    uint32_t pos = RAM8(0x200001CCUL);
    uint8_t  op_mode = RAM_PTR8(0x20000496UL)[pos + 2U];
    uint8_t  hi = (uint8_t)(val >> 8);
    uint8_t  lo = (uint8_t)val;

    if (op_mode == 2U) {
        uint8_t area = RAM_PTR8(0x20000496UL)[pos + 3U];
        if (area == 1U) {
            uint32_t off = ((uint32_t)RAM_PTR8(0x20000496UL)[pos + 4U] << 8) |
                           RAM_PTR8(0x20000496UL)[pos + 5U];
            RAM_PTR8(0x2000108CUL)[off] = hi;
            RAM_PTR8(0x2000108DUL)[off] = lo;
            RAM8(0x200001CCUL) = (uint8_t)(pos + 4U);
            return;
        }
        if (area == 5U) {
            uint32_t off = ((uint32_t)RAM_PTR8(0x20000496UL)[pos + 4U] << 8) |
                           RAM_PTR8(0x20000496UL)[pos + 5U];
            RAM_PTR8(0x20005F28UL)[off] = hi;
            RAM_PTR8(0x20005F29UL)[off] = lo;
            RAM8(0x200001CCUL) = (uint8_t)(pos + 4U);
            return;
        }

        uint32_t sub = RAM_PTR8(0x20000496UL)[pos + 4U];
        switch (area) {
        case 2U:
            RAM_PTR8(0x20005ED8UL)[sub] = hi;
            RAM_PTR8(0x20005ED9UL)[sub] = lo;
            break;
        case 3U:
            RAM_PTR8(0x20005EE8UL)[sub] = hi;
            RAM_PTR8(0x20005EE9UL)[sub] = lo;
            break;
        case 4U:
            RAM_PTR8(0x20005EF8UL)[sub] = hi;
            RAM_PTR8(0x20005EF9UL)[sub] = lo;
            break;
        case 8U:
            RAM_PTR16(0x20006354UL)[sub] = val;
            break;
        case 9U:
            RAM_PTR16(0x20006954UL)[sub] = val;
            break;
        case 11U:
            RAM_PTR16(0x20000830UL)[sub >> 1] = val;
            break;
        case 12U:
            RAM_PTR32(0x20005F18UL)[sub] =
                (RAM_PTR32(0x20005F18UL)[sub] & 0xFFFF0000UL) | val;
            break;
        case 15U:
            RAM_PTR8(0x20005EB8UL)[sub] = hi;
            RAM_PTR8(0x20005EB9UL)[sub] = lo;
            break;
        case 0x70U:
            RAM_PTR8(0x20006110UL)[sub] = hi;
            RAM_PTR8(0x20006111UL)[sub] = lo;
            break;
        default:
            break;
        }
        RAM8(0x200001CCUL) = (uint8_t)(pos + 3U);
        return;
    }

    if (op_mode == 3U) {
        uint8_t ptr_area = RAM_PTR8(0x20000496UL)[pos + 3U];
        uint32_t ptr_val = 0;
        if (ptr_area == 1U) {
            uint32_t v_off = ((uint32_t)RAM_PTR8(0x20000496UL)[pos + 4U] << 8) |
                             RAM_PTR8(0x20000496UL)[pos + 5U];
            ptr_val = ((uint32_t)RAM_PTR8(0x2000108CUL)[v_off] << 24) |
                      ((uint32_t)RAM_PTR8(0x2000108DUL)[v_off] << 16) |
                      ((uint32_t)RAM_PTR8(0x2000108EUL)[v_off] << 8)  |
                      RAM_PTR8(0x2000108FUL)[v_off];
            RAM8(0x200001CCUL) = (uint8_t)(pos + 4U);
        } else if (ptr_area == 0x0CU) {
            uint8_t ac_idx = RAM_PTR8(0x20000496UL)[pos + 4U];
            if (ac_idx != 0) {
                ptr_val = RAM_PTR32(0x20005F18UL)[ac_idx];
            }
            RAM8(0x200001CCUL) = (uint8_t)(pos + 3U);
        } else if (ptr_area == 0x70U) {
            uint8_t l_off = RAM_PTR8(0x20000496UL)[pos + 4U];
            ptr_val = ((uint32_t)RAM_PTR8(0x20006110UL)[l_off] << 24) |
                      ((uint32_t)RAM_PTR8(0x20006111UL)[l_off] << 16) |
                      ((uint32_t)RAM_PTR8(0x20006112UL)[l_off] << 8)  |
                      RAM_PTR8(0x20006113UL)[l_off];
            RAM8(0x200001CCUL) = (uint8_t)(pos + 3U);
        }

        if (ptr_val >= 0x08005000UL) {
            if (ptr_val < 0x08009FFFUL) {
                uint32_t off = ptr_val - 0x08005000UL;
                RAM_PTR8(0x2000108CUL)[off] = hi;
                RAM_PTR8(0x2000108DUL)[off] = lo;
                return;
            }
        } else {
            uint32_t sub = ptr_val & 0xFFU;
            switch ((ptr_val >> 8) & 0xFFU) {
            case 0U:
                if (ptr_val < 0x0800000FUL) {
                    RAM_PTR8(0x20005ED8UL)[sub] = hi;
                    RAM_PTR8(0x20005ED9UL)[sub] = lo;
                    return;
                }
                break;
            case 1U:
                if (ptr_val < 0x0800010FUL) {
                    uint32_t off = (ptr_val - 0x08000100UL) & 0xFFFFU;
                    RAM_PTR8(0x20005EE8UL)[off] = hi;
                    RAM_PTR8(0x20005EE9UL)[off] = lo;
                    return;
                }
                break;
            case 2U:
                if (ptr_val < 0x0800021FUL) {
                    uint32_t off = (ptr_val - 0x08000200UL) & 0xFFFFU;
                    RAM_PTR8(0x20005EF8UL)[off] = hi;
                    RAM_PTR8(0x20005EF9UL)[off] = lo;
                    return;
                }
                break;
            case 3U:
            case 4U:
                if (ptr_val >= 0x0800031EUL && ptr_val < 0x0800042BUL) {
                    uint32_t off = (ptr_val - 0x08000300UL) & 0xFFFFU;
                    RAM_PTR8(0x20005F28UL)[off] = hi;
                    RAM_PTR8(0x20005F29UL)[off] = lo;
                    return;
                }
                break;
            case 6U:
                if ((ptr_val & 1U) == 0 && ptr_val < 0x0800063FUL) {
                    RAM_PTR16(0x20000830UL)[((ptr_val - 0x08000600UL) & 0xFFFFU) >> 1] = val;
                    return;
                }
                break;
            case 7U:
            case 8U:
                if (ptr_val < 0x08000800UL) {
                    RAM_PTR16(0x20006354UL)[(ptr_val - 0x08000700UL) & 0xFFFFU] = val;
                    return;
                }
                break;
            case 9U:
            case 10U:
                if (ptr_val < 0x08000A00UL) {
                    RAM_PTR16(0x20006954UL)[(ptr_val - 0x08000900UL) & 0xFFFFU] = val;
                    return;
                }
                break;
            case 11U:
                if (ptr_val < 0x08000B1FUL) {
                    uint32_t off = (ptr_val - 0x08000B00UL) & 0xFFFFU;
                    RAM_PTR8(0x20005EB8UL)[off] = hi;
                    RAM_PTR8(0x20005EB9UL)[off] = lo;
                    return;
                }
                break;
            default:
                break;
            }
        }
        RAM8(0x20005F2CUL) |= 0x10U;
    }
}

/* External declarations for UART/Timer reset helpers called by FUN_0803bf24 */
extern void FUN_080143c8(uint8_t port_source, uint8_t pin_source);
extern void FUN_08016ab8(const uint8_t *nvic_init);
extern void FUN_080401b4(void);
extern void FUN_0804211c(void);
extern void FUN_080421f0(void);

/* Lookup table at 0x0804A2A2 mapping 8-bit Expansion Module ID code (index)
 * to expected 2-bit status/type class returned on the expansion bus */
#define EXP_MOD_CLASS_TABLE FLASH_PTR8(0x0804A2A2UL)

/* ==========================================================================
 * Function: FUN_0803bf24 (PLC_SystemReset_ParseSDB_EnumExpansionBus)
 *           @ 0x0803bf24 .. 0x0803dfdd (8378 bytes)
 *
 * Master PLC initialization, System Data Block (SDB) parser, Expansion Module
 * Bus enumerator, Retentive Flash restorer, and User Program/DB1 indexer:
 *   1. Resets all V, I, Q, M, S, SM, L, Timer, Counter, HSC, and PTO state.
 *   2. Copies the SDB image from Flash `0x08075800` into RAM `0x200058CC`
 *      (applying nibble de-obfuscation per `0x08075802` mode), validates CRC,
 *      and extracts CPU configuration:
 *        - Port 0 & Port 1 PPI/Modbus station addresses & baudrates
 *        - Default/Substitute Digital Output states (Q0..Q15) & freeze masks
 *        - Default/Substitute Analog Output values (AQW0..AQW31) & freeze masks
 *        - Retentive Memory Ranges (up to 6 ranges across V, M, T, C)
 *        - Digital Input Filter time constants (I0.0..I1.7) & Pulse Catch bits
 *        - Analog Input Filter enable mask & sample counts
 *        - Background communication time slice percentage (`SMB28` / `0x2000017A`)
 *   3. Disables all user EXTI lines and HSC/PTO timers, resets interrupt queues.
 *   4. Pulses the Expansion Module Bus reset/strobe lines (PC8..PC11) and
 *      enumerates up to 7 Expansion Modules (Slots 0..6) via `FUN_08012aac`
 *      and `FUN_08013346`, populating `SMB8..SMB21` (`0x20005F30+`) and module
 *      I/O byte/word allocation tables (`0x20000356`, `0x2000035D`, `0x20000364`,
 *      `0x20000462`).
 *   5. Restores retentive V/M/T/C data from Flash sector `0x08069000` if valid.
 *   6. Scans User Program block (`0x0806D800`) to build Subroutine (`SBR_0..SBR_n`
 *      at `0x20000C1C`) and Interrupt Routine (`INT_0..INT_n` at `0x20000B8E`)
 *      entry point tables, and loads Data Block (`DB1` at `0x08073000`) initial
 *      values into V-memory.
 * ========================================================================== */
void FUN_0803bf24(void)
{
    uint16_t i;
    uint16_t j;

    RAM8(0x200001C8UL) = 0;

    /* 1. Clear V-Memory (0x2000108C, 18496 bytes) */
    for (i = 0; i < 0x4840U; i++) {
        RAM_PTR8(0x2000108CUL)[i] = 0;
    }

    /* 2. Clear I, Q, M, S, and AI/AQ process image buffers */
    for (i = 0; i < 16U; i++) {
        RAM_PTR8(0x20005ED8UL)[i] = 0; /* I0.0 .. I15.7 */
        RAM_PTR8(0x20005EE8UL)[i] = 0; /* Q0.0 .. Q15.7 */
        RAM_PTR8(0x2000046EUL)[i] = 0; /* Q freeze mask */
        RAM_PTR8(0x20000476UL)[i] = 0; /* Q substitute table */
        RAM_PTR8(0x200004AEUL)[i] = 0;
        RAM_PTR8(0x200004D8UL)[i] = 0;
    }
    for (i = 0; i < 32U; i++) {
        RAM_PTR8(0x20005EF8UL)[i]  = 0; /* M0.0 .. M31.7 */
        RAM_PTR8(0x20005EB8UL)[i]  = 0; /* S0.0 .. S31.7 */
        RAM_PTR16(0x200007E8UL)[i] = 0; /* AIW0 .. AIW62 */
        RAM_PTR16(0x20000830UL)[i] = 0; /* AQW0 .. AQW62 */
        RAM_PTR16(0x20000520UL)[i] = 0; /* AQ substitute values */
    }
    RAM32(0x20000518UL) = 0;

    /* 3. Clear Accumulators AC0..AC3 and SM memory (SMB0..SMB299) */
    for (i = 0; i < 4U; i++) {
        RAM_PTR32(0x20005F18UL)[i] = 0;
        RAM_PTR32(0x2000BE04UL)[i] = 0;
    }
    for (i = 0; i < 300U; i++) {
        RAM_PTR8(0x20005F28UL)[i] = 0;
    }

    /* Set initial system status bits in SMB0..SMB7 */
    RAM8(0x20005F28UL) = 0x03U; /* SM0.0 = 1 (Always ON), SM0.1 = 1 (First Scan) */
    RAM8(0x20005F2FUL) = 0x01U;

    /* 4. Clear Local (L) memory frames for OB1, SBRs, and INTs */
    for (i = 0; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i]       = 0;
        RAM_PTR8(0x20006110UL)[i + 64U] = 0;
        RAM_PTR8(0x2000BB98UL)[i]       = 0;
    }
    for (i = 0; i < 0x1100U; i++) {
        RAM_PTR8(0x2000BBD8UL)[i] = 0;
    }

    /* 5. Clear Timers (T0..T255) and Counters (C0..C255) */
    for (i = 0; i < 256U; i++) {
        RAM_PTR16(0x20006354UL)[i] = 0; /* Timer current value */
        RAM_PTR8(0x20006554UL)[i]  = 0; /* Timer status bits */
        RAM_PTR32(0x20006754UL)[i] = 0;
        RAM_PTR16(0x20006954UL)[i] = 0; /* Counter current value */
        RAM_PTR8(0x20006B54UL)[i]  = 0; /* Counter status bits */
    }
    for (i = 0; i < 32U; i++) {
        RAM_PTR8(0x20005ED0UL)[i] = 0;
        RAM_PTR8(0x20005EC8UL)[i] = 0;
    }

    /* 6. Clear High-Speed Counters (HC0..HC5) and stop hardware timers */
    for (i = 0; i < 6U; i++) {
        RAM_PTR32(0x20006150UL)[i] = 0;
        RAM_PTR32(0x200061ACUL)[i] = 0;
    }
    TIM1_CR1  &= 0xFFFEU;
    TIM1_BDTR &= 0x7FFFU;
    TIM1_DIER &= 0xFFFEU;
    TIM1_SR    = 0xFFFEU;
    TIM1_CNT   = 0;

    TIM2_CR1  &= 0xFFFEU;
    TIM2_DIER &= 0xFFFEU;
    TIM2_SR    = 0xFFFEU;
    TIM2_CNT   = 0;

    TIM8_CR1  &= 0xFFFEU;
    TIM8_BDTR &= 0x7FFFU;
    TIM8_DIER &= 0xFFFEU;
    TIM8_SR    = 0xFFFEU;
    TIM8_CNT   = 0;

    TIM5_CR1  &= 0xFFFEU;
    TIM5_DIER &= 0xFFFEU;
    TIM5_SR    = 0xFFFEU;
    TIM5_CNT   = 0;

    TIM3_CR1  &= 0xFFFEU;
    TIM3_CNT   = 0;
    TIM4_CR1  &= 0xFFFEU;
    TIM4_CNT   = 0;

    TIM6_CR1  &= 0xFFFEU;
    TIM6_DIER &= 0xFFFEU;
    TIM6_SR    = 0xFFFEU;
    TIM6_CNT   = 0;

    TIM7_CR1  &= 0xFFFEU;
    TIM7_DIER &= 0xFFFEU;
    TIM7_SR    = 0xFFFEU;
    TIM7_CNT   = 0;

    /* 7. Initialize Default System Configuration before SDB Parse */
    RAM8(0x20000172UL) = 2U; /* Port 0 Default Station Address = 2 */
    RAM8(0x20000173UL) = 2U; /* Port 1 Default Station Address = 2 */
    RAM8(0x20000174UL) = 2U; /* Port 0 Default Baudrate Code = 2 (9.6 kbps) */
    RAM8(0x20000175UL) = 2U; /* Port 1 Default Baudrate Code = 2 (9.6 kbps) */
    RAM8(0x2000017AUL) = 10U;

    /* Set default Retentive Ranges (V0..V4095, C0..C31, T0..T31) */
    RAM16(0x200003A0UL) = 0U;
    RAM16(0x200003ACUL) = 4096U;
    for (i = 1U; i < 6U; i++) {
        RAM_PTR16(0x200003A0UL)[i] = 0;
        RAM_PTR16(0x200003ACUL)[i] = 0;
        RAM_PTR8(0x200003B8UL)[i]  = 0;
    }
    RAM8(0x200003B8UL) = 1U;

    /* 8. Parse System Data Block (SDB) from Flash 0x08075800 if present */
    uint16_t sdb_len = ((uint16_t)FLASH_PTR8(0x08075800UL)[0] << 8) |
                       FLASH_PTR8(0x08075800UL)[1];
    uint8_t  sdb_obf = FLASH_PTR8(0x08075800UL)[2];

    if (sdb_len != 0xFFFFU && sdb_len > 0U && sdb_len <= 0x0600U) {
        for (i = 0; i < sdb_len; i++) {
            uint8_t raw = FLASH_PTR8(0x08075804UL)[i];
            if (sdb_obf == 1U) {
                raw = (uint8_t)FUN_08014972(raw);
            } else if (sdb_obf == 2U) {
                raw = (uint8_t)FUN_080149ea(raw);
            } else if (sdb_obf == 3U) {
                raw = (uint8_t)FUN_08014a5a(raw);
            } else if (sdb_obf == 4U) {
                raw = (uint8_t)FUN_08014e46(raw);
            }
            RAM_PTR8(0x200058CCUL)[i] = raw;
        }

        /* Extract Port 0 & Port 1 station addresses and baudrates from SDB */
        uint8_t p0_addr = RAM_PTR8(0x200058CCUL)[42];
        RAM8(0x20000172UL) = (p0_addr == 0U || p0_addr > 126U) ? 2U : p0_addr;

        uint8_t p1_addr = RAM_PTR8(0x200058CCUL)[45];
        RAM8(0x20000173UL) = (p1_addr == 0U || p1_addr > 126U) ? 2U : p1_addr;

        uint8_t p0_baud = RAM_PTR8(0x200058CCUL)[43];
        if (p0_baud != RAM8(0x20000174UL)) {
            RAM8(0x20000174UL) = p0_baud;
            FUN_0804211c();
        }

        uint8_t p1_baud = RAM_PTR8(0x200058CCUL)[47];
        if (p1_baud != RAM8(0x20000175UL)) {
            RAM8(0x20000175UL) = p1_baud;
            FUN_080421f0();
        }
    }

    /* 9. Reset Freeport/Modbus state and disable all user EXTI lines */
    RAM8(0x20000487UL) = 0;
    RAM8(0x20000488UL) = 0;
    RAM8(0x20000489UL) = 0;
    RAM8(0x2000048AUL) = 0;
    RAM8(0x20000493UL) = 0;
    RAM8(0x20000156UL) = 0;
    RAM8(0x20000155UL) = 0;
    RAM8(0x20000157UL) = 0;
    RAM8(0x2000048CUL) = 0;
    RAM8(0x20000494UL) = 0;
    RAM8(0x20000486UL) = 0;
    RAM8(0x20005F2CUL) &= 0xE0U;
    RAM8(0x200003DCUL) = 0;

    for (i = 0; i < 36U; i++) {
        RAM_PTR8(0x20000B5AUL)[i] = 0;
    }
    for (i = 0; i < 4U; i++) {
        RAM_PTR8(0x200003C0UL)[i] = 0;
    }
    for (i = 0; i < 30U; i++) {
        RAM_PTR8(0x20000B7EUL)[i] = 0;
    }

    uint32_t exti_cfg[2] = { 0, 0 };
    uint8_t  nvic_cfg[4] = { 0, 0, 0, 0 };

    FUN_080143c8(3U, 12U);
    exti_cfg[0] = 0x1000UL;
    FUN_08012a22(exti_cfg);

    FUN_080143c8(3U, 13U);
    exti_cfg[0] = 0x2000UL;
    FUN_08012a22(exti_cfg);

    FUN_080143c8(4U, 0U);
    exti_cfg[0] = 0x0001UL;
    FUN_08012a22(exti_cfg);

    FUN_080143c8(2U, 6U);
    exti_cfg[0] = 0x0040UL;
    FUN_08012a22(exti_cfg);

    nvic_cfg[0] = 23U; /* EXTI9_5_IRQn */
    FUN_08016ab8(nvic_cfg);
    nvic_cfg[0] = 6U;  /* EXTI0_IRQn */
    FUN_08016ab8(nvic_cfg);
    nvic_cfg[0] = 40U; /* EXTI15_10_IRQn */
    FUN_08016ab8(nvic_cfg);

    FUN_080143c8(4U, 1U);
    exti_cfg[0] = 0x0002UL;
    FUN_08012a22(exti_cfg);

    FUN_080143c8(2U, 7U);
    exti_cfg[0] = 0x0080UL;
    FUN_08012a22(exti_cfg);

    nvic_cfg[0] = 7U;  /* EXTI1_IRQn */
    FUN_08016ab8(nvic_cfg);

    /* Clear interrupt queue pointers and counts */
    for (i = 0; i < 10U; i++) {
        RAM_PTR8(0x200003E1UL)[i] = 0;
    }
    for (i = 0; i < 6U; i++) {
        RAM_PTR8(0x20000338UL)[i] = 0;
        RAM_PTR8(0x2000033EUL)[i] = 0;
        RAM_PTR8(0x200003D0UL)[i] = 0;
        RAM_PTR8(0x200003CAUL)[i] = 0;
        RAM_PTR8(0x20000350UL)[i] = 0;
        RAM_PTR8(0x20000344UL)[i] = 0;
        RAM_PTR8(0x200003D6UL)[i] = 0;
        RAM_PTR8(0x2000034AUL)[i] = 0;
    }

    RAM8(0x200003B8UL) = 0;
    FUN_080401b4();

    RAM8(0x2000002AUL) = 0;
    RAM8(0x2000002BUL) = 0;
    RAM8(0x2000002CUL) = 0;
    RAM8(0x2000002DUL) = 0;
    RAM8(0x2000002EUL) = 0;
    RAM8(0x2000002FUL) = 0;
    RAM8(0x20000030UL) = 0;
    RAM8(0x20000031UL) = 0;
    RAM8(0x20000032UL) = 0;

    /* Reset PTO0 / PTO1 and PWM runtime state */
    RAM8(0x20000440UL)  = 0;
    RAM8(0x20000441UL)  = 0;
    RAM8(0x20000194UL)  = 0;
    RAM32(0x2000009CUL) = 0;
    RAM32(0x20000388UL) = 0;
    RAM32(0x2000038CUL) = 0;
    RAM8(0x20000198UL)  = 0;
    RAM32(0x200000A8UL) = 0;
    RAM32(0x200002E4UL) = 0;
    RAM8(0x20000199UL)  = 1U;
    RAM8(0x20000195UL)  = 0;
    RAM32(0x200000B0UL) = 0;
    RAM32(0x200000ACUL) = 0;
    RAM8(0x2000019AUL)  = 0;
    RAM8(0x20000196UL)  = 0;
    RAM8(0x20000197UL)  = 0;
    RAM8(0x20000428UL)  = 0;
    RAM8(0x20005FD9UL)  = 0;
    RAM32(0x200002B8UL) = 0;

    RAM8(0x20000404UL)  = 0;
    RAM8(0x20000405UL)  = 0;
    RAM8(0x2000018DUL)  = 0;
    RAM32(0x20000084UL) = 0;
    RAM32(0x20000380UL) = 0;
    RAM32(0x20000384UL) = 0;
    RAM8(0x20000191UL)  = 0;
    RAM32(0x20000090UL) = 0;
    RAM32(0x200002D8UL) = 0;
    RAM8(0x20000192UL)  = 1U;
    RAM8(0x2000018EUL)  = 0;
    RAM32(0x20000098UL) = 0;
    RAM32(0x20000094UL) = 0;
    RAM8(0x20000193UL)  = 0;
    RAM8(0x2000018FUL)  = 0;
    RAM8(0x20000190UL)  = 0;
    RAM8(0x200003EDUL)  = 0;
    RAM8(0x20005FCFUL)  = 0;
    RAM32(0x200002B4UL) = 0;
    RAM8(0x2000018AUL)  = 1U;
    RAM8(0x20000189UL)  = 1U;

    for (i = 0; i < 8U; i++) {
        RAM_PTR8(0x200007E0UL)[i]  = 0;
        RAM_PTR32(0x2000BE14UL)[i] = 0;
    }

    RAM8(0x2000016EUL) = 0;
    uint32_t pwr_chk = FUN_08001070();
    RAM8(0x20000495UL) = 0;
    RAM8(0x2000051CUL) = 0;
    if (pwr_chk == 0 || RAM16(0x200001ECUL) != 0) {
        RAM8(0x200001C8UL) = 0;
    } else {
        RAM8(0x200001C8UL) = 1U;
    }

    RAM16(0x200003BAUL) = 0;
    for (i = 0; i < 64U; i++) {
        RAM_PTR8(0x20004E40UL)[i] = 0;
    }

    /* ======================================================================
     * 10. Expansion Module Bus Reset & Slot 0..6 Enumeration
     * ====================================================================== */
    for (i = 0; i < 20U; i++) {
        FUN_080043b8(80000UL);
    }

    if (RAM8(0x2000051DUL) != 0 || RAM8(0x200001C7UL) == 0) {
        RAM8(0x2000051DUL) = 0;
        FUN_08014526(GPIOC_BASE, 0x0300U); /* PC8, PC9 = 0 */
        FUN_0801452a(GPIOC_BASE, 0x0800U); /* PC11 = 1 */
        for (i = 0; i < 100U; i++) {
            FUN_080043b8(80000UL);
        }
        FUN_080043b8(5000000UL);
        FUN_080043b8(5000000UL);

        BITBAND_GPIOC_ODR_PC9 = 1UL;
        FUN_080043b8(8000UL);
        BITBAND_GPIOC_ODR_PC11 = 1UL;
        FUN_080043b8(40UL);
        GPIOC_CRH = (GPIOC_CRH & 0xFFFFF0FFUL) | 0x00000300UL;
        BITBAND_GPIOC_ODR_PC10 = 0UL;
        BITBAND_GPIOC_ODR_PC8  = 0UL;

        RAM8(0x200001A3UL) = 0;
        RAM8(0x200001A2UL) = 0;
        RAM8(0x2000046DUL) = 0;
        RAM8(0x20000469UL) = 3U; /* Next Digital Input byte offset = IB3 (after onboard IB0..IB2) */
        RAM8(0x2000046AUL) = 2U; /* Next Digital Output byte offset = QB2 (after onboard QB0..QB1) */
        RAM8(0x2000046BUL) = 0U; /* Next Analog Input word offset = AIW0 */
        RAM8(0x2000046CUL) = 0U; /* Next Analog Output word offset = AQW0 */

        for (i = 0; i < 7U; i++) {
            RAM_PTR8(0x20000462UL)[i] = 7U;
            RAM_PTR8(0x20000356UL)[i] = 0;
            RAM_PTR8(0x2000035DUL)[i] = 0;
            RAM_PTR8(0x20000364UL)[i] = 0;
            RAM_PTR8(0x20005F30UL)[i * 2U]      = 0xFFU; /* SMB8, 10, 12, 14, 16, 18, 20 */
            RAM_PTR8(0x20005F30UL)[i * 2U + 1U] = 0U;    /* SMB9, 11, 13, 15, 17, 19, 21 */
        }

        /* Probe Slots 0..6 with initial 16-bit sync command (0x1200 | (slot << 13)) */
        for (RAM16(0x200001FCUL) = 0; RAM16(0x200001FCUL) < 7U; RAM16(0x200001FCUL)++) {
            for (uint16_t retry = 0; retry < 11U; retry++) {
                uint16_t slot = RAM16(0x200001FCUL);
                uint16_t cmd  = (uint16_t)(0x1200U | ((slot & 0xFFU) << 13));
                RAM16(0x20000200UL) = cmd;
                uint16_t ack = (uint16_t)FUN_08012aac(cmd, 25U);
                RAM16(0x200001FEUL) = ack;
                if (ack == 1U || ack == 5U) {
                    RAM_PTR8(0x20000462UL)[slot] = (uint8_t)ack;
                    break;
                }
            }
            FUN_080043b8(10UL);
        }

        FUN_080043b8(20UL);

        /* Query module ID & I/O configuration for each Slot 0..6 */
        for (RAM16(0x200001FCUL) = 0; RAM16(0x200001FCUL) < 7U; ) {
            uint16_t slot = RAM16(0x200001FCUL);
            uint8_t  cmd8 = (uint8_t)((slot & 0xFFU) << 5);
            RAM8(0x2000019BUL)  = cmd8;
            uint16_t resp = (uint16_t)FUN_08013346(cmd8, 24U);
            RAM16(0x200001FEUL) = resp;

            uint8_t mod_id   = (uint8_t)((resp >> 2) & 0xFFU);
            uint8_t mod_stat = (uint8_t)(resp & 0x03U);
            RAM8(0x2000019CUL) = mod_id;
            RAM8(0x2000019DUL) = mod_stat;

            if (EXP_MOD_CLASS_TABLE[mod_id] != mod_stat) {
                if (mod_id != 0xFFU) {
                    RAM8(0x2000046DUL) = 1U;
                    goto exp_slot_next;
                }
            } else if (mod_id < 16U) {
                /* Digital Expansion Module (EM221 / EM222 / EM223) */
                RAM_PTR8(0x20000356UL)[slot] = mod_id;
                cmd8 = (uint8_t)(0x01U | ((slot & 0xFFU) << 5));
                RAM8(0x2000019BUL)  = cmd8;
                resp = (uint16_t)FUN_08013346(cmd8, 24U);
                RAM16(0x200001FEUL) = resp;
                RAM8(0x2000019EUL)  = (uint8_t)(resp >> 2);
                RAM8(0x2000019FUL)  = (uint8_t)(resp & 0x03U);
                goto exp_slot_advance;
            }

            if ((uint8_t)(mod_id - 16U) < 16U) {
                /* Analog Expansion Module (EM231 / EM232 / EM235) */
                cmd8 = (uint8_t)(0x01U | ((slot & 0xFFU) << 5));
                RAM8(0x2000019BUL)  = cmd8;
                resp = (uint16_t)FUN_08013346(cmd8, 24U);
                RAM16(0x200001FEUL) = resp;
                RAM8(0x2000019EUL)  = (uint8_t)(resp >> 2);
                RAM8(0x2000019FUL)  = (uint8_t)(resp & 0x03U);

                cmd8 = (uint8_t)(0x02U | ((slot & 0xFFU) << 5));
                RAM8(0x2000019BUL)  = cmd8;
                resp = (uint16_t)FUN_08013346(cmd8, 24U);
                RAM16(0x200001FEUL) = resp;
                uint8_t sub_id   = (uint8_t)((resp >> 2) & 0xFFU);
                uint8_t sub_stat = (uint8_t)(resp & 0x03U);
                RAM8(0x2000019EUL) = sub_id;
                RAM8(0x2000019FUL) = sub_stat;

                if (EXP_MOD_CLASS_TABLE[sub_id] != sub_stat) {
                    RAM8(0x2000046DUL) = 1U;
                    goto exp_slot_next;
                }

                uint8_t merged_id = RAM8(0x2000019CUL);
                if (sub_id == 0x3CU) {
                    merged_id |= 0x20U;
                } else if (sub_id == 0xF0U) {
                    merged_id |= 0x40U;
                }
                RAM_PTR8(0x20000356UL)[slot] = merged_id;

                cmd8 = (uint8_t)(0x03U | ((slot & 0xFFU) << 5));
                RAM8(0x2000019BUL)  = cmd8;
                resp = (uint16_t)FUN_08013346(cmd8, 24U);
                RAM16(0x200001FEUL) = resp;
                RAM8(0x2000019FUL)  = (uint8_t)(resp & 0x03U);

                uint8_t aq_cfg = (merged_id >> 2) & 0x03U;
                RAM8(0x2000019EUL) = aq_cfg;
                if (merged_id < 0x40U) {
                    if (aq_cfg != 0U) {
                        uint16_t cmd16 = (uint16_t)(0x1200U | (slot << 13));
                        RAM16(0x20000200UL) = cmd16;
                        RAM16(0x200001FEUL) = (uint16_t)FUN_08012aac(cmd16, 25U);
                        FUN_080043b8(20UL);

                        cmd16 = (uint16_t)((slot << 13) | ((uint16_t)RAM8(0x2000019EUL) << 8));
                        RAM16(0x20000200UL) = cmd16;
                        RAM16(0x200001FEUL) = (uint16_t)FUN_08012aac(cmd16, 25U);
                        FUN_080043b8(20UL);

                        cmd16 = (uint16_t)(0x0400U | (slot << 13) | ((uint16_t)RAM8(0x2000019EUL) << 8));
                        RAM16(0x20000200UL) = cmd16;
                        RAM16(0x200001FEUL) = (uint16_t)FUN_08012aac(cmd16, 25U);
                        FUN_080043b8(20UL);
                    }

                    uint8_t ai_cfg = merged_id & 0x03U;
                    RAM8(0x2000019EUL) = ai_cfg;
                    if (ai_cfg != 0U) {
                        uint8_t ch_cnt = (ai_cfg == 3U) ? 8U : (uint8_t)(ai_cfg * 2U);
                        for (uint8_t ch = 0; ch < ch_cnt; ch++) {
                            cmd8 = (uint8_t)((slot << 5) | (ch * 4U));
                            RAM8(0x2000019BUL)  = cmd8;
                            resp = (uint16_t)FUN_08013346(cmd8, 24U);
                            RAM16(0x200001FEUL) = resp;
                            RAM8(0x2000019FUL)  = (uint8_t)(resp & 0x03U);
                            FUN_080043b8(20UL);
                        }
                    }
                }
            } else if ((uint8_t)(mod_id - 0x40U) < 0x40U) {
                /* Intelligent / Position Control Module (EM253 / CP243) */
                RAM_PTR8(0x20000356UL)[slot] = mod_id;
            } else {
                RAM_PTR8(0x20000356UL)[slot] = 0;
            }

        exp_slot_next:
            if (RAM8(0x2000046DUL) != 0) {
                RAM8(0x2000046DUL) = 0;
                uint8_t retries = (uint8_t)(RAM8(0x200001A2UL) + 1U);
                RAM8(0x200001A2UL) = retries;
                if (retries < 11U) {
                    continue;
                }
                RAM_PTR8(0x20000356UL)[RAM16(0x200001FCUL)] = 0;
            }
            RAM8(0x200001A2UL) = 0;
            if (RAM_PTR8(0x20000356UL)[RAM16(0x200001FCUL)] == 0) {
                break;
            }

        exp_slot_advance:
            FUN_080043b8(20UL);
            RAM16(0x200001FCUL) = (uint16_t)(RAM16(0x200001FCUL) + 1U);
        }

        /* Populate SMB8..SMB21 module ID and I/O allocation registers */
        for (i = 0; i < 7U; i++) {
            uint8_t code = RAM_PTR8(0x20000356UL)[i];
            if (code == 0) {
                continue;
            }
            RAM8(0x200001A3UL) += 1U;

            if (code < 16U) {
                /* Digital Module */
                uint8_t q_bits = (code >> 2) & 0x03U;
                if (q_bits != 0U) {
                    RAM_PTR8(0x20000364UL)[i] = RAM8(0x2000046AUL);
                    RAM8(0x2000046AUL) = (uint8_t)(RAM8(0x2000046AUL) + (1U << (q_bits - 1U)));
                }
                uint8_t i_bits = code & 0x03U;
                if (i_bits != 0U) {
                    RAM_PTR8(0x2000035DUL)[i] = RAM8(0x20000469UL);
                    RAM8(0x20000469UL) = (uint8_t)(RAM8(0x20000469UL) + (1U << (i_bits - 1U)));
                }
                RAM_PTR8(0x20005F30UL)[i * 2U] = code;
            } else if (code < 0x40U) {
                /* Analog Module (4AI/2AQ/8AI) */
                uint8_t aq_bits = (code >> 2) & 0x03U;
                if (aq_bits != 0U) {
                    RAM_PTR8(0x20000364UL)[i] = RAM8(0x2000046CUL);
                    RAM8(0x2000046CUL) = (uint8_t)(RAM8(0x2000046CUL) + (1U << aq_bits));
                }
                uint8_t ai_bits = code & 0x03U;
                if (ai_bits != 0U) {
                    RAM_PTR8(0x2000035DUL)[i] = RAM8(0x2000046BUL);
                    RAM8(0x2000046BUL) = (uint8_t)(RAM8(0x2000046BUL) +
                                                   ((ai_bits == 3U) ? 16U : (uint8_t)(ai_bits * 4U)));
                }
                if (code < 0x20U) {
                    RAM_PTR8(0x20005F30UL)[i * 2U] = (code & 0xDFU) | 0x80U;
                } else {
                    RAM_PTR8(0x20005F30UL)[i * 2U] = code & 0xDFU;
                }
            } else if (code < 0x60U) {
                /* RTD / Thermocouple Analog Input Module */
                uint8_t ai_bits = code & 0x03U;
                if (ai_bits != 0U) {
                    RAM_PTR8(0x2000035DUL)[i] = RAM8(0x2000046BUL);
                    RAM8(0x2000046BUL) = (uint8_t)(RAM8(0x2000046BUL) +
                                                   ((ai_bits == 3U) ? 16U : (uint8_t)(ai_bits * 4U)));
                }
                RAM_PTR8(0x20005F30UL)[i * 2U] = (code & 0x9FU) | 0x80U;
            } else if (code < 0x80U) {
                /* Intelligent / EM253 Module */
                uint8_t q_bits = (code >> 2) & 0x03U;
                if (q_bits != 0U) {
                    RAM_PTR8(0x20000364UL)[i] = RAM8(0x2000046AUL);
                    RAM8(0x2000046AUL) = (uint8_t)(RAM8(0x2000046AUL) + (1U << (q_bits - 1U)));
                }
                uint8_t i_bits = code & 0x03U;
                if (i_bits != 0U) {
                    RAM_PTR8(0x2000035DUL)[i] = RAM8(0x20000469UL);
                    RAM8(0x20000469UL) = (uint8_t)(RAM8(0x20000469UL) + (1U << (i_bits - 1U)));
                }
                RAM_PTR8(0x20005F30UL)[i * 2U] = (code & 0xBFU) | 0xC0U;
            }

            /* Clamp maximum I/O image limits (16 bytes DI/DO, 64 bytes AI/AQ) */
            if (RAM8(0x20000469UL) > 16U) {
                RAM_PTR8(0x2000035DUL)[i] = 0;
                RAM8(0x20000469UL) = 16U;
            }
            if (RAM8(0x2000046AUL) > 16U) {
                RAM_PTR8(0x20000364UL)[i] = 0;
                RAM8(0x2000046AUL) = 16U;
            }
            if (RAM8(0x2000046BUL) > 64U) {
                RAM_PTR8(0x2000035DUL)[i] = 0;
                RAM8(0x2000046BUL) = 64U;
            }
            if (RAM8(0x2000046CUL) > 64U) {
                RAM_PTR8(0x20000364UL)[i] = 0;
                RAM8(0x2000046CUL) = 64U;
            }
        }
    } else {
        for (i = 0; i < 7U; i++) {
            uint8_t code = RAM_PTR8(0x20000356UL)[i];
            if (code == 0) continue;
            if (code < 16U) {
                RAM_PTR8(0x20005F30UL)[i * 2U] = code;
            } else if (code < 0x40U) {
                RAM_PTR8(0x20005F30UL)[i * 2U] = (code < 0x20U) ? ((code & 0xDFU) | 0x80U)
                                                                 : (code & 0xDFU);
            } else if (code < 0x60U) {
                RAM_PTR8(0x20005F30UL)[i * 2U] = (code & 0x9FU) | 0x80U;
            } else if (code < 0x80U) {
                RAM_PTR8(0x20005F30UL)[i * 2U] = (code & 0xBFU) | 0xC0U;
            }
        }
    }

    RAM8(0x200001C7UL) = 0;
    FUN_08040dd2();

    /* ======================================================================
     * 11. Restore Retentive Data from Flash 0x08069000 (if saved on power-down)
     * ====================================================================== */
    if (FLASH_PTR16(0x08069000UL)[0] == 1U) {
        uint32_t fl_off = 2U;

        /* Restore 32 bytes of M memory (MB0..MB31) */
        for (i = 0; i < 32U; i += 2U) {
            uint16_t hw = FLASH_PTR16(0x08069000UL)[fl_off >> 1];
            RAM_PTR8(0x20005EF8UL)[i]      = (uint8_t)(hw >> 8);
            RAM_PTR8(0x20005EF8UL)[i + 1U] = (uint8_t)hw;
            fl_off += 2U;
        }

        /* Restore 256 Counters (C0..C255 current value & status bits) */
        for (i = 0; i < 256U; i++) {
            RAM_PTR16(0x20006954UL)[i] = FLASH_PTR16(0x08069000UL)[fl_off >> 1];
            fl_off += 2U;
        }
        for (i = 0; i < 256U; i += 2U) {
            uint16_t hw = FLASH_PTR16(0x08069000UL)[fl_off >> 1];
            RAM_PTR8(0x20006B54UL)[i]      = (uint8_t)(hw >> 8);
            RAM_PTR8(0x20006B54UL)[i + 1U] = (uint8_t)hw;
            fl_off += 2U;
        }
        for (i = 0; i < 32U; i += 2U) {
            uint16_t hw = FLASH_PTR16(0x08069000UL)[fl_off >> 1];
            RAM_PTR8(0x20005EC8UL)[i]      = (uint8_t)(hw >> 8);
            RAM_PTR8(0x20005EC8UL)[i + 1U] = (uint8_t)hw;
            fl_off += 2U;
        }

        /* Restore Retentive Timers (T0..T31, T64..T95) */
        for (i = 0; i < 32U; i++) {
            RAM_PTR16(0x20006354UL)[i] = FLASH_PTR16(0x08069000UL)[fl_off >> 1];
            fl_off += 2U;
        }
        for (i = 64U; i < 96U; i++) {
            RAM_PTR16(0x20006354UL)[i] = FLASH_PTR16(0x08069000UL)[fl_off >> 1];
            fl_off += 2U;
        }
        for (i = 0; i < 32U; i += 2U) {
            uint16_t hw = FLASH_PTR16(0x08069000UL)[fl_off >> 1];
            RAM_PTR8(0x20006554UL)[i]      = (uint8_t)(hw >> 8);
            RAM_PTR8(0x20006554UL)[i + 1U] = (uint8_t)hw;
            fl_off += 2U;
        }
        for (i = 64U; i < 96U; i += 2U) {
            uint16_t hw = FLASH_PTR16(0x08069000UL)[fl_off >> 1];
            RAM_PTR8(0x20006554UL)[i]      = (uint8_t)(hw >> 8);
            RAM_PTR8(0x20006554UL)[i + 1U] = (uint8_t)hw;
            fl_off += 2U;
        }
        for (i = 0; i < 4U; i += 2U) {
            uint16_t hw = FLASH_PTR16(0x08069000UL)[fl_off >> 1];
            RAM_PTR8(0x20005ED0UL)[i]      = (uint8_t)(hw >> 8);
            RAM_PTR8(0x20005ED0UL)[i + 1U] = (uint8_t)hw;
            fl_off += 2U;
        }
        for (i = 8U; i < 12U; i += 2U) {
            uint16_t hw = FLASH_PTR16(0x08069000UL)[fl_off >> 1];
            RAM_PTR8(0x20005ED0UL)[i]      = (uint8_t)(hw >> 8);
            RAM_PTR8(0x20005ED0UL)[i + 1U] = (uint8_t)hw;
            fl_off += 2U;
        }

        /* Restore 10240 bytes of Retentive V-Memory (VB0..VB10239) */
        for (i = 0; i < 0x2800U; i += 2U) {
            uint16_t hw = FLASH_PTR16(0x08069000UL)[fl_off >> 1];
            RAM_PTR8(0x2000108CUL)[i]      = (uint8_t)(hw >> 8);
            RAM_PTR8(0x2000108CUL)[i + 1U] = (uint8_t)hw;
            fl_off += 2U;
        }
    }

    /* ======================================================================
     * 12. Index User Program Block (0x0806D800) & Data Block DB1 (0x08073000)
     * ====================================================================== */
    uint16_t prog_words = ((uint16_t)FLASH_PTR8(0x0806D800UL)[0] << 8) |
                          FLASH_PTR8(0x0806D800UL)[1];

    if (prog_words < 0x2000U) {
        uint32_t prog_bytes = (uint32_t)prog_words * 2U;
        uint32_t calc_crc   = FUN_08036ca0(FLASH_PTR8(0x0806D800UL), prog_bytes - 2U);
        uint16_t stored_crc = ((uint16_t)FLASH_PTR8(0x0806D800UL)[prog_bytes - 2U] << 8) |
                              FLASH_PTR8(0x0806D800UL)[prog_bytes - 1U];

        if ((calc_crc & 0xFFFFU) == stored_crc) {
            RAM8(0x200001DAUL)  = 0;
            RAM32(0x200001DCUL) = 0x16UL; /* First instruction word offset in OB1 */
            RAM8(0x200001EAUL)  = 0;
            RAM8(0x200001EBUL)  = 0;

            for (i = 0; i < 68U; i++) {
                RAM_PTR16(0x20000B8EUL)[i] = 0; /* INT_0 .. INT_67 entry offsets */
                RAM_PTR16(0x20000C1CUL)[i] = 0; /* SBR_0 .. SBR_67 entry offsets */
                RAM_PTR8(0x20000D1CUL)[i]  = 0; /* SBR parameter byte counts */
            }

            while (RAM32(0x200001DCUL) < prog_words) {
                uint32_t w_off = RAM32(0x200001DCUL);
                uint32_t b_off = w_off * 2U;
                uint8_t  ins_len = FLASH_PTR8(0x0806D800UL)[b_off];
                uint8_t  ins_opc = FLASH_PTR8(0x0806D800UL)[b_off + 1U];

                if (ins_opc == 0xFCU) {
                    /* MEND (End of OB1 Main Program) */
                    RAM32(0x200001E0UL) = w_off + 1U;
                } else if (ins_opc == 0xE7U) {
                    /* SBR_n Header */
                    uint8_t sbr_idx = FLASH_PTR8(0x0806D800UL)[b_off + 2U];
                    RAM_PTR16(0x20000C1CUL)[sbr_idx] = (uint16_t)(w_off + (ins_len >> 1));
                    RAM8(0x200001EAUL) += 1U;
                    if (ins_len == 4U) {
                        RAM_PTR8(0x20000D1CUL)[sbr_idx] = 0;
                    } else {
                        RAM_PTR8(0x20000D1CUL)[sbr_idx] = FLASH_PTR8(0x0806D800UL)[b_off + 5U];
                    }
                } else if (ins_opc == 0xE8U) {
                    /* INT_n Header */
                    uint8_t int_idx = FLASH_PTR8(0x0806D800UL)[b_off + 2U];
                    RAM_PTR16(0x20000B8EUL)[int_idx] = (uint16_t)(w_off + 2U);
                    RAM8(0x200001EBUL) += 1U;
                }

                if (ins_len == 0) {
                    break;
                }
                RAM32(0x200001DCUL) += (uint32_t)(ins_len >> 1);
            }
        } else {
            RAM8(0x200001DAUL)  = 1U; /* Program CRC error */
            RAM16(0x20005F3EUL) = 0xFFFFU;
        }
    } else {
        /* Empty or unprogrammed User Program block */
        RAM8(0x200001DAUL)  = 0;
        RAM32(0x200001DCUL) = 0;
        RAM8(0x200001EAUL)  = 0;
        RAM8(0x200001EBUL)  = 0;
        RAM32(0x200001E0UL) = 0;
        for (i = 0; i < 68U; i++) {
            RAM_PTR16(0x20000B8EUL)[i] = 0;
            RAM_PTR16(0x20000C1CUL)[i] = 0;
        }
    }

    /* ======================================================================
     * 13. Parse Data Block DB1 (0x08073000) into V-Memory & Apply Retentive Rules
     * ====================================================================== */
    uint16_t db_len = ((uint16_t)FLASH_PTR8(0x08073000UL)[0] << 8) |
                      FLASH_PTR8(0x08073000UL)[1];
    uint8_t  db_obf = FLASH_PTR8(0x08073000UL)[2];

    if (db_len < 0x2800U && db_len > 2U) {
        for (i = 0; i < db_len; i++) {
            uint8_t raw = FLASH_PTR8(0x08073004UL)[i];
            if (db_obf == 1U) {
                raw = (uint8_t)FUN_08014972(raw);
            } else if (db_obf == 2U) {
                raw = (uint8_t)FUN_080149ea(raw);
            } else if (db_obf == 3U) {
                raw = (uint8_t)FUN_08014a5a(raw);
            } else if (db_obf == 4U) {
                raw = (uint8_t)FUN_08014e46(raw);
            }
            RAM_PTR8(0x200038CCUL)[i] = raw;
        }

        uint32_t db_crc = FUN_08036ca0((const uint8_t *)RAM_PTR8(0x200038CCUL),
                                       (uint32_t)db_len - 2U);
        uint16_t stored_db_crc = ((uint16_t)RAM_PTR8(0x200038CCUL)[db_len - 2U] << 8) |
                                 RAM_PTR8(0x200038CCUL)[db_len - 1U];

        if ((db_crc & 0xFFFFU) == stored_db_crc) {
            uint16_t cur_pos = (uint16_t)((((uint16_t)RAM8(0x200038DCUL) << 8) |
                                           RAM8(0x200038DDUL)) + 0x10U);
            while (cur_pos < (uint16_t)(db_len - 6U)) {
                uint16_t next_pos = (uint16_t)(cur_pos + (uint16_t)RAM_PTR8(0x200038CCUL)[cur_pos] + 1U);
                if (FUN_08040ab4(cur_pos, next_pos) != 0) {
                    RAM8(0x200001DBUL) = 1U;
                    break;
                }
                cur_pos = next_pos;
            }
        } else {
            RAM8(0x200001DBUL)  = 1U;
            RAM16(0x20005F40UL) = 0xFFFFU;
        }

        if (RAM8(0x200001DBUL) == 1U) {
            for (i = 0; i < 0x2800U; i++) {
                RAM_PTR8(0x200038CCUL)[i] = 0;
            }
        }
    } else {
        RAM8(0x200001DBUL) = 0;
    }

    /* Apply non-retentive clearing / DB1 reload for ranges not marked retentive */
    uint8_t v_ret_flags[10240];
    for (i = 0; i < 0x2800U; i++) {
        v_ret_flags[i] = 0;
    }

    for (j = 0; j < 6U; j++) {
        uint8_t  area_code = RAM_PTR8(0x200003B8UL)[j];
        uint16_t start_idx = RAM_PTR16(0x200003A0UL)[j];
        uint16_t count     = RAM_PTR16(0x200003ACUL)[j];

        if (area_code == 1U) {
            /* V-memory retentive range */
            for (i = start_idx; i < (uint16_t)(start_idx + count) && i < 0x2800U; i++) {
                v_ret_flags[i] = 1U;
            }
        }
    }

    for (i = 0; i < 0x2800U; i++) {
        if (v_ret_flags[i] == 0U || FLASH_PTR16(0x08069000UL)[0] != 1U) {
            RAM_PTR8(0x2000108CUL)[i] = RAM_PTR8(0x200038CCUL)[i];
        }
    }

    FUN_0803e06c(RAM8(0x20000174UL), RAM8(0x20000175UL));
    FUN_080421b4();
    FUN_0800547c();
}

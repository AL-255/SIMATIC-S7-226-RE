/**
 * @file plc_int_dispatch.c
 * @brief PLC Interrupt Event Queue Dispatcher, INT_n & Nested SBR_n Execution
 *        Wrappers, OB1 Main Program Scan Loop, Operand Fetchers, Comparison
 *        Evaluators, Modbus RTU Slave/Master Handlers, and PWM/PTO/GPIO/Flash
 *        Driver Primitives (0x08013ca8 .. 0x0801744f & 0x08037928 .. 0x0803b100).
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
#define RAM_S8(addr)        (*(volatile int8_t   *)(uintptr_t)(addr))
#define RAM_S16(addr)       (*(volatile int16_t  *)(uintptr_t)(addr))
#define RAM_S32(addr)       (*(volatile int32_t  *)(uintptr_t)(addr))
#define RAM_F32(addr)       (*(volatile float    *)(uintptr_t)(addr))
#define RAM_PTR8(addr)      ((volatile uint8_t   *)(uintptr_t)(addr))
#define RAM_PTR16(addr)     ((volatile uint16_t  *)(uintptr_t)(addr))
#define RAM_PTR32(addr)     ((volatile uint32_t  *)(uintptr_t)(addr))

#define FLASH_PTR8(addr)    ((const uint8_t      *)(uintptr_t)(addr))
#define FLASH_PTR16(addr)   ((const uint16_t     *)(uintptr_t)(addr))

/* Flash Controller Registers (0x40022000) */
#define FLASH_KEYR          REG32(0x40022004UL)
#define FLASH_SR            REG32(0x4002200CUL)
#define FLASH_CR            REG32(0x40022010UL)
#define FLASH_AR            REG32(0x40022014UL)

/* AFIO & GPIO Base Addresses */
#define AFIO_EVCR           REG32(0x40010000UL)
#define AFIO_MAPR           REG32(0x40010004UL)
#define AFIO_EXTICR_BASE    0x40010008UL

#define GPIOA_BASE          ((volatile void *)0x40010800UL)
#define GPIOB_BASE          ((volatile void *)0x40010C00UL)
#define GPIOC_BASE          ((volatile void *)0x40011000UL)
#define GPIOD_BASE          ((volatile void *)0x40011400UL)
#define GPIOE_BASE          ((volatile void *)0x40011800UL)

/* Timers & USARTs */
#define TIM2_BASE           0x40000000UL
#define TIM5_BASE           0x40000C00UL
#define TIM1_BASE           0x40012C00UL
#define TIM8_BASE           0x40013400UL
#define USART2_BASE         0x40004400UL

/* NVIC & SCB Registers */
#define NVIC_ISER_BASE      ((volatile uint32_t *)0xE000E100UL)
#define NVIC_ICER_BASE      ((volatile uint32_t *)0xE000E180UL)
#define NVIC_IPR_BASE       ((volatile uint8_t  *)0xE000E400UL)
#define SCB_AIRCR           REG32(0xE000ED0CUL)

/* External declarations */
extern void     FUN_0801e2b4(void);
extern int32_t  FUN_08021948(void);
extern void     FUN_08040ea0(void);
extern void     FUN_080421b4(void);
extern uint16_t FUN_080366c4(volatile uint8_t *buf, uint16_t len);
extern uint32_t FUN_080002d0(float a, float b);
extern uint32_t FUN_080002e4(float a, float b);
extern uint32_t FUN_080002f8(float a, float b);
extern uint64_t FUN_0800016c(uint32_t num_lo, uint32_t num_hi, uint32_t den_lo, uint32_t den_hi);

/* Forward declarations within this file */
uint32_t FUN_08013d74(uint32_t timeout);
void     FUN_080143e4(volatile uint32_t *gpio_base, const uint8_t *init_struct);
void     FUN_08014526(volatile uint32_t *gpio_base, uint16_t pins);
void     FUN_0801452a(volatile uint32_t *gpio_base, uint16_t pins);
void     FUN_08015402(uint16_t int_routine_id);
void     FUN_08037928(uint16_t sbr_id);
void     FUN_08037f32(uint16_t sbr_id);
void     FUN_080384b2(uint16_t sbr_id);
void     FUN_08038796(uint16_t sbr_id);
void     FUN_08038d08(uint16_t sbr_id);
void     FUN_0803928c(uint16_t sbr_id);
void     FUN_08039826(uint16_t sbr_id);
void     FUN_08039dce(uint16_t sbr_id);
void     FUN_0803a376(uint16_t sbr_id);
void     FUN_0803a91e(uint16_t sbr_id);
void     FUN_0803aee6(uint16_t sbr_id);

/* ==========================================================================
 * Flash Controller Primitives (0x08013ca8 .. 0x08013d95)
 * ========================================================================== */

/* Function: FUN_08013ca8 (FLASH_ErasePage) @ 0x08013ca8 (62 bytes) */
uint32_t FUN_08013ca8(uint32_t page_addr)
{
    uint32_t status = FUN_08013d74(0x00000FFFUL);
    if (status == 4U) {
        FLASH_CR |= 0x00000002UL; /* PER = 1 */
        FLASH_AR  = page_addr;
        FLASH_CR |= 0x00000040UL; /* STRT = 1 */
        status = FUN_08013d74(0x00000FFFUL);
        FLASH_CR &= 0x00001FFDUL; /* Clear PER */
    }
    return status;
}

/* Function: FUN_08013ce6 (FLASH_GetStatus) @ 0x08013ce6 (40 bytes) */
uint32_t FUN_08013ce6(void)
{
    if ((FLASH_SR & 0x01UL) != 0) {
        return 1U; /* FLASH_BUSY */
    }
    if ((FLASH_SR & 0x04UL) != 0) {
        return 2U; /* FLASH_ERROR_PG */
    }
    if ((FLASH_SR & 0x10UL) != 0) {
        return 3U; /* FLASH_ERROR_WRP */
    }
    return 4U; /* FLASH_COMPLETE */
}

/* Function: FUN_08013d0e (FLASH_Lock) @ 0x08013d0e (18 bytes) */
void FUN_08013d0e(void)
{
    FLASH_CR |= 0x00000080UL; /* LOCK = 1 */
}

/* Function: FUN_08013d20 (FLASH_ProgramHalfWord) @ 0x08013d20 (54 bytes) */
uint32_t FUN_08013d20(uint32_t addr, uint16_t data)
{
    uint32_t status = FUN_08013d74(0x0000000FUL);
    if (status == 4U) {
        FLASH_CR |= 0x00000001UL; /* PG = 1 */
        *(volatile uint16_t *)(uintptr_t)addr = data;
        status = FUN_08013d74(0x0000000FUL);
        FLASH_CR &= 0x00001FFEUL; /* Clear PG */
    }
    return status;
}

/* Function: FUN_08013d56 (FLASH_Unlock) @ 0x08013d56 (30 bytes) */
void FUN_08013d56(void)
{
    FLASH_KEYR = 0x45670123UL;
    FLASH_KEYR = 0xCDEF89ABUL;
}

/* Function: FUN_08013d74 (FLASH_WaitForLastOperation) @ 0x08013d74 (34 bytes) */
uint32_t FUN_08013d74(uint32_t timeout)
{
    uint32_t status;
    do {
        status = FUN_08013ce6();
        if (status != 1U) {
            break;
        }
        timeout--;
    } while (timeout != 0);

    if (timeout == 0) {
        status = 5U; /* FLASH_TIMEOUT */
    }
    return status;
}

/* ==========================================================================
 * Function: FUN_08013d96 (PLC_ExecuteOB1MainScan) @ 0x08013d96 (1248 bytes)
 *
 * Executes one complete scan cycle of the user main program block (OB1)
 * stored in Flash at 0x0806D800:
 *   - Copies OB1 Local (L) memory frame (64 bytes) and calls FUN_08021948
 *     to decode and execute each instruction until END / MEND (-4) or STOP (-10).
 *   - Dispatches SBR_n calls (target_id < 0xC9) to FUN_08037928 (nesting level 1)
 *     and asynchronous queued interrupt events (target_id >= 0xC9) to FUN_08015402.
 * ========================================================================== */
void FUN_08013d96(void)
{
    uint8_t saved_l_mem[64];
    uint16_t i;
    int32_t exec_res = 0;

    for (i = 0; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i] = RAM_PTR8(0x2000BB98UL)[i];
    }

    RAM32(0x200001DCUL) = 0;
    RAM8(0x200001D6UL)  = 0;

    do {
        if (RAM32(0x200001DCUL) >= RAM32(0x200001E0UL)) {
            goto ob1_scan_done;
        }

        uint32_t pc_off = RAM32(0x200001DCUL) * 2U;
        for (i = 0; i < 67U && i < RAM_PTR8(0x0806D800UL)[pc_off]; i += 2U) {
            uint16_t word_idx = i >> 1;
            uint32_t byte_pos = pc_off + (uint32_t)word_idx * 2U;
            RAM_PTR8(0x20000496UL)[i]      = FLASH_PTR8(0x0806D800UL)[byte_pos];
            RAM_PTR8(0x20000496UL)[i + 1U] = FLASH_PTR8(0x0806D801UL)[byte_pos];
        }

        RAM32(0x200001DCUL) += (uint32_t)(i >> 1);
        exec_res = FUN_08021948();
        uint16_t target_id = (uint16_t)exec_res;

        if (exec_res >= 0) {
            uint32_t saved_pc = RAM32(0x200001DCUL);
            for (i = 0; i < 64U; i++) {
                saved_l_mem[i] = RAM_PTR8(0x20006110UL)[i];
            }

            uint8_t saved_stack[36];
            for (i = 0; i < 36U; i++) {
                saved_stack[i] = RAM_PTR8(0x200004DCUL)[i];
            }
            uint8_t saved_for_idx = RAM8(0x20000500UL);
            uint8_t saved_rlo     = RAM8(0x200001D6UL);

            if (target_id < 0x00C9U) {
                /* Call Subroutine SBR_n (Nesting Level 1) */
                FUN_08037928(target_id);
                if (RAM8(0x20000501UL) != 0 && RAM8(0x2000051BUL) == 1U) {
                    RAM16(0x2000051CUL) = 0x0100U;
                    FUN_08040ea0();
                    if ((RAM8(0x20005F29UL) & 0x80U) != 0) {
                        uint8_t io_cnt = RAM8(0x200003E9UL);
                        if (io_cnt != 0 && RAM8(0x200003DCUL) == 0 && RAM8(0x200003DFUL) == 0) {
                            if (io_cnt < 17U) {
                                RAM8(0x200003DFUL) = 1U;
                                uint8_t rd_idx = RAM8(0x200003E6UL);
                                RAM8(0x200005F0UL) = RAM_PTR8(0x20000B3EUL)[rd_idx];
                                rd_idx++;
                                RAM8(0x200003E6UL) = (rd_idx > 15U) ? 0U : rd_idx;
                                RAM8(0x200003E9UL) = (uint8_t)(io_cnt - 1U);
                                FUN_08015402(RAM_PTR8(0x20000B58UL)[RAM8(0x200005F0UL)]);
                                RAM8(0x200003DFUL) = 0;
                            } else {
                                RAM8(0x20005F2CUL) |= 0x02U;
                            }
                        }

                        uint8_t comm_cnt = RAM8(0x200003E8UL);
                        if (comm_cnt != 0 && RAM8(0x200003DDUL) == 0 &&
                            RAM8(0x200003E0UL) == 0 && RAM8(0x200003DFUL) == 0) {
                            if (comm_cnt < 5U) {
                                RAM8(0x200003E0UL) = 1U;
                                uint8_t rd_idx = RAM8(0x200003E5UL);
                                RAM8(0x200005F0UL) = RAM_PTR8(0x20000B52UL)[rd_idx];
                                rd_idx++;
                                RAM8(0x200003E5UL) = (rd_idx > 3U) ? 0U : rd_idx;
                                RAM8(0x200003E8UL) = (uint8_t)(comm_cnt - 1U);
                                FUN_08015402(RAM_PTR8(0x20000B58UL)[RAM8(0x200005F0UL)]);
                                RAM8(0x200003E0UL) = 0;
                            } else {
                                RAM8(0x20005F2CUL) |= 0x01U;
                            }
                        }

                        uint8_t tim_cnt = RAM8(0x200003EAUL);
                        if (tim_cnt != 0 && RAM8(0x200003DEUL) == 0 && RAM8(0x200003E1UL) == 0 &&
                            RAM8(0x200003DFUL) == 0 && RAM8(0x200003E0UL) == 0) {
                            if (tim_cnt < 9U) {
                                RAM8(0x200003E1UL) = 1U;
                                uint8_t rd_idx = RAM8(0x200003E7UL);
                                RAM8(0x200005F0UL) = RAM_PTR8(0x20000B4EUL)[rd_idx];
                                rd_idx++;
                                RAM8(0x200003E7UL) = (rd_idx > 3U) ? 0U : rd_idx;
                                RAM8(0x200003EAUL) = (uint8_t)(tim_cnt - 1U);
                                FUN_08015402(RAM_PTR8(0x20000B58UL)[RAM8(0x200005F0UL)]);
                                RAM8(0x200003E1UL) = 0;
                            } else {
                                RAM8(0x20005F2CUL) |= 0x04U;
                            }
                        }
                    }
                    FUN_080421b4();
                }
            } else {
                /* Execute Interrupt Routine INT_(target_id - 0xC9) */
                FUN_08015402((uint16_t)(target_id - 0x00C9U));
            }

            RAM8(0x200001D6UL)  = saved_rlo;
            RAM8(0x20000500UL)  = saved_for_idx;
            for (i = 0; i < 36U; i++) {
                RAM_PTR8(0x200004DCUL)[i] = saved_stack[i];
            }
            for (i = 0; i < 64U; i++) {
                RAM_PTR8(0x20006110UL)[i] = saved_l_mem[i];
            }
            RAM32(0x200001DCUL) = saved_pc;
        } else if (exec_res == -10) {
            /* STOP instruction executed */
            FUN_0801e2b4();
            break;
        } else if (exec_res == -4) {
            /* MEND / END of OB1 reached */
            break;
        }
    } while (exec_res != -4);

ob1_scan_done:
    for (i = 0; i < 64U; i++) {
        RAM_PTR8(0x2000BB98UL)[i] = RAM_PTR8(0x20006110UL)[i];
    }
}

/* ==========================================================================
 * GPIO & AFIO Configuration Primitives (0x08014278 .. 0x0801452d)
 * ========================================================================== */

/* Function: FUN_08014278 (PLC_GPIO_InitStatusLEDsAndSwitch) @ 0x08014278 (326 bytes) */
void FUN_08014278(void)
{
    uint8_t gpio_cfg[4];

    /* PE2 (RUN/STOP switch input), PE4 (Input Pull-Up) */
    gpio_cfg[0] = 0x04U;
    gpio_cfg[1] = 0x00U;
    gpio_cfg[2] = 0x03U; /* 50MHz */
    gpio_cfg[3] = 0x28U; /* Input Pull-Up/Down */
    FUN_080143e4((volatile uint32_t *)GPIOE_BASE, gpio_cfg);
    FUN_0801452a((volatile uint32_t *)GPIOE_BASE, 0x0004U);

    gpio_cfg[0] = 0x10U;
    gpio_cfg[1] = 0x00U;
    gpio_cfg[2] = 0x03U;
    gpio_cfg[3] = 0x28U;
    FUN_080143e4((volatile uint32_t *)GPIOE_BASE, gpio_cfg);

    /* PB7, PB9 (Push-Pull Outputs, initial High) */
    gpio_cfg[0] = 0x80U;
    gpio_cfg[1] = 0x02U;
    gpio_cfg[2] = 0x03U;
    gpio_cfg[3] = 0x10U; /* Output Push-Pull */
    FUN_080143e4((volatile uint32_t *)GPIOB_BASE, gpio_cfg);
    FUN_0801452a((volatile uint32_t *)GPIOB_BASE, 0x0280U);

    /* PB12 (Input Pull-Up/Down) */
    gpio_cfg[0] = 0x00U;
    gpio_cfg[1] = 0x10U;
    gpio_cfg[2] = 0x03U;
    gpio_cfg[3] = 0x28U;
    FUN_080143e4((volatile uint32_t *)GPIOB_BASE, gpio_cfg);

    /* PB15 (Output Push-Pull, initial High) */
    gpio_cfg[0] = 0x00U;
    gpio_cfg[1] = 0x80U;
    gpio_cfg[2] = 0x03U;
    gpio_cfg[3] = 0x10U;
    FUN_080143e4((volatile uint32_t *)GPIOB_BASE, gpio_cfg);
    FUN_0801452a((volatile uint32_t *)GPIOB_BASE, 0x8000U);

    /* PA1, PA6 (Output Push-Pull: PA6 High, PA1 Low) */
    gpio_cfg[0] = 0x42U;
    gpio_cfg[1] = 0x00U;
    gpio_cfg[2] = 0x03U;
    gpio_cfg[3] = 0x10U;
    FUN_080143e4((volatile uint32_t *)GPIOA_BASE, gpio_cfg);
    FUN_0801452a((volatile uint32_t *)GPIOA_BASE, 0x0040U);
    FUN_08014526((volatile uint32_t *)GPIOA_BASE, 0x0002U);

    /* PA0 (Input Pull-Up/Down) */
    gpio_cfg[0] = 0x01U;
    gpio_cfg[1] = 0x00U;
    gpio_cfg[2] = 0x03U;
    gpio_cfg[3] = 0x28U;
    FUN_080143e4((volatile uint32_t *)GPIOA_BASE, gpio_cfg);

    /* PB14 (Output Push-Pull, initial High) */
    gpio_cfg[0] = 0x00U;
    gpio_cfg[1] = 0x40U;
    gpio_cfg[2] = 0x03U;
    gpio_cfg[3] = 0x10U;
    FUN_080143e4((volatile uint32_t *)GPIOB_BASE, gpio_cfg);
    FUN_0801452a((volatile uint32_t *)GPIOB_BASE, 0x4000U);

    /* PB10 (Output Push-Pull, initial High) */
    gpio_cfg[0] = 0x00U;
    gpio_cfg[1] = 0x04U;
    gpio_cfg[2] = 0x03U;
    gpio_cfg[3] = 0x10U;
    FUN_080143e4((volatile uint32_t *)GPIOB_BASE, gpio_cfg);
    FUN_0801452a((volatile uint32_t *)GPIOB_BASE, 0x0400U);

    /* PB11 (Output Push-Pull, initial Low) */
    gpio_cfg[0] = 0x00U;
    gpio_cfg[1] = 0x08U;
    gpio_cfg[2] = 0x03U;
    gpio_cfg[3] = 0x10U;
    FUN_080143e4((volatile uint32_t *)GPIOB_BASE, gpio_cfg);
    FUN_08014526((volatile uint32_t *)GPIOB_BASE, 0x0800U);

    /* PC4 (Input Pull-Up/Down) */
    gpio_cfg[0] = 0x10U;
    gpio_cfg[1] = 0x00U;
    gpio_cfg[2] = 0x03U;
    gpio_cfg[3] = 0x28U;
    FUN_080143e4((volatile uint32_t *)GPIOC_BASE, gpio_cfg);
}

/* Function: FUN_080143be (GPIO_EventOutputCmd) @ 0x080143be (10 bytes) */
void FUN_080143be(uint32_t enable)
{
    REG32(0x4220001CUL) = enable; /* Bit-band AFIO_EVCR.EVOEN */
}

/* Function: FUN_080143c8 (GPIO_EXTILineConfig) @ 0x080143c8 (28 bytes) */
void FUN_080143c8(uint8_t port_source, uint8_t pin_source)
{
    uint32_t shift = (uint32_t)(pin_source & 3U) * 4U;
    volatile uint32_t *exticr = (volatile uint32_t *)(AFIO_EXTICR_BASE + ((uint32_t)(pin_source >> 2) * 4U));
    *exticr = (*exticr & ~(0x0FUL << shift)) | ((uint32_t)port_source << shift);
}

/* Function: FUN_080143e4 (GPIO_Init) @ 0x080143e4 (156 bytes) */
void FUN_080143e4(volatile uint32_t *gpio_base, const uint8_t *init_struct)
{
    uint32_t pins = (uint32_t)init_struct[0] | ((uint32_t)init_struct[1] << 8);
    uint32_t mode = init_struct[3] & 0x0FU;
    if ((init_struct[3] & 0x10U) != 0) {
        mode |= init_struct[2];
    }

    if ((pins & 0x00FFUL) != 0) {
        uint32_t crl = gpio_base[0];
        for (uint32_t pos = 0; pos < 8U; pos++) {
            if (((1UL << pos) & pins & 0x00FFUL) != 0) {
                uint32_t shift = pos << 2;
                crl = (crl & ~(0x0FUL << shift)) | (mode << shift);
                if (init_struct[3] == 0x28U) {
                    gpio_base[5] = 1UL << pos; /* BRR: pull-down */
                } else if (init_struct[3] == 0x48U) {
                    gpio_base[4] = 1UL << pos; /* BSRR: pull-up */
                }
            }
        }
        gpio_base[0] = crl;
    }

    if (pins > 0x00FFUL) {
        uint32_t crh = gpio_base[1];
        for (uint32_t pos = 0; pos < 8U; pos++) {
            uint32_t bit = pos + 8U;
            if (((1UL << bit) & (pins & 0xFF00UL)) != 0) {
                uint32_t shift = pos << 2;
                crh = (crh & ~(0x0FUL << shift)) | (mode << shift);
                if (init_struct[3] == 0x28U) {
                    gpio_base[5] = 1UL << bit;
                } else if (init_struct[3] == 0x48U) {
                    gpio_base[4] = 1UL << bit;
                }
            }
        }
        gpio_base[1] = crh;
    }
}

/* Function: FUN_08014480 (GPIO_EventOutputConfig) @ 0x08014480 (38 bytes) */
void FUN_08014480(uint8_t port_src, uint8_t pin_src)
{
    AFIO_EVCR = (AFIO_EVCR & 0xFF80U) | ((uint16_t)port_src << 4) | pin_src;
}

/* Function: FUN_080144a6 (GPIO_PinLockConfig) @ 0x080144a6 (36 bytes) */
void FUN_080144a6(volatile uint32_t *gpio_base, uint16_t pins)
{
    uint32_t lckr = 0x00010000UL | pins;
    gpio_base[6] = lckr;
    gpio_base[6] = pins;
    gpio_base[6] = lckr;
    (void)gpio_base[6];
    (void)gpio_base[6];
}

/* Function: FUN_080144ca (GPIO_PinRemapConfig) @ 0x080144ca (86 bytes) */
void FUN_080144ca(uint32_t remap, int32_t enable)
{
    uint32_t mapr = AFIO_MAPR;
    uint32_t mask = remap & 0xFFFFUL;
    if ((remap & 0x80000000UL) != 0) {
        mask <<= ((remap >> 16) & 0x0FUL);
    }
    uint32_t cleared = mapr & ~mask;
    if ((remap & 0x00300000UL) != 0x00300000UL) {
        cleared &= 0xF0FFFFFFUL;
        mapr &= ~mask | 0x0F000000UL;
    }
    AFIO_MAPR = cleared | (mapr & 0x0F000000UL);
    if (enable != 0) {
        AFIO_MAPR |= mask;
    }
}

/* Function: FUN_08014520 (GPIO_ReadInputData) @ 0x08014520 (6 bytes) */
uint16_t FUN_08014520(volatile void *gpio_base)
{
    return *(volatile uint16_t *)((uintptr_t)gpio_base + 8U);
}

/* Function: FUN_08014526 (GPIO_ResetBits) @ 0x08014526 (4 bytes) */
void FUN_08014526(volatile uint32_t *gpio_base, uint16_t pins)
{
    gpio_base[5] = pins; /* BRR */
}

/* Function: FUN_0801452a (GPIO_SetBits) @ 0x0801452a (4 bytes) */
void FUN_0801452a(volatile uint32_t *gpio_base, uint16_t pins)
{
    gpio_base[4] = pins; /* BSRR */
}

/* ==========================================================================
 * PLC Operand Address Resolution, Nibble Permutation & RTC Calendar Helpers
 * (0x0801452e .. 0x080153ff)
 * ========================================================================== */

/* Function: FUN_0801452e (PLC_ResolvePointerAddress) @ 0x0801452e (332 bytes) */
uint32_t FUN_0801452e(void)
{
    uint8_t area_type = RAM_PTR8(0x20000496UL)[RAM8(0x200001CCUL) + 3U];
    uint16_t offset = ((uint16_t)RAM_PTR8(0x20000496UL)[RAM8(0x200001CCUL) + 4U] << 8) |
                      RAM_PTR8(0x20000496UL)[RAM8(0x200001CCUL) + 5U];
    uint32_t resolved_ptr = 0;

    switch (area_type) {
    case 1U:  resolved_ptr = offset + 0x08005000UL; break; /* V memory */
    case 2U:  resolved_ptr = offset + 0x08000000UL; break; /* I memory */
    case 3U:  resolved_ptr = offset + 0x08000100UL; break; /* Q memory */
    case 4U:  resolved_ptr = offset + 0x08000200UL; break; /* M memory */
    case 5U:  resolved_ptr = offset + 0x08000300UL; break; /* SM memory */
    case 8U:  resolved_ptr = offset + 0x08000700UL; break; /* T current value */
    case 9U:  resolved_ptr = offset + 0x08000900UL; break; /* C current value */
    case 10U: resolved_ptr = offset + 0x08000500UL; break; /* AI memory */
    case 11U: resolved_ptr = offset + 0x08000600UL; break; /* AQ memory */
    case 15U: resolved_ptr = offset + 0x08000B00UL; break; /* S memory */
    case 16U: resolved_ptr = offset + 0x08000B20UL; break; /* HC memory */
    default:  break;
    }
    RAM8(0x200001CCUL) += 4U;
    return resolved_ptr;
}

/* Function: FUN_0801467a (PLC_FetchByteOperand) @ 0x0801467a (760 bytes) */
uint32_t FUN_0801467a(void)
{
    uint32_t pos = RAM8(0x200001CCUL);
    uint8_t  op_mode = RAM_PTR8(0x20000496UL)[pos + 2U];

    if (op_mode == 1U) {
        /* Immediate 8-bit constant */
        uint8_t val = RAM_PTR8(0x20000496UL)[pos + 3U];
        RAM8(0x200001CCUL) = (uint8_t)(pos + 2U);
        return val;
    }

    if (op_mode != 2U) {
        if (op_mode != 3U) {
            return 0;
        }
        /* Indirect pointer (*VDxx / *LDxx / *AC1..3) */
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
                return RAM_PTR8(0x2000108CUL)[ptr_val - 0x08005000UL];
            }
        } else {
            switch ((ptr_val >> 8) & 0xFFU) {
            case 0U: return RAM_PTR8(0x20005ED8UL)[ptr_val & 0xFFU];
            case 1U: return RAM_PTR8(0x20005EE8UL)[(ptr_val - 0x08000100UL) & 0xFFFFU];
            case 2U: return RAM_PTR8(0x20005EF8UL)[(ptr_val - 0x08000200UL) & 0xFFFFU];
            case 3U:
            case 4U: return RAM_PTR8(0x20005F28UL)[(ptr_val - 0x08000300UL) & 0xFFFFU];
            case 11U:
                if (ptr_val < 0x08000B20UL) {
                    return RAM_PTR8(0x20005EB8UL)[(ptr_val - 0x08000B00UL) & 0xFFFFU];
                }
                break;
            default:
                break;
            }
        }
        RAM8(0x20005F2CUL) |= 0x10U; /* Indirect addressing error */
        return 0;
    }

    /* Direct Memory Area Addressing */
    uint8_t area = RAM_PTR8(0x20000496UL)[pos + 3U];
    if (area == 1U) {
        uint16_t off = ((uint16_t)RAM_PTR8(0x20000496UL)[pos + 4U] << 8) |
                       RAM_PTR8(0x20000496UL)[pos + 5U];
        RAM8(0x200001CCUL) = (uint8_t)(pos + 4U);
        return RAM_PTR8(0x2000108CUL)[off];
    }
    if (area == 5U) {
        uint16_t off = ((uint16_t)RAM_PTR8(0x20000496UL)[pos + 4U] << 8) |
                       RAM_PTR8(0x20000496UL)[pos + 5U];
        RAM8(0x200001CCUL) = (uint8_t)(pos + 4U);
        return RAM_PTR8(0x20005F28UL)[off];
    }

    uint8_t sub_off = RAM_PTR8(0x20000496UL)[pos + 4U];
    uint32_t res = 0;
    switch (area) {
    case 2U:  res = RAM_PTR8(0x20005ED8UL)[sub_off]; break;
    case 3U:  res = RAM_PTR8(0x20005EE8UL)[sub_off]; break;
    case 4U:  res = RAM_PTR8(0x20005EF8UL)[sub_off]; break;
    case 12U: res = RAM_PTR32(0x20005F18UL)[sub_off] & 0xFFU; break;
    case 15U: res = RAM_PTR8(0x20005EB8UL)[sub_off]; break;
    case 0x70U: res = RAM_PTR8(0x20006110UL)[sub_off]; break;
    default:  break;
    }
    RAM8(0x200001CCUL) = (uint8_t)(pos + 3U);
    return res;
}

/* Function: FUN_08014972 (PLC_PermuteByteMode1) @ 0x08014972 (120 bytes) */
uint32_t FUN_08014972(uint32_t val)
{
    uint32_t b3 = ((val & 0xFFU) << 3) & 0x08U;
    uint32_t b7 = ((val & 0xFFU) << 3) & 0x80U;
    uint32_t lo = b3 | ((val & 0xFFU) >> 1);
    uint32_t hi = b7 | ((val & 0xFFU) >> 1);
    return ((lo & 0x0FU) << 4) | ((hi & 0xFFU) >> 4);
}

/* Function: FUN_080149ea (PLC_PermuteByteMode2) @ 0x080149ea (112 bytes) */
uint32_t FUN_080149ea(uint32_t val)
{
    uint32_t b23 = ((val & 0xFFU) << 2) & 0x0CU;
    uint32_t b67 = ((val & 0xFFU) << 2) & 0xC0U;
    uint32_t lo  = b23 | ((val & 0xFFU) >> 2);
    uint32_t hi  = b67 | ((val & 0xFFU) >> 2);
    return ((lo & 0x0FU) << 4) | ((hi & 0xFFU) >> 4);
}

/* Function: FUN_08014a5a (PLC_PermuteByteMode3) @ 0x08014a5a (104 bytes) */
uint32_t FUN_08014a5a(uint32_t val)
{
    uint32_t lo = ((val >> 3) & 1U) | (val * 2U);
    uint32_t hi = (val & 0x80U) ? (((val * 2U) & 0xFFU) | 0x10U) : ((val * 2U) & 0xEFU);
    return ((lo & 0x0FU) << 4) | (hi >> 4);
}

/* Function: FUN_08014ac2 (PLC_FetchDWordOperand) @ 0x08014ac2 (898 bytes) */
uint32_t FUN_08014ac2(void)
{
    uint32_t pos = RAM8(0x200001CCUL);
    uint8_t  op_mode = RAM_PTR8(0x20000496UL)[pos + 2U];

    if (op_mode == 1U) {
        uint32_t val = ((uint32_t)RAM_PTR8(0x20000496UL)[pos + 3U] << 24) |
                       ((uint32_t)RAM_PTR8(0x20000496UL)[pos + 4U] << 16) |
                       ((uint32_t)RAM_PTR8(0x20000496UL)[pos + 5U] << 8)  |
                       RAM_PTR8(0x20000496UL)[pos + 6U];
        RAM8(0x200001CCUL) = (uint8_t)(pos + 5U);
        return val;
    }

    if (op_mode != 2U) {
        if (op_mode != 3U) {
            return 0;
        }
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
                return ((uint32_t)RAM_PTR8(0x2000108CUL)[off] << 24) |
                       ((uint32_t)RAM_PTR8(0x2000108DUL)[off] << 16) |
                       ((uint32_t)RAM_PTR8(0x2000108EUL)[off] << 8)  |
                       RAM_PTR8(0x2000108FUL)[off];
            }
        } else {
            uint32_t sub = ptr_val & 0xFFU;
            switch ((ptr_val >> 8) & 0xFFU) {
            case 0U:
                if (ptr_val < 0x0800000DUL) {
                    return ((uint32_t)RAM_PTR8(0x20005ED8UL)[sub] << 24) |
                           ((uint32_t)RAM_PTR8(0x20005ED9UL)[sub] << 16) |
                           ((uint32_t)RAM_PTR8(0x20005EDAUL)[sub] << 8)  |
                           RAM_PTR8(0x20005EDBUL)[sub];
                }
                break;
            case 1U:
                if (ptr_val < 0x0800010DUL) {
                    uint32_t off = (ptr_val - 0x08000100UL) & 0xFFFFU;
                    return ((uint32_t)RAM_PTR8(0x20005EE8UL)[off] << 24) |
                           ((uint32_t)RAM_PTR8(0x20005EE9UL)[off] << 16) |
                           ((uint32_t)RAM_PTR8(0x20005EEAUL)[off] << 8)  |
                           RAM_PTR8(0x20005EEBUL)[off];
                }
                break;
            case 2U:
                if (ptr_val < 0x0800021DUL) {
                    uint32_t off = (ptr_val - 0x08000200UL) & 0xFFFFU;
                    return ((uint32_t)RAM_PTR8(0x20005EF8UL)[off] << 24) |
                           ((uint32_t)RAM_PTR8(0x20005EF9UL)[off] << 16) |
                           ((uint32_t)RAM_PTR8(0x20005EFAUL)[off] << 8)  |
                           RAM_PTR8(0x20005EFBUL)[off];
                }
                break;
            case 3U:
            case 4U:
                if (ptr_val < 0x08000429UL) {
                    uint32_t off = (ptr_val - 0x08000300UL) & 0xFFFFU;
                    return ((uint32_t)RAM_PTR8(0x20005F28UL)[off] << 24) |
                           ((uint32_t)RAM_PTR8(0x20005F29UL)[off] << 16) |
                           ((uint32_t)RAM_PTR8(0x20005F2AUL)[off] << 8)  |
                           RAM_PTR8(0x20005F2BUL)[off];
                }
                break;
            case 11U:
                if (ptr_val < 0x08000B1DUL) {
                    uint32_t off = (ptr_val - 0x08000B00UL) & 0xFFFFU;
                    return ((uint32_t)RAM_PTR8(0x20005EB8UL)[off] << 24) |
                           ((uint32_t)RAM_PTR8(0x20005EB9UL)[off] << 16) |
                           ((uint32_t)RAM_PTR8(0x20005EBAUL)[off] << 8)  |
                           RAM_PTR8(0x20005EBBUL)[off];
                }
                if (ptr_val < 0x08000B26UL) {
                    return RAM_PTR32(0x20006150UL)[(ptr_val - 0x08000B20UL) & 0xFFFFU];
                }
                break;
            default:
                break;
            }
        }
        RAM8(0x20005F2CUL) |= 0x10U;
        return 0;
    }

    uint8_t area = RAM_PTR8(0x20000496UL)[pos + 3U];
    if (area == 1U) {
        uint32_t off = ((uint32_t)RAM_PTR8(0x20000496UL)[pos + 4U] << 8) |
                       RAM_PTR8(0x20000496UL)[pos + 5U];
        RAM8(0x200001CCUL) = (uint8_t)(pos + 4U);
        return ((uint32_t)RAM_PTR8(0x2000108CUL)[off] << 24) |
               ((uint32_t)RAM_PTR8(0x2000108DUL)[off] << 16) |
               ((uint32_t)RAM_PTR8(0x2000108EUL)[off] << 8)  |
               RAM_PTR8(0x2000108FUL)[off];
    }
    if (area == 5U) {
        uint32_t off = ((uint32_t)RAM_PTR8(0x20000496UL)[pos + 4U] << 8) |
                       RAM_PTR8(0x20000496UL)[pos + 5U];
        RAM8(0x200001CCUL) = (uint8_t)(pos + 4U);
        return ((uint32_t)RAM_PTR8(0x20005F28UL)[off] << 24) |
               ((uint32_t)RAM_PTR8(0x20005F29UL)[off] << 16) |
               ((uint32_t)RAM_PTR8(0x20005F2AUL)[off] << 8)  |
               RAM_PTR8(0x20005F2BUL)[off];
    }

    uint32_t sub = RAM_PTR8(0x20000496UL)[pos + 4U];
    uint32_t res = 0;
    switch (area) {
    case 2U:
        res = ((uint32_t)RAM_PTR8(0x20005ED8UL)[sub] << 24) |
              ((uint32_t)RAM_PTR8(0x20005ED9UL)[sub] << 16) |
              ((uint32_t)RAM_PTR8(0x20005EDAUL)[sub] << 8)  |
              RAM_PTR8(0x20005EDBUL)[sub];
        break;
    case 3U:
        res = ((uint32_t)RAM_PTR8(0x20005EE8UL)[sub] << 24) |
              ((uint32_t)RAM_PTR8(0x20005EE9UL)[sub] << 16) |
              ((uint32_t)RAM_PTR8(0x20005EEAUL)[sub] << 8)  |
              RAM_PTR8(0x20005EEBUL)[sub];
        break;
    case 4U:
        res = ((uint32_t)RAM_PTR8(0x20005EF8UL)[sub] << 24) |
              ((uint32_t)RAM_PTR8(0x20005EF9UL)[sub] << 16) |
              ((uint32_t)RAM_PTR8(0x20005EFAUL)[sub] << 8)  |
              RAM_PTR8(0x20005EFBUL)[sub];
        break;
    case 12U:
        res = RAM_PTR32(0x20005F18UL)[sub];
        break;
    case 15U:
        res = ((uint32_t)RAM_PTR8(0x20005EB8UL)[sub] << 24) |
              ((uint32_t)RAM_PTR8(0x20005EB9UL)[sub] << 16) |
              ((uint32_t)RAM_PTR8(0x20005EBAUL)[sub] << 8)  |
              RAM_PTR8(0x20005EBBUL)[sub];
        break;
    case 16U:
        res = RAM_PTR32(0x20006150UL)[sub];
        break;
    case 0x70U:
        res = ((uint32_t)RAM_PTR8(0x20006110UL)[sub] << 24) |
              ((uint32_t)RAM_PTR8(0x20006111UL)[sub] << 16) |
              ((uint32_t)RAM_PTR8(0x20006112UL)[sub] << 8)  |
              RAM_PTR8(0x20006113UL)[sub];
        break;
    default:
        break;
    }
    RAM8(0x200001CCUL) = (uint8_t)(pos + 3U);
    return res;
}

/* Function: FUN_08014e46 (PLC_SwapNibbles) @ 0x08014e46 (40 bytes) */
uint32_t FUN_08014e46(uint32_t val)
{
    return (((val & 0xFFU) & 0x0FU) << 4) | ((val & 0xFFU) >> 4);
}

/* Function: FUN_08014e6e (PLC_FetchWordOperand) @ 0x08014e6e (928 bytes) */
uint32_t FUN_08014e6e(void)
{
    uint32_t pos = RAM8(0x200001CCUL);
    uint8_t  op_mode = RAM_PTR8(0x20000496UL)[pos + 2U];

    if (op_mode == 1U) {
        uint16_t val = ((uint16_t)RAM_PTR8(0x20000496UL)[pos + 3U] << 8) |
                       RAM_PTR8(0x20000496UL)[pos + 4U];
        RAM8(0x200001CCUL) = (uint8_t)(pos + 3U);
        return val;
    }

    if (op_mode != 2U) {
        if (op_mode != 3U) {
            return 0;
        }
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
                return ((uint16_t)RAM_PTR8(0x2000108CUL)[off] << 8) |
                       RAM_PTR8(0x2000108DUL)[off];
            }
        } else {
            uint32_t sub = ptr_val & 0xFFU;
            switch ((ptr_val >> 8) & 0xFFU) {
            case 0U:
                if (ptr_val < 0x0800000FUL) {
                    return ((uint16_t)RAM_PTR8(0x20005ED8UL)[sub] << 8) |
                           RAM_PTR8(0x20005ED9UL)[sub];
                }
                break;
            case 1U:
                if (ptr_val < 0x0800010FUL) {
                    uint32_t off = (ptr_val - 0x08000100UL) & 0xFFFFU;
                    return ((uint16_t)RAM_PTR8(0x20005EE8UL)[off] << 8) |
                           RAM_PTR8(0x20005EE9UL)[off];
                }
                break;
            case 2U:
                if (ptr_val < 0x0800021FUL) {
                    uint32_t off = (ptr_val - 0x08000200UL) & 0xFFFFU;
                    return ((uint16_t)RAM_PTR8(0x20005EF8UL)[off] << 8) |
                           RAM_PTR8(0x20005EF9UL)[off];
                }
                break;
            case 3U:
            case 4U:
                if (ptr_val < 0x0800042BUL) {
                    uint32_t off = (ptr_val - 0x08000300UL) & 0xFFFFU;
                    return ((uint16_t)RAM_PTR8(0x20005F28UL)[off] << 8) |
                           RAM_PTR8(0x20005F29UL)[off];
                }
                break;
            case 5U:
                if ((ptr_val & 1U) == 0 && ptr_val < 0x0800053FUL) {
                    return RAM_PTR16(0x200007E8UL)[((ptr_val - 0x08000500UL) & 0xFFFFU) >> 1];
                }
                break;
            case 7U:
            case 8U:
                if (ptr_val < 0x08000800UL) {
                    return RAM_PTR16(0x20006354UL)[(ptr_val - 0x08000700UL) & 0xFFFFU];
                }
                break;
            case 9U:
            case 10U:
                if (ptr_val < 0x08000A00UL) {
                    return RAM_PTR16(0x20006954UL)[(ptr_val - 0x08000900UL) & 0xFFFFU];
                }
                break;
            case 11U:
                if (ptr_val < 0x08000B1FUL) {
                    uint32_t off = (ptr_val - 0x08000B00UL) & 0xFFFFU;
                    return ((uint16_t)RAM_PTR8(0x20005EB8UL)[off] << 8) |
                           RAM_PTR8(0x20005EB9UL)[off];
                }
                break;
            default:
                break;
            }
        }
        RAM8(0x20005F2CUL) |= 0x10U;
        return 0;
    }

    uint8_t area = RAM_PTR8(0x20000496UL)[pos + 3U];
    if (area == 1U) {
        uint32_t off = ((uint32_t)RAM_PTR8(0x20000496UL)[pos + 4U] << 8) |
                       RAM_PTR8(0x20000496UL)[pos + 5U];
        RAM8(0x200001CCUL) = (uint8_t)(pos + 4U);
        return ((uint16_t)RAM_PTR8(0x2000108CUL)[off] << 8) |
               RAM_PTR8(0x2000108DUL)[off];
    }
    if (area == 5U) {
        uint32_t off = ((uint32_t)RAM_PTR8(0x20000496UL)[pos + 4U] << 8) |
                       RAM_PTR8(0x20000496UL)[pos + 5U];
        RAM8(0x200001CCUL) = (uint8_t)(pos + 4U);
        return ((uint16_t)RAM_PTR8(0x20005F28UL)[off] << 8) |
               RAM_PTR8(0x20005F29UL)[off];
    }

    uint32_t sub = RAM_PTR8(0x20000496UL)[pos + 4U];
    uint32_t res = 0;
    switch (area) {
    case 2U:  res = ((uint16_t)RAM_PTR8(0x20005ED8UL)[sub] << 8) | RAM_PTR8(0x20005ED9UL)[sub]; break;
    case 3U:  res = ((uint16_t)RAM_PTR8(0x20005EE8UL)[sub] << 8) | RAM_PTR8(0x20005EE9UL)[sub]; break;
    case 4U:  res = ((uint16_t)RAM_PTR8(0x20005EF8UL)[sub] << 8) | RAM_PTR8(0x20005EF9UL)[sub]; break;
    case 8U:  res = RAM_PTR16(0x20006354UL)[sub]; break;
    case 9U:  res = RAM_PTR16(0x20006954UL)[sub]; break;
    case 10U: res = RAM_PTR16(0x200007E8UL)[sub >> 1]; break;
    case 12U: res = RAM_PTR32(0x20005F18UL)[sub] & 0xFFFFU; break;
    case 15U: res = ((uint16_t)RAM_PTR8(0x20005EB8UL)[sub] << 8) | RAM_PTR8(0x20005EB9UL)[sub]; break;
    case 0x70U: res = ((uint16_t)RAM_PTR8(0x20006110UL)[sub] << 8) | RAM_PTR8(0x20006111UL)[sub]; break;
    default:  break;
    }
    RAM8(0x200001CCUL) = (uint8_t)(pos + 3U);
    return res;
}

/* Function: FUN_0801520e (PLC_RTC_UpdateBCDClockBuffer) @ 0x0801520e (496 bytes) */
void FUN_0801520e(void)
{
    uint32_t rem_sec  = RAM32(0x200002B0UL) % 86400UL;
    uint32_t rem_days = RAM32(0x200002B0UL) / 86400UL;
    uint16_t year     = 2000U;
    uint8_t  month    = 1U;

    RAM8(0x20000183UL) = (uint8_t)((rem_days + 6U) % 7U + 1U); /* Day of Week (1=Sun..7=Sat) */

    while (rem_days >= 365U) {
        if ((year & 3U) == 0) {
            if (rem_days < 366U) {
                break;
            }
            rem_days -= 366U;
        } else {
            rem_days -= 365U;
        }
        year++;
    }

    uint16_t day_of_year = (uint16_t)(rem_days + 1U);
    while (day_of_year > 28U) {
        if (month == 2U) {
            if ((year & 3U) == 0) {
                if (day_of_year < 30U) break;
                day_of_year -= 29U;
            } else {
                if (day_of_year < 29U) break;
                day_of_year -= 28U;
            }
        } else if (month == 4U || month == 6U || month == 9U || month == 11U) {
            if (day_of_year < 31U) break;
            day_of_year -= 30U;
        } else {
            if (day_of_year < 32U) break;
            day_of_year -= 31U;
        }
        month++;
    }

    uint8_t yr2  = (uint8_t)(year % 100U);
    uint8_t dom  = (uint8_t)day_of_year;
    uint8_t hour = (uint8_t)(rem_sec / 3600UL);
    uint8_t min  = (uint8_t)((rem_sec % 3600UL) / 60UL);
    uint8_t sec  = (uint8_t)((rem_sec % 3600UL) % 60UL);

    RAM8(0x2000017DUL) = (uint8_t)((yr2  / 10U) * 16U + (yr2  % 10U));
    RAM8(0x2000017EUL) = (uint8_t)((month / 10U) * 16U + (month % 10U));
    RAM8(0x2000017FUL) = (uint8_t)((dom  / 10U) * 16U + (dom  % 10U));
    RAM8(0x20000180UL) = (uint8_t)((hour / 10U) * 16U + (hour % 10U));
    RAM8(0x20000181UL) = (uint8_t)((min  / 10U) * 16U + (min  % 10U));
    RAM8(0x20000182UL) = (uint8_t)((sec  / 10U) * 16U + (sec  % 10U));
}

/* Function: FUN_08015400 (PLC_NoOpStub) @ 0x08015400 (2 bytes) */
void FUN_08015400(void)
{
}

/* ==========================================================================
 * Function: FUN_08015402 (PLC_ExecuteInterruptRoutine_INTn) @ 0x08015402 (2072 bytes)
 *
 * Executes user Interrupt Routine `INT_<int_routine_id>`:
 *   1. Updates HSC status bytes (SMB36, SMB46, SMB56, SMB136, SMB146, SMB156)
 *      according to the active interrupt event ID in `0x200005F0` (events 0x0D..0x22).
 *   2. Updates the Communication, I/O, and Timed Interrupt queue overflow flags
 *      in `SMB4` (`0x20005F2C`) and clears the active event slot in `0x20000B3E`,
 *      `0x20000B4E`, or `0x20000B52`.
 *   3. Saves Accumulators `AC0..AC3` (`0x20005F18..0x20005F24`) to `0x2000BE04..0x2000BE10`
 *      and copies the `INT_n` 64-byte Local (`L`) memory frame.
 *   4. Fetches and executes `INT_n` instructions via `FUN_08021948` until `RETI` (`-0x1B`),
 *      supporting 1 level of subroutine nesting inside an ISR via `FUN_08038796`.
 *   5. Restores `AC0..AC3`, `L` memory, and clears HSC status bits on exit.
 * ========================================================================== */
void FUN_08015402(uint16_t int_routine_id)
{
    uint8_t saved_l_mem[64];
    uint8_t saved_stack[36];
    uint16_t i;

    for (i = 0; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i] =
            RAM_PTR8(0x2000BD98UL)[i + (uint32_t)int_routine_id * 64U];
    }

    uint8_t ev = RAM8(0x200005F0UL);
    switch (ev) {
    case 0x0DU: /* Event 13: HSC0 CV == PV */
        RAM8(0x20005F4CUL) |= 0x40U;
        if (RAM8(0x200003C4UL) == 1U) {
            RAM8(0x20005F4CUL) |= 0x20U;
        } else {
            RAM8(0x20005F4CUL) &= 0xDFU;
        }
        break;

    case 0x0EU: /* Event 14: HSC2 CV == PV */
        RAM8(0x20005F56UL) |= 0x40U;
        if (RAM8(0x200003C5UL) == 1U) {
            RAM8(0x20005F56UL) |= 0x20U;
        } else {
            RAM8(0x20005F56UL) &= 0xDFU;
        }
        break;

    case 0x0FU: /* Event 15: HSC2 Direction Changed */
        RAM8(0x20005F56UL) |= 0x10U;
        if (RAM8(0x200003C5UL) == 1U) {
            RAM8(0x20005F56UL) |= 0x20U;
        } else {
            RAM8(0x20005F56UL) &= 0xDFU;
        }
        break;

    case 0x10U: /* Event 16: HSC2 External Reset */
        RAM8(0x20005F56UL) |= 0x80U;
        break;

    case 0x11U: /* Event 17: HSC4 CV == PV */
        RAM8(0x20005FB0UL) |= 0x40U;
        if (RAM8(0x200003C6UL) == 1U) {
            RAM8(0x20005FB0UL) |= 0x20U;
        } else {
            RAM8(0x20005FB0UL) &= 0xDFU;
        }
        break;

    case 0x12U: /* Event 18: HSC4 Direction Changed */
        RAM8(0x20005FB0UL) |= 0x10U;
        if (RAM8(0x200003C6UL) == 1U) {
            RAM8(0x20005FB0UL) |= 0x20U;
        } else {
            RAM8(0x20005FB0UL) &= 0xDFU;
        }
        break;

    case 0x13U: /* Event 19: HSC4 CV == PV (Timer/SW) */
        RAM8(0x20005FB0UL) |= 0x60U;
        break;

    case 0x1CU: /* Event 28: HSC0 Direction Changed */
        RAM8(0x20005F4CUL) |= 0x10U;
        if (RAM8(0x200003C4UL) == 1U) {
            RAM8(0x20005F4CUL) |= 0x20U;
        } else {
            RAM8(0x20005F4CUL) &= 0xDFU;
        }
        break;

    case 0x1DU: /* Event 29: HSC0 External Reset */
        RAM8(0x20005F4CUL) |= 0x80U;
        break;

    case 0x1EU: /* Event 30: HSC1 CV == PV */
        RAM8(0x20005F60UL) |= 0x40U;
        if (RAM8(0x200003C8UL) == 1U) {
            RAM8(0x20005F60UL) |= 0x20U;
        } else {
            RAM8(0x20005F60UL) &= 0xDFU;
        }
        break;

    case 0x1FUL: /* Event 31: HSC1 Direction Changed */
        RAM8(0x20005F60UL) |= 0x10U;
        if (RAM8(0x200003C8UL) == 1U) {
            RAM8(0x20005F60UL) |= 0x20U;
        } else {
            RAM8(0x20005F60UL) &= 0xDFU;
        }
        break;

    case 0x20U: /* Event 32: HSC1 External Reset */
        RAM8(0x20005F60UL) |= 0x80U;
        break;

    case 0x21U: /* Event 33: HSC5 CV == PV */
        RAM8(0x20005FC4UL) |= 0x40U;
        if (RAM8(0x200003C7UL) == 1U) {
            RAM8(0x20005FC4UL) |= 0x20U;
        } else {
            RAM8(0x20005FC4UL) &= 0xDFU;
        }
        break;

    case 0x22U: /* Event 34: HSC3 CV == PV */
        RAM8(0x20005FBAUL) |= 0x40U;
        if (RAM8(0x200003C9UL) == 1U) {
            RAM8(0x20005FBAUL) |= 0x20U;
        } else {
            RAM8(0x20005FBAUL) &= 0xDFU;
        }
        break;

    default:
        break;
    }

    /* Update SMB4 queue overflow flags and clear dequeued slot */
    if (ev == 8U || ev == 9U || (uint8_t)(ev - 0x17U) < 4U) {
        /* Communication interrupt queue (4 entries @ 0x20000B52) */
        if (RAM8(0x200003E8UL) < 5U) {
            RAM8(0x20005F2CUL) &= 0xFEU;
        } else {
            RAM8(0x20005F2CUL) |= 0x01U;
        }
        uint8_t rd = RAM8(0x200003E5UL);
        if (rd != 0) {
            RAM_PTR8(0x20000B52UL)[rd - 1U] = 0;
        } else {
            RAM8(0x20000B55UL) = 0;
        }
    } else if (ev == 10U || ev == 11U || ev == 0x15U || ev == 0x16U) {
        /* Timed interrupt queue (8 entries @ 0x20000B4E) */
        if (RAM8(0x200003EAUL) < 9U) {
            RAM8(0x20005F2CUL) &= 0xFBU;
        } else {
            RAM8(0x20005F2CUL) |= 0x04U;
        }
        uint8_t rd = RAM8(0x200003E7UL);
        if (rd != 0) {
            RAM_PTR8(0x20000B4EUL)[rd - 1U] = 0;
        } else {
            RAM8(0x20000B51UL) = 0;
        }
    } else {
        /* I/O & HSC interrupt queue (16 entries @ 0x20000B3E) */
        if (RAM8(0x200003E9UL) < 17U) {
            RAM8(0x20005F2CUL) &= 0xFDU;
        } else {
            RAM8(0x20005F2CUL) |= 0x02U;
        }
        uint8_t rd = RAM8(0x200003E6UL);
        if (rd != 0) {
            RAM_PTR8(0x20000B3EUL)[rd - 1U] = 0;
        } else {
            RAM8(0x20000B4DUL) = 0;
        }
    }

    RAM32(0x200001DCUL) = RAM_PTR16(0x20000B8EUL)[int_routine_id];
    RAM8(0x20000501UL)  = 0;

    /* Save Accumulators AC0..AC3 */
    for (i = 0; i < 4U; i++) {
        RAM_PTR32(0x2000BE04UL)[i] = RAM_PTR32(0x20005F18UL)[i];
    }

    do {
        if (RAM32(0x200001DCUL) >= RAM32(0x200001E0UL)) {
            break;
        }

        uint32_t pc_off = RAM32(0x200001DCUL) * 2U;
        for (i = 0; i < 67U && i < RAM_PTR8(0x0806D800UL)[pc_off]; i += 2U) {
            uint16_t word_idx = i >> 1;
            uint32_t byte_pos = pc_off + (uint32_t)word_idx * 2U;
            RAM_PTR8(0x20000496UL)[i]      = FLASH_PTR8(0x0806D800UL)[byte_pos];
            RAM_PTR8(0x20000496UL)[i + 1U] = FLASH_PTR8(0x0806D801UL)[byte_pos];
        }

        RAM32(0x200001DCUL) += (uint32_t)(i >> 1);
        int32_t exec_res = FUN_08021948();
        uint16_t target_id = (uint16_t)exec_res;

        if (exec_res >= 0) {
            uint32_t saved_pc = RAM32(0x200001DCUL);
            for (i = 0; i < 64U; i++) {
                saved_l_mem[i] = RAM_PTR8(0x20006110UL)[i];
            }
            for (i = 0; i < 36U; i++) {
                saved_stack[i] = RAM_PTR8(0x200004DCUL)[i];
            }
            uint8_t saved_for_idx = RAM8(0x20000500UL);
            uint8_t saved_rlo     = RAM8(0x200001D6UL);

            if (target_id < 0x00C9U) {
                /* Call Subroutine SBR_n from within an ISR (ISR Nesting Level 1) */
                FUN_08038796(target_id);
                if (RAM8(0x20000501UL) != 0 && RAM8(0x2000051BUL) == 1U) {
                    RAM16(0x2000051CUL) = 0x0100U;
                    FUN_08040ea0();
                    FUN_080421b4();
                }
            }

            RAM8(0x200001D6UL)  = saved_rlo;
            RAM8(0x20000500UL)  = saved_for_idx;
            for (i = 0; i < 36U; i++) {
                RAM_PTR8(0x200004DCUL)[i] = saved_stack[i];
            }
            for (i = 0; i < 64U; i++) {
                RAM_PTR8(0x20006110UL)[i] = saved_l_mem[i];
            }
            RAM32(0x200001DCUL) = saved_pc;
        } else if (exec_res == -10) {
            FUN_0801e2b4();
            break;
        } else if (exec_res == -0x1B) {
            /* RETI (Return from Interrupt) */
            break;
        }
    } while (true);

    /* Clear HSC status event bits after ISR completion */
    switch (RAM8(0x200005F0UL)) {
    case 0x0DU:
    case 0x1CU:
    case 0x1DU:
        RAM8(0x20005F4CUL) &= 0x0FU;
        break;
    case 0x0EU:
    case 0x0FU:
    case 0x10U:
        RAM8(0x20005F56UL) &= 0x0FU;
        break;
    case 0x11U:
    case 0x12U:
    case 0x13U:
        RAM8(0x20005FB0UL) &= 0x0FU;
        break;
    case 0x1EU:
    case 0x1FUL:
    case 0x20U:
        RAM8(0x20005F60UL) &= 0x0FU;
        break;
    case 0x21U:
        RAM8(0x20005FC4UL) &= 0x0FU;
        break;
    case 0x22U:
        RAM8(0x20005FBAUL) &= 0x0FU;
        break;
    default:
        break;
    }

    RAM8(0x200005F0UL) = 0xFFU;
    RAM8(0x20000501UL) = 1U;

    for (i = 0; i < 64U; i++) {
        RAM_PTR8(0x2000BD98UL)[i + (uint32_t)int_routine_id * 64U] =
            RAM_PTR8(0x20006110UL)[i];
    }
    for (i = 0; i < 4U; i++) {
        RAM_PTR32(0x20005F18UL)[i] = RAM_PTR32(0x2000BE04UL)[i];
    }
}

/* ==========================================================================
 * Function: FUN_08015c1a (PLC_GPIO_InitDigitalInputs) @ 0x08015c1a (140 bytes)
 *
 * Configures the 24 onboard Digital Input pins (I0.0..I2.7) as Input with
 * Pull-Up/Pull-Down (0x28):
 *   - GPIOD: PD8..PD15 (0xFF00)
 *   - GPIOE: PE0, PE1, PE6..PE13, PE15 (0xBFC3)
 *   - GPIOC: PC2, PC3, PC5, PC6, PC7 (0x00EC)
 *   - GPIOB: PB4, PB5, PB6, PB8 (0x0170)
 * ========================================================================== */
void FUN_08015c1a(void)
{
    uint8_t gpio_cfg[4];

    gpio_cfg[0] = 0x00U;
    gpio_cfg[1] = 0xFFU;
    gpio_cfg[2] = 0x03U;
    gpio_cfg[3] = 0x28U;
    FUN_080143e4((volatile uint32_t *)GPIOD_BASE, gpio_cfg);

    gpio_cfg[0] = 0xC3U;
    gpio_cfg[1] = 0xBFU;
    gpio_cfg[2] = 0x03U;
    gpio_cfg[3] = 0x28U;
    FUN_080143e4((volatile uint32_t *)GPIOE_BASE, gpio_cfg);

    gpio_cfg[0] = 0xECU;
    gpio_cfg[1] = 0x00U;
    gpio_cfg[2] = 0x03U;
    gpio_cfg[3] = 0x28U;
    FUN_080143e4((volatile uint32_t *)GPIOC_BASE, gpio_cfg);

    gpio_cfg[0] = 0x70U;
    gpio_cfg[1] = 0x01U;
    gpio_cfg[2] = 0x03U;
    gpio_cfg[3] = 0x28U;
    FUN_080143e4((volatile uint32_t *)GPIOB_BASE, gpio_cfg);
}

/* ==========================================================================
 * PLC Comparison Instruction Evaluators (Byte, DWord/Real, Word)
 * ========================================================================== */

/* Function: FUN_08015ca6 (PLC_EvalCompareByte) @ 0x08015ca6 (434 bytes) */
void FUN_08015ca6(void)
{
    uint8_t op_a = (uint8_t)FUN_0801467a();
    uint8_t op_b = (uint8_t)FUN_0801467a();
    uint8_t sub_op = RAM_PTR8(0x20000496UL)[RAM8(0x200001CCUL) + 1U];
    uint8_t rlo = RAM8(0x200001D6UL);

    switch (sub_op) {
    case 0x10U: /* LDB= */
        rlo = (uint8_t)((rlo << 1) | (op_a == op_b ? 1U : 0U));
        break;
    case 0x11U: /* AB= */
        rlo = (op_a == op_b) ? (rlo | 0x01U) : (rlo & 0xFEU);
        break;
    case 0x12U: /* OB= */
        if (op_a != op_b) rlo &= 0xFEU;
        break;

    case 0x20U: /* LDB>= */
        rlo = (uint8_t)((rlo << 1) | (op_a >= op_b ? 1U : 0U));
        break;
    case 0x21U: /* AB>= */
        rlo = (op_a >= op_b) ? (rlo | 0x01U) : (rlo & 0xFEU);
        break;
    case 0x22U: /* OB>= */
        if (op_a < op_b) rlo &= 0xFEU;
        break;

    case 0x30U: /* LDB<= */
        rlo = (uint8_t)((rlo << 1) | (op_a <= op_b ? 1U : 0U));
        break;
    case 0x31U: /* AB<= */
        rlo = (op_a <= op_b) ? (rlo | 0x01U) : (rlo & 0xFEU);
        break;
    case 0x32U: /* OB<= */
        if (op_a > op_b) rlo &= 0xFEU;
        break;

    case 0x40U: /* LDB> */
        rlo = (uint8_t)((rlo << 1) | (op_a > op_b ? 1U : 0U));
        break;
    case 0x41U: /* AB> */
        rlo = (op_a > op_b) ? (rlo | 0x01U) : (rlo & 0xFEU);
        break;
    case 0x42U: /* OB> */
        if (op_a <= op_b) rlo &= 0xFEU;
        break;

    case 0x50U: /* LDB< */
        rlo = (uint8_t)((rlo << 1) | (op_a < op_b ? 1U : 0U));
        break;
    case 0x51U: /* AB< */
        rlo = (op_a < op_b) ? (rlo | 0x01U) : (rlo & 0xFEU);
        break;
    case 0x52U: /* OB< */
        if (op_a >= op_b) rlo &= 0xFEU;
        break;

    case 0x60U: /* LDB<> */
        rlo = (uint8_t)((rlo << 1) | (op_a != op_b ? 1U : 0U));
        break;
    case 0x61U: /* AB<> */
        rlo = (op_a != op_b) ? (rlo | 0x01U) : (rlo & 0xFEU);
        break;
    case 0x62U: /* OB<> */
        if (op_a == op_b) rlo &= 0xFEU;
        break;

    default:
        break;
    }

    RAM8(0x200001D6UL) = rlo;
    RAM8(0x200001CCUL) += 2U;
}

/* Function: FUN_08015e58 (PLC_EvalCompareDWordOrReal) @ 0x08015e58 (1010 bytes) */
void FUN_08015e58(void)
{
    uint8_t sub_op = RAM_PTR8(0x20000496UL)[RAM8(0x200001CCUL) + 1U];
    RAM32(0x200004F4UL) = FUN_08014ac2();
    RAM32(0x200004F8UL) = FUN_08014ac2();

    int32_t i_a = RAM_S32(0x200004F4UL);
    int32_t i_b = RAM_S32(0x200004F8UL);
    float   f_a = RAM_F32(0x200004F4UL);
    float   f_b = RAM_F32(0x200004F8UL);
    uint8_t rlo = RAM8(0x200001D6UL);

    switch (sub_op) {
    /* Signed 32-bit DInt Comparisons */
    case 0x10U: rlo = (uint8_t)((rlo << 1) | (i_a == i_b ? 1U : 0U)); break;
    case 0x11U: rlo = (i_a == i_b) ? (rlo | 1U) : (rlo & 0xFEU); break;
    case 0x12U: if (i_a != i_b) rlo &= 0xFEU; break;

    case 0x20U: rlo = (uint8_t)((rlo << 1) | (i_a >= i_b ? 1U : 0U)); break;
    case 0x21U: rlo = (i_a >= i_b) ? (rlo | 1U) : (rlo & 0xFEU); break;
    case 0x22U: if (i_a < i_b) rlo &= 0xFEU; break;

    case 0x30U: rlo = (uint8_t)((rlo << 1) | (i_a <= i_b ? 1U : 0U)); break;
    case 0x31U: rlo = (i_a <= i_b) ? (rlo | 1U) : (rlo & 0xFEU); break;
    case 0x32U: if (i_a > i_b) rlo &= 0xFEU; break;

    case 0x40U: rlo = (uint8_t)((rlo << 1) | (i_a > i_b ? 1U : 0U)); break;
    case 0x41U: rlo = (i_a > i_b) ? (rlo | 1U) : (rlo & 0xFEU); break;
    case 0x42U: if (i_a <= i_b) rlo &= 0xFEU; break;

    case 0x50U: rlo = (uint8_t)((rlo << 1) | (i_a < i_b ? 1U : 0U)); break;
    case 0x51U: rlo = (i_a < i_b) ? (rlo | 1U) : (rlo & 0xFEU); break;
    case 0x52U: if (i_a >= i_b) rlo &= 0xFEU; break;

    case 0x60U: rlo = (uint8_t)((rlo << 1) | (i_a != i_b ? 1U : 0U)); break;
    case 0x61U: rlo = (i_a != i_b) ? (rlo | 1U) : (rlo & 0xFEU); break;
    case 0x62U: if (i_a == i_b) rlo &= 0xFEU; break;

    /* IEEE-754 32-bit Real Comparisons */
    case 0x18U: rlo = (uint8_t)((rlo << 1) | (FUN_080002d0(f_a, f_b) != 0 ? 1U : 0U)); break;
    case 0x19U: rlo = (FUN_080002d0(f_a, f_b) != 0) ? (rlo | 1U) : (rlo & 0xFEU); break;
    case 0x1AU: if (FUN_080002d0(f_a, f_b) == 0) rlo &= 0xFEU; break;

    case 0x28U: rlo = (uint8_t)((rlo << 1) | (FUN_080002e4(f_b, f_a) != 0 ? 1U : 0U)); break;
    case 0x29U: rlo = (FUN_080002e4(f_b, f_a) != 0) ? (rlo | 1U) : (rlo & 0xFEU); break;
    case 0x2AU: if (FUN_080002e4(f_b, f_a) == 0) rlo &= 0xFEU; break;

    case 0x38U: rlo = (uint8_t)((rlo << 1) | (FUN_080002e4(f_a, f_b) != 0 ? 1U : 0U)); break;
    case 0x39U: rlo = (FUN_080002e4(f_a, f_b) != 0) ? (rlo | 1U) : (rlo & 0xFEU); break;
    case 0x3AU: if (FUN_080002e4(f_a, f_b) == 0) rlo &= 0xFEU; break;

    case 0x48U: rlo = (uint8_t)((rlo << 1) | (FUN_080002f8(f_b, f_a) != 0 ? 1U : 0U)); break;
    case 0x49U: rlo = (FUN_080002f8(f_b, f_a) != 0) ? (rlo | 1U) : (rlo & 0xFEU); break;
    case 0x4AU: if (FUN_080002f8(f_b, f_a) == 0) rlo &= 0xFEU; break;

    case 0x58U: rlo = (uint8_t)((rlo << 1) | (FUN_080002f8(f_a, f_b) != 0 ? 1U : 0U)); break;
    case 0x59U: rlo = (FUN_080002f8(f_a, f_b) != 0) ? (rlo | 1U) : (rlo & 0xFEU); break;
    case 0x5AU: if (FUN_080002f8(f_a, f_b) == 0) rlo &= 0xFEU; break;

    case 0x68U: rlo = (uint8_t)((rlo << 1) | (FUN_080002d0(f_a, f_b) == 0 ? 1U : 0U)); break;
    case 0x69U: rlo = (FUN_080002d0(f_a, f_b) == 0) ? (rlo | 1U) : (rlo & 0xFEU); break;
    case 0x6AU: if (FUN_080002d0(f_a, f_b) != 0) rlo &= 0xFEU; break;

    default:
        break;
    }

    RAM8(0x200001D6UL) = rlo;
    RAM8(0x200001CCUL) += 2U;
}

/* Function: FUN_08016252 (PLC_EvalCompareWord) @ 0x08016252 (486 bytes) */
void FUN_08016252(void)
{
    uint8_t sub_op = RAM_PTR8(0x20000496UL)[RAM8(0x200001CCUL) + 1U];
    RAM16(0x200004F0UL) = (uint16_t)FUN_08014e6e();
    RAM16(0x200004F2UL) = (uint16_t)FUN_08014e6e();

    int16_t op_a = RAM_S16(0x200004F0UL);
    int16_t op_b = RAM_S16(0x200004F2UL);
    uint8_t rlo  = RAM8(0x200001D6UL);

    switch (sub_op) {
    case 0x10U: rlo = (uint8_t)((rlo << 1) | (op_a == op_b ? 1U : 0U)); break;
    case 0x11U: rlo = (op_a == op_b) ? (rlo | 1U) : (rlo & 0xFEU); break;
    case 0x12U: if (op_a != op_b) rlo &= 0xFEU; break;

    case 0x20U: rlo = (uint8_t)((rlo << 1) | (op_a >= op_b ? 1U : 0U)); break;
    case 0x21U: rlo = (op_a >= op_b) ? (rlo | 1U) : (rlo & 0xFEU); break;
    case 0x22U: if (op_a < op_b) rlo &= 0xFEU; break;

    case 0x30U: rlo = (uint8_t)((rlo << 1) | (op_a <= op_b ? 1U : 0U)); break;
    case 0x31U: rlo = (op_a <= op_b) ? (rlo | 1U) : (rlo & 0xFEU); break;
    case 0x32U: if (op_a > op_b) rlo &= 0xFEU; break;

    case 0x40U: rlo = (uint8_t)((rlo << 1) | (op_a > op_b ? 1U : 0U)); break;
    case 0x41U: rlo = (op_a > op_b) ? (rlo | 1U) : (rlo & 0xFEU); break;
    case 0x42U: if (op_a <= op_b) rlo &= 0xFEU; break;

    case 0x50U: rlo = (uint8_t)((rlo << 1) | (op_a < op_b ? 1U : 0U)); break;
    case 0x51U: rlo = (op_a < op_b) ? (rlo | 1U) : (rlo & 0xFEU); break;
    case 0x52U: if (op_a >= op_b) rlo &= 0xFEU; break;

    case 0x60U: rlo = (uint8_t)((rlo << 1) | (op_a != op_b ? 1U : 0U)); break;
    case 0x61U: rlo = (op_a != op_b) ? (rlo | 1U) : (rlo & 0xFEU); break;
    case 0x62U: if (op_a == op_b) rlo &= 0xFEU; break;

    default:
        break;
    }

    RAM8(0x200001D6UL) = rlo;
    RAM8(0x200001CCUL) += 2U;
}

/* ==========================================================================
 * Function: FUN_0801643a (PLC_ModbusRTUSlave_ProcessRequest) @ 0x0801643a (1398 bytes)
 *
 * Processes an incoming Modbus RTU Slave frame in `0x2000083C` and builds the
 * response frame in `0x20000D5C`:
 *   - FC 01 (Read Coils -> Q memory `0x20005EE8`)
 *   - FC 02 (Read Discrete Inputs -> I memory `0x20005ED8`)
 *   - FC 03 (Read Holding Registers -> V memory `0x2000108C` + `0x20000110` base)
 *   - FC 04 (Read Input Registers -> AI memory `0x200007E8`)
 *   - FC 05 (Write Single Coil -> Q memory `0x20005EE8`)
 *   - FC 06 (Write Single Register -> V memory `0x2000108C`)
 *   - FC 15 (0x0F, Write Multiple Coils -> Q memory `0x20005EE8`)
 *   - FC 16 (0x10, Write Multiple Registers -> V memory `0x2000108C`)
 * ========================================================================== */
void FUN_0801643a(void)
{
    uint16_t start_addr = ((uint16_t)RAM8(0x2000083EUL) << 8) | RAM8(0x2000083FUL);
    uint16_t quantity   = ((uint16_t)RAM8(0x20000840UL) << 8) | RAM8(0x20000841UL);

    RAM8(0x20000D5CUL) = RAM8(0x2000083CUL); /* Slave Address */
    RAM8(0x20000D5DUL) = RAM8(0x2000083DUL); /* Function Code */
    RAM8(0x20000117UL) = 0;

    switch (RAM8(0x2000083DUL)) {
    case 1U: /* FC 01: Read Coils (Q0.0 .. Q15.7) */
        if (quantity != 0 && (uint32_t)start_addr + (uint32_t)quantity < 0x81UL) {
            if (RAM8(0x20000178UL) != 8U) goto mb_exc_03;
            uint8_t byte_cnt = ((quantity & 7U) == 0) ? (uint8_t)(quantity >> 3)
                                                      : (uint8_t)((quantity >> 3) + 1U);
            RAM8(0x20000D5EUL) = byte_cnt;
            for (uint16_t b = 0; b < byte_cnt; b++) {
                RAM_PTR8(0x20000D5FUL)[b] = 0;
            }
            for (uint16_t bit = 0; bit < quantity; bit++) {
                uint16_t cur = (uint16_t)(start_addr + bit);
                if (((RAM_PTR8(0x20005EE8UL)[cur >> 3] >> (cur & 7U)) & 1U) != 0) {
                    RAM_PTR8(0x20000D5FUL)[bit >> 3] |= (uint8_t)(1U << (bit & 7U));
                }
            }
            uint16_t crc = FUN_080366c4(RAM_PTR8(0x20000D5CUL), (uint16_t)(byte_cnt + 3U));
            RAM_PTR8(0x20000D5FUL)[byte_cnt]      = (uint8_t)crc;
            RAM_PTR8(0x20000D60UL)[byte_cnt]      = (uint8_t)(crc >> 8);
            RAM8(0x20000112UL) = (uint8_t)(byte_cnt + 5U);
        } else {
            RAM8(0x20000D5DUL) = 0x81U;
            goto mb_exc_02;
        }
        break;

    case 2U: /* FC 02: Read Discrete Inputs (I0.0 .. I15.7) */
        if (quantity != 0 && (uint32_t)start_addr + (uint32_t)quantity < 0x81UL) {
            if (RAM8(0x20000178UL) != 8U) goto mb_exc_03;
            uint8_t byte_cnt = ((quantity & 7U) == 0) ? (uint8_t)(quantity >> 3)
                                                      : (uint8_t)((quantity >> 3) + 1U);
            RAM8(0x20000D5EUL) = byte_cnt;
            for (uint16_t b = 0; b < byte_cnt; b++) {
                RAM_PTR8(0x20000D5FUL)[b] = 0;
            }
            for (uint16_t bit = 0; bit < quantity; bit++) {
                uint16_t cur = (uint16_t)(start_addr + bit);
                if (((RAM_PTR8(0x20005ED8UL)[cur >> 3] >> (cur & 7U)) & 1U) != 0) {
                    RAM_PTR8(0x20000D5FUL)[bit >> 3] |= (uint8_t)(1U << (bit & 7U));
                }
            }
            uint16_t crc = FUN_080366c4(RAM_PTR8(0x20000D5CUL), (uint16_t)(byte_cnt + 3U));
            RAM_PTR8(0x20000D5FUL)[byte_cnt]      = (uint8_t)crc;
            RAM_PTR8(0x20000D60UL)[byte_cnt]      = (uint8_t)(crc >> 8);
            RAM8(0x20000112UL) = (uint8_t)(byte_cnt + 5U);
        } else {
            RAM8(0x20000D5DUL) = 0x82U;
            goto mb_exc_02;
        }
        break;

    case 3U: /* FC 03: Read Holding Registers (V Memory) */
        if ((uint32_t)(quantity - 1U) < 120U &&
            ((uint32_t)RAM16(0x20000110UL) + ((uint32_t)start_addr + (uint32_t)quantity) * 2U) <=
                (uint32_t)RAM16(0x20000114UL) + 0x2800UL) {
            if (RAM8(0x20000178UL) != 8U) goto mb_exc_03;
            RAM8(0x20000D5EUL) = (uint8_t)(quantity * 2U);
            for (uint16_t r = 0; r < quantity; r++) {
                uint32_t v_off = (uint32_t)RAM16(0x20000110UL) + (uint32_t)(start_addr + r) * 2U;
                RAM_PTR8(0x20000D5FUL)[r * 2U]      = RAM_PTR8(0x2000108CUL)[v_off];
                RAM_PTR8(0x20000D60UL)[r * 2U]      = RAM_PTR8(0x2000108DUL)[v_off];
            }
            uint16_t byte_cnt = (uint16_t)(quantity * 2U);
            uint16_t crc = FUN_080366c4(RAM_PTR8(0x20000D5CUL), (uint16_t)(byte_cnt + 3U));
            RAM_PTR8(0x20000D5FUL)[byte_cnt] = (uint8_t)crc;
            RAM_PTR8(0x20000D60UL)[byte_cnt] = (uint8_t)(crc >> 8);
            RAM8(0x20000112UL) = (uint8_t)(byte_cnt + 5U);
        } else {
            RAM8(0x20000D5DUL) = 0x83U;
            goto mb_exc_02;
        }
        break;

    case 4U: /* FC 04: Read Input Registers (AI Memory) */
        if (quantity != 0 && (uint32_t)start_addr + (uint32_t)quantity < 0x21UL) {
            if (RAM8(0x20000178UL) != 8U) goto mb_exc_03;
            RAM8(0x20000D5EUL) = (uint8_t)(quantity * 2U);
            for (uint16_t r = 0; r < quantity; r++) {
                uint16_t ai_val = RAM_PTR16(0x200007E8UL)[start_addr + r];
                RAM_PTR8(0x20000D5FUL)[r * 2U] = (uint8_t)(ai_val >> 8);
                RAM_PTR8(0x20000D60UL)[r * 2U] = (uint8_t)ai_val;
            }
            uint16_t byte_cnt = (uint16_t)(quantity * 2U);
            uint16_t crc = FUN_080366c4(RAM_PTR8(0x20000D5CUL), (uint16_t)(byte_cnt + 3U));
            RAM_PTR8(0x20000D5FUL)[byte_cnt] = (uint8_t)crc;
            RAM_PTR8(0x20000D60UL)[byte_cnt] = (uint8_t)(crc >> 8);
            RAM8(0x20000112UL) = (uint8_t)(byte_cnt + 5U);
        } else {
            RAM8(0x20000D5DUL) = 0x84U;
            goto mb_exc_02;
        }
        break;

    case 5U: /* FC 05: Write Single Coil */
        if (start_addr < 0x80U) {
            if ((quantity != 0xFF00U && quantity != 0x0000U) || RAM8(0x20000178UL) != 8U) {
                goto mb_exc_03;
            }
            uint8_t mask = (uint8_t)(1U << (start_addr & 7U));
            if (quantity == 0xFF00U) {
                RAM_PTR8(0x20005EE8UL)[start_addr >> 3] |= mask;
            } else {
                RAM_PTR8(0x20005EE8UL)[start_addr >> 3] &= (uint8_t)~mask;
            }
            for (uint16_t i = 0; i < 8U; i++) {
                RAM_PTR8(0x20000D5CUL)[i] = RAM_PTR8(0x2000083CUL)[i];
            }
            RAM8(0x20000112UL) = 8U;
        } else {
            RAM8(0x20000D5DUL) = 0x85U;
            goto mb_exc_02;
        }
        break;

    case 6U: /* FC 06: Write Single Holding Register */
        if (((uint32_t)RAM16(0x20000110UL) + (uint32_t)start_addr * 2U) <=
            (uint32_t)RAM16(0x20000114UL) + 0x27FEUL) {
            if (RAM8(0x20000178UL) != 8U) goto mb_exc_03;
            uint32_t v_off = (uint32_t)RAM16(0x20000110UL) + (uint32_t)start_addr * 2U;
            RAM_PTR8(0x2000108CUL)[v_off] = (uint8_t)(quantity >> 8);
            RAM_PTR8(0x2000108DUL)[v_off] = (uint8_t)quantity;
            for (uint16_t i = 0; i < 8U; i++) {
                RAM_PTR8(0x20000D5CUL)[i] = RAM_PTR8(0x2000083CUL)[i];
            }
            RAM8(0x20000112UL) = 8U;
        } else {
            RAM8(0x20000D5DUL) = 0x86U;
            goto mb_exc_02;
        }
        break;

    case 15U: /* FC 15 (0x0F): Write Multiple Coils */
        if (quantity != 0 && (uint32_t)start_addr + (uint32_t)quantity < 0x81UL) {
            uint8_t exp_bytes = ((quantity & 7U) == 0) ? (uint8_t)(quantity >> 3)
                                                       : (uint8_t)((quantity >> 3) + 1U);
            if (RAM8(0x20000842UL) != exp_bytes || (uint32_t)RAM8(0x20000178UL) != (uint32_t)exp_bytes + 9U) {
                goto mb_exc_03;
            }
            for (uint16_t bit = 0; bit < quantity; bit++) {
                uint16_t dst = (uint16_t)(start_addr + bit);
                uint8_t  msk = (uint8_t)(1U << (dst & 7U));
                if (((RAM_PTR8(0x20000843UL)[bit >> 3] >> (bit & 7U)) & 1U) != 0) {
                    RAM_PTR8(0x20005EE8UL)[dst >> 3] |= msk;
                } else {
                    RAM_PTR8(0x20005EE8UL)[dst >> 3] &= (uint8_t)~msk;
                }
            }
            goto mb_echo_6_plus_crc;
        } else {
            RAM8(0x20000D5DUL) = 0x8FU;
            goto mb_exc_02;
        }

    case 16U: /* FC 16 (0x10): Write Multiple Holding Registers */
        if ((uint32_t)(quantity - 1U) < 120U &&
            ((uint32_t)RAM16(0x20000110UL) + ((uint32_t)start_addr + (uint32_t)quantity) * 2U) <=
                (uint32_t)RAM16(0x20000114UL) + 0x2800UL) {
            if ((uint32_t)RAM8(0x20000842UL) == (uint32_t)quantity * 2U &&
                (uint32_t)RAM8(0x20000178UL) == (uint32_t)RAM8(0x20000842UL) + 9U) {
                for (uint16_t r = 0; r < quantity; r++) {
                    uint32_t v_off = (uint32_t)RAM16(0x20000110UL) + (uint32_t)(start_addr + r) * 2U;
                    RAM_PTR8(0x2000108CUL)[v_off] = RAM_PTR8(0x20000843UL)[r * 2U];
                    RAM_PTR8(0x2000108DUL)[v_off] = RAM_PTR8(0x20000844UL)[r * 2U];
                }
            mb_echo_6_plus_crc:
                for (uint16_t i = 0; i < 6U; i++) {
                    RAM_PTR8(0x20000D5CUL)[i] = RAM_PTR8(0x2000083CUL)[i];
                }
                uint16_t crc = FUN_080366c4(RAM_PTR8(0x20000D5CUL), 6U);
                RAM8(0x20000D62UL) = (uint8_t)crc;
                RAM8(0x20000D63UL) = (uint8_t)(crc >> 8);
                RAM8(0x20000112UL) = 8U;
            } else {
            mb_exc_03:
                RAM8(0x20000D5DUL) = RAM8(0x2000083DUL) | 0x80U;
                RAM8(0x20000D5EUL) = 3U; /* Illegal Data Value */
                RAM8(0x20000117UL) = 5U;
                goto mb_append_exc_crc;
            }
        } else {
            RAM8(0x20000D5DUL) = 0x90U;
        mb_exc_02:
            RAM8(0x20000D5EUL) = 2U; /* Illegal Data Address */
            RAM8(0x20000117UL) = 6U;
            goto mb_append_exc_crc;
        }
        break;

    default: /* Unsupported Function Code */
        RAM8(0x20000D5DUL) = RAM8(0x2000083DUL) | 0x80U;
        RAM8(0x20000D5EUL) = 1U; /* Illegal Function */
        RAM8(0x20000117UL) = 4U;
    mb_append_exc_crc:
        {
            uint16_t crc = FUN_080366c4(RAM_PTR8(0x20000D5CUL), 3U);
            RAM8(0x20000D5FUL) = (uint8_t)crc;
            RAM8(0x20000D60UL) = (uint8_t)(crc >> 8);
            RAM8(0x20000112UL) = 5U;
        }
        break;
    }
}

/* ==========================================================================
 * Function: FUN_080169b0 (PLC_ModbusMaster_SetupTransaction) @ 0x080169b0 (262 bytes)
 * ========================================================================== */
void FUN_080169b0(void)
{
    uint16_t crc = FUN_080366c4(RAM_PTR8(0x20000E5CUL), RAM8(0x20000118UL));
    uint8_t  len = RAM8(0x20000118UL);
    RAM_PTR8(0x20000E5CUL)[len]      = (uint8_t)crc;
    RAM_PTR8(0x20000E5DUL)[len]      = (uint8_t)(crc >> 8);
    RAM8(0x20000118UL) = (uint8_t)(len + 2U);

    switch (RAM8(0x2000011BUL)) {
    case 1U:
    case 2U:
        if ((RAM16(0x20000120UL) & 7U) == 0) {
            RAM8(0x20000119UL) = (uint8_t)((RAM16(0x20000120UL) >> 3) + 5U);
        } else {
            RAM8(0x20000119UL) = (uint8_t)((RAM16(0x20000120UL) >> 3) + 6U);
        }
        break;
    case 3U:
    case 4U:
        RAM8(0x20000119UL) = (uint8_t)((RAM16(0x20000120UL) & 0x7FU) * 2U + 5U);
        break;
    case 5U:
    case 6U:
    case 15U:
    case 16U:
        RAM8(0x20000119UL) = 8U;
        break;
    default:
        break;
    }

    RAM16(0x2000011CUL) = 0;
    RAM16(0x20000176UL) = 0;
    REG16(USART2_BASE + 0x00UL) &= 0xFFDFU; /* Clear RXNEIE/RXNE */
    REG16(USART2_BASE + 0x0CUL) &= 0xFFFBU; /* Disable RE */
    FUN_0801452a((volatile uint32_t *)GPIOA_BASE, 0x0002U); /* RS485 DE = 1 (TX) */
    RAM8(0x2000012AUL) = 1U;
}

/* ==========================================================================
 * NVIC, PWM0/1, PTO0/1, Digital Output GPIO & Status Trace Helpers
 * (0x08016ab8 .. 0x0801744f)
 * ========================================================================== */

/* Function: FUN_08016ab8 (NVIC_Init) @ 0x08016ab8 (100 bytes) */
void FUN_08016ab8(const uint8_t *nvic_init)
{
    uint8_t irq_ch = nvic_init[0];
    if (nvic_init[3] != 0) {
        uint32_t prigroup = (~SCB_AIRCR & 0x0700UL) >> 8;
        uint32_t pre_bits = 4U - prigroup;
        uint32_t sub_bits = prigroup + 4U;
        uint8_t  prio     = (uint8_t)((nvic_init[1] << sub_bits) |
                                      (nvic_init[2] & (0x0FU >> pre_bits)) << 4);
        NVIC_IPR_BASE[irq_ch] = prio;
        NVIC_ISER_BASE[irq_ch >> 5] = 1UL << (irq_ch & 0x1FU);
    } else {
        NVIC_ICER_BASE[irq_ch >> 5] = 1UL << (irq_ch & 0x1FU);
    }
}

/* Function: FUN_08016b1c (NVIC_PriorityGroupConfig) @ 0x08016b1c (20 bytes) */
void FUN_08016b1c(uint32_t prio_group)
{
    SCB_AIRCR = 0x05FA0000UL | prio_group;
}

/* Function: FUN_08016b30 (PLC_NVIC_EnableTIM1_UP_IRQ) @ 0x08016b30 (50 bytes) */
void FUN_08016b30(void)
{
    uint8_t cfg[4] = { 25U, 0U, 2U, 1U };
    FUN_08016ab8(cfg);
}

/* Function: FUN_08016b62 (PLC_NVIC_EnableTIM2_IRQ) @ 0x08016b62 (52 bytes) */
void FUN_08016b62(void)
{
    uint8_t cfg[4] = { 28U, 1U, 1U, 1U };
    FUN_08016ab8(cfg);
}

/* Function: FUN_08016b96 (PLC_NVIC_EnableTIM5_IRQ) @ 0x08016b96 (52 bytes) */
void FUN_08016b96(void)
{
    uint8_t cfg[4] = { 50U, 1U, 2U, 1U };
    FUN_08016ab8(cfg);
}

/* Function: FUN_08016bca (PLC_NVIC_EnableTIM6_IRQ) @ 0x08016bca (52 bytes) */
void FUN_08016bca(void)
{
    uint8_t cfg[4] = { 54U, 2U, 1U, 1U };
    FUN_08016ab8(cfg);
}

/* Function: FUN_08016bfe (PLC_NVIC_EnableTIM7_IRQ) @ 0x08016bfe (52 bytes) */
void FUN_08016bfe(void)
{
    uint8_t cfg[4] = { 55U, 2U, 1U, 1U };
    FUN_08016ab8(cfg);
}

/* Function: FUN_08016c32 (PLC_NVIC_EnableTIM6_7_IRQ) @ 0x08016c32 (58 bytes) */
void FUN_08016c32(void)
{
    uint8_t cfg[4] = { 54U, 2U, 1U, 1U };
    FUN_08016ab8(cfg);
    cfg[0] = 55U;
    cfg[1] = 2U;
    cfg[2] = 2U;
    cfg[3] = 1U;
    FUN_08016ab8(cfg);
}

/* Function: FUN_08016c6c (PLC_NVIC_EnableTIM8_UP_IRQ) @ 0x08016c6c (40 bytes) */
void FUN_08016c6c(void)
{
    uint8_t cfg[4] = { 44U, 0U, 3U, 1U };
    FUN_08016ab8(cfg);
}

/* Function: FUN_08016c94 (PLC_PWM0_UpdateCycleAndPulseWidth) @ 0x08016c94 (250 bytes) */
void FUN_08016c94(uint8_t timebase_ms, uint16_t cycle_time, uint16_t pulse_width)
{
    uint32_t arr_val = 2U;
    uint32_t ccr_val = 0U;

    if (timebase_ms == 0) {
        /* Microsecond timebase */
        if (cycle_time == 1U) {
            arr_val = 48U;
            ccr_val = (pulse_width == 0) ? 0U : 24U;
        } else {
            uint64_t scaled = (uint64_t)cycle_time * 1500ULL;
            if ((uint32_t)(scaled >> 16) < 0x10000UL) {
                arr_val = ((uint32_t)cycle_time * 3U) >> 1;
                ccr_val = ((uint32_t)pulse_width * 3U) >> 1;
            }
        }
    } else {
        /* Millisecond timebase */
        arr_val = (uint32_t)cycle_time * 2U;
        ccr_val = (uint32_t)pulse_width * 2U;
        if (arr_val > 0xFFFFUL) {
            arr_val = (uint32_t)cycle_time + ((uint32_t)cycle_time >> 1);
            ccr_val = (uint32_t)pulse_width + ((uint32_t)pulse_width >> 1);
        }
    }

    REG16(TIM1_BASE + 0x2CUL) = (uint16_t)(arr_val - 1U); /* TIM1_ARR */
    if (pulse_width == 0) {
        REG16(TIM1_BASE + 0x34UL) = 0; /* TIM1_CCR1 */
    } else if (pulse_width < cycle_time) {
        REG16(TIM1_BASE + 0x34UL) = (uint16_t)(arr_val - ccr_val);
    } else {
        REG16(TIM1_BASE + 0x34UL) = 0xFFFFU;
    }
}

/* Function: FUN_08016d8e (PLC_PWM1_UpdateCycleAndPulseWidth) @ 0x08016d8e (254 bytes) */
void FUN_08016d8e(uint8_t timebase_ms, uint16_t cycle_time, uint16_t pulse_width)
{
    uint32_t arr_val = 2U;
    uint32_t ccr_val = 0U;

    if (timebase_ms == 0) {
        if (cycle_time == 1U) {
            arr_val = 48U;
            ccr_val = (pulse_width == 0) ? 0U : 24U;
        } else {
            uint64_t scaled = (uint64_t)cycle_time * 1500ULL;
            if ((uint32_t)(scaled >> 16) < 0x10000UL) {
                arr_val = ((uint32_t)cycle_time * 3U) >> 1;
                ccr_val = ((uint32_t)pulse_width * 3U) >> 1;
            }
        }
    } else {
        arr_val = (uint32_t)cycle_time * 2U;
        ccr_val = (uint32_t)pulse_width * 2U;
        if (arr_val > 0xFFFFUL) {
            arr_val = (uint32_t)cycle_time + ((uint32_t)cycle_time >> 1);
            ccr_val = (uint32_t)pulse_width + ((uint32_t)pulse_width >> 1);
        }
    }

    REG16(TIM8_BASE + 0x2CUL) = (uint16_t)(arr_val - 1U); /* TIM8_ARR */
    if (pulse_width == 0) {
        REG16(TIM8_BASE + 0x34UL) = 0; /* TIM8_CCR1 */
    } else if (pulse_width < cycle_time) {
        REG16(TIM8_BASE + 0x34UL) = (uint16_t)(arr_val - ccr_val);
    } else {
        REG16(TIM8_BASE + 0x34UL) = 0xFFFFU;
    }
}

/* Function: FUN_08016e8c (PLC_PTO0_ConfigTimers_TIM1_TIM2) @ 0x08016e8c (450 bytes) */
void FUN_08016e8c(uint32_t pulse_cnt, uint32_t start_freq_mHz, uint32_t end_freq_mHz,
                  uint32_t ramp_steps, uint32_t is_table_mode)
{
    uint16_t psc = 1U;
    uint32_t min_freq = (start_freq_mHz <= end_freq_mHz) ? start_freq_mHz : end_freq_mHz;

    if (min_freq < 146500UL) {
        psc = 9600U;
    } else if (min_freq < 1465000UL) {
        psc = 960U;
    } else if (min_freq < 14650000UL) {
        psc = 96U;
    } else if (min_freq < 73250000UL) {
        psc = 16U;
    } else {
        psc = 2U;
    }

    uint32_t timer_clk_mHz = (96000000UL / psc) * 100U;
    uint32_t arr_start = timer_clk_mHz / start_freq_mHz;
    uint32_t arr_end   = timer_clk_mHz / end_freq_mHz;

    REG16(TIM1_BASE + 0x28UL) = psc - 1U;                  /* TIM1_PSC */
    REG16(TIM1_BASE + 0x2CUL) = (uint16_t)(arr_start - 1U); /* TIM1_ARR */
    REG16(TIM1_BASE + 0x34UL) = (uint16_t)(arr_start >> 1); /* TIM1_CCR1 (50% duty) */
    REG16(TIM1_BASE + 0x14UL) |= 0x0001U;                  /* TIM1_EGR.UG = 1 */

    if (pulse_cnt < 0x10000UL) {
        REG16(TIM2_BASE + 0x2CUL) = (uint16_t)(pulse_cnt - 1U);
    } else {
        REG16(TIM2_BASE + 0x2CUL) = 19999U;
    }

    if (arr_start == arr_end) {
        RAM32(0x200002B8UL) = 0;
        RAM8(0x20000193UL)  = 0;
        RAM8(0x2000018AUL)  = 1U;
        return;
    }

    int32_t diff = (int32_t)(arr_end - arr_start);
    if (diff < 0) diff = -diff;
    uint32_t steps = ramp_steps;
    if (is_table_mode != 1U) {
        steps--;
    }

    uint64_t scaled_diff = (uint64_t)(uint32_t)diff * 10000ULL;
    uint64_t step_inc = FUN_0800016c((uint32_t)scaled_diff, (uint32_t)(scaled_diff >> 32),
                                     steps, 0);
    RAM32(0x200002D8UL) = (uint32_t)step_inc;
    RAM32(0x20000090UL) = (uint32_t)step_inc;
    if ((uint32_t)step_inc < 10000UL) {
        RAM32(0x200002B8UL) = 1UL;
        RAM8(0x2000018AUL)  = (uint8_t)(10000UL / (uint32_t)step_inc);
    } else {
        RAM32(0x200002B8UL) = (uint32_t)step_inc / 10000UL;
        RAM8(0x2000018AUL)  = 1U;
    }
    RAM32(0x200002B4UL) = arr_start - 1U;
    RAM8(0x20000193UL)  = (arr_start >= arr_end) ? 2U : 1U;
}

/* Function: FUN_0801704e (PLC_PTO1_ConfigTimers_TIM8_TIM5) @ 0x0801704e (474 bytes) */
void FUN_0801704e(uint32_t pulse_cnt, uint32_t start_freq_mHz, uint32_t end_freq_mHz,
                  uint32_t ramp_steps)
{
    uint16_t psc = 1U;
    uint32_t min_freq = (start_freq_mHz <= end_freq_mHz) ? start_freq_mHz : end_freq_mHz;

    if (min_freq < 146500UL) {
        psc = 9600U;
    } else if (min_freq < 1465000UL) {
        psc = 960U;
    } else if (min_freq < 14650000UL) {
        psc = 96U;
    } else if (min_freq < 73250000UL) {
        psc = 16U;
    } else {
        psc = 2U;
    }

    uint32_t timer_clk_mHz = (96000000UL / psc) * 100U;
    uint32_t arr_start = timer_clk_mHz / start_freq_mHz;
    uint32_t arr_end   = timer_clk_mHz / end_freq_mHz;

    REG16(TIM8_BASE + 0x28UL) = psc - 1U;                  /* TIM8_PSC */
    REG16(TIM8_BASE + 0x2CUL) = (uint16_t)(arr_start - 1U); /* TIM8_ARR */
    REG16(TIM8_BASE + 0x34UL) = (uint16_t)(arr_start >> 1); /* TIM8_CCR1 */
    REG16(TIM8_BASE + 0x14UL) |= 0x0001U;                  /* TIM8_EGR.UG = 1 */

    if (pulse_cnt < 0x10000UL) {
        REG16(TIM5_BASE + 0x2CUL) = (uint16_t)(pulse_cnt - 1U);
    } else {
        REG16(TIM5_BASE + 0x2CUL) = 19999U;
    }

    if (arr_start == arr_end) {
        RAM32(0x200002BCUL) = 0;
        RAM8(0x2000019AUL)  = 0;
        RAM8(0x20000189UL)  = 1U;
        return;
    }

    int32_t diff = (int32_t)(arr_end - arr_start);
    if (diff < 0) diff = -diff;
    uint32_t steps = ramp_steps;
    if (RAM32(0x20000438UL) != 1UL) {
        steps--;
    }

    uint64_t scaled_diff = (uint64_t)(uint32_t)diff * 10000ULL;
    uint64_t step_inc = FUN_0800016c((uint32_t)scaled_diff, (uint32_t)(scaled_diff >> 32),
                                     steps, 0);
    RAM32(0x200002E4UL) = (uint32_t)step_inc;
    RAM32(0x200000A8UL) = (uint32_t)step_inc;
    if ((uint32_t)step_inc < 10000UL) {
        RAM32(0x200002BCUL) = 1UL;
        RAM8(0x20000189UL)  = (uint8_t)(10000UL / (uint32_t)step_inc);
    } else {
        RAM32(0x200002BCUL) = (uint32_t)step_inc / 10000UL;
        RAM8(0x20000189UL)  = 1U;
    }
    RAM32(0x200002B8UL) = arr_start - 1U;
    RAM8(0x2000019AUL)  = (arr_start >= arr_end) ? 2U : 1U;
}

/* Function: FUN_08017228 (PLC_GPIO_InitDigitalOutputs_All) @ 0x08017228 (352 bytes) */
void FUN_08017228(void)
{
    uint8_t gpio_cfg[4];

    /* Disable JTAG (keep SWD enabled) so PA15 / PB3 / PB4 work as GPIO */
    FUN_080144ca(0x00300200UL, 1);

    /* PC0, PC1, PC8, PC9, PC11, PC12, PC13 (0x3B03) */
    gpio_cfg[0] = 0x03U;
    gpio_cfg[1] = 0x3BU;
    gpio_cfg[2] = 0x03U;
    gpio_cfg[3] = 0x10U;
    FUN_080143e4((volatile uint32_t *)GPIOC_BASE, gpio_cfg);
    FUN_08014526((volatile uint32_t *)GPIOC_BASE, 0x0300U);
    FUN_0801452a((volatile uint32_t *)GPIOC_BASE, 0x0800U);
    FUN_0801452a((volatile uint32_t *)GPIOC_BASE, 0x3003U);

    /* PA7, PA8, PA11, PA12, PA15 (0x9980) */
    gpio_cfg[0] = 0x80U;
    gpio_cfg[1] = 0x99U;
    gpio_cfg[2] = 0x03U;
    gpio_cfg[3] = 0x10U;
    FUN_080143e4((volatile uint32_t *)GPIOA_BASE, gpio_cfg);
    FUN_0801452a((volatile uint32_t *)GPIOA_BASE, 0x9980U);

    /* PB3 (0x0008) */
    gpio_cfg[0] = 0x08U;
    gpio_cfg[1] = 0x00U;
    gpio_cfg[2] = 0x03U;
    gpio_cfg[3] = 0x10U;
    FUN_080143e4((volatile uint32_t *)GPIOB_BASE, gpio_cfg);
    FUN_0801452a((volatile uint32_t *)GPIOB_BASE, 0x0008U);

    /* PD0..PD7 (0x00FF) */
    gpio_cfg[0] = 0xFFU;
    gpio_cfg[1] = 0x00U;
    gpio_cfg[2] = 0x03U;
    gpio_cfg[3] = 0x10U;
    FUN_080143e4((volatile uint32_t *)GPIOD_BASE, gpio_cfg);
    FUN_0801452a((volatile uint32_t *)GPIOD_BASE, 0x00FFU);

    /* PE3, PE5, PE14 (0x4028) */
    gpio_cfg[0] = 0x28U;
    gpio_cfg[1] = 0x40U;
    gpio_cfg[2] = 0x03U;
    gpio_cfg[3] = 0x10U;
    FUN_080143e4((volatile uint32_t *)GPIOE_BASE, gpio_cfg);
    FUN_0801452a((volatile uint32_t *)GPIOE_BASE, 0x4028U);
}

/* Function: FUN_08017388 (PLC_GPIO_InitHighSpeedOutputs_PA8_PC6) @ 0x08017388 (56 bytes) */
void FUN_08017388(void)
{
    uint8_t gpio_cfg[4];

    gpio_cfg[0] = 0x00U;
    gpio_cfg[1] = 0x01U; /* PA8 (TIM1_CH1 = Q0.0) */
    gpio_cfg[2] = 0x03U;
    gpio_cfg[3] = 0x10U;
    FUN_080143e4((volatile uint32_t *)GPIOA_BASE, gpio_cfg);

    gpio_cfg[0] = 0x40U; /* PC6 (TIM8_CH1 = Q0.1) */
    gpio_cfg[1] = 0x00U;
    gpio_cfg[2] = 0x03U;
    gpio_cfg[3] = 0x10U;
    FUN_080143e4((volatile uint32_t *)GPIOC_BASE, gpio_cfg);
}

/* Function: FUN_080173c0 (PLC_TraceStatus_RecordPacket) @ 0x080173c0 (144 bytes) */
void FUN_080173c0(uint8_t tag, uint8_t sub_tag)
{
    if (RAM8(0x200001D2UL) != 1U) {
        return;
    }
    uint32_t wr = RAM16(0x200001D4UL);
    RAM_PTR8(0x2000B918UL)[wr] = tag;
    RAM_PTR8(0x2000B919UL)[wr] = (uint8_t)(RAM16(0x200001CEUL) >> 8);
    RAM_PTR8(0x2000B91AUL)[wr] = (uint8_t)RAM16(0x200001CEUL);
    RAM_PTR8(0x2000B91BUL)[wr] = sub_tag;
    RAM_PTR8(0x2000B91CUL)[wr] = RAM8(0x200004FAUL);
    RAM_PTR8(0x2000B91DUL)[wr] = RAM8(0x200004FBUL);
    RAM_PTR8(0x2000B91EUL)[wr] = RAM8(0x200004FCUL);
    RAM_PTR8(0x2000B91FUL)[wr] = RAM8(0x200004FDUL);
    RAM16(0x200001D4UL) = (uint16_t)(wr + 8U);
}

/* ==========================================================================
 * Nested Subroutine (SBR_n) Execution Wrappers & Interrupt Arbiters
 * (0x08037928 .. 0x0803b100 / 0x0803b423)
 *
 * The S7-226 firmware implements up to 8 levels of Subroutine nesting from
 * OB1 main scan:
 *   Level 1: FUN_08037928 -> calls Level 2: FUN_08037f32
 *   Level 2: FUN_08037f32 -> calls Level 3: FUN_08038d08
 *   Level 3: FUN_08038d08 -> calls Level 4: FUN_0803928c
 *   Level 4: FUN_0803928c -> calls Level 5: FUN_08039826
 *   Level 5: FUN_08039826 -> calls Level 6: FUN_08039dce
 *   Level 6: FUN_08039dce -> calls Level 7: FUN_0803a376
 *   Level 7: FUN_0803a376 -> calls Level 8: FUN_0803a91e / FUN_0803aee6
 *
 * And 2 levels of Subroutine nesting from inside an Interrupt Routine (INT_n):
 *   ISR SBR Level 1: FUN_08038796 -> calls ISR SBR Level 2: FUN_080384b2
 * ========================================================================== */

/**
 * @brief Helper that arbitrates and dispatches pending I/O, Communication,
 *        and Timed interrupts after an inner subroutine or ISR finishes when
 *        interrupts are globally enabled (`SMB0` / `0x20005F29` bit 7).
 */
static inline void plc_check_and_dispatch_pending_interrupts(void)
{
    if (RAM8(0x20000501UL) == 0 || RAM8(0x2000051BUL) != 1U) {
        return;
    }

    RAM16(0x2000051CUL) = 0x0100U;
    FUN_08040ea0();

    if ((RAM8(0x20005F29UL) & 0x80U) != 0) {
        /* 1. Highest priority: I/O & High-Speed Counter Interrupt Queue */
        uint8_t io_cnt = RAM8(0x200003E9UL);
        if (io_cnt != 0 && RAM8(0x200003DCUL) == 0 && RAM8(0x200003DFUL) == 0) {
            if (io_cnt < 17U) {
                RAM8(0x200003DFUL) = 1U;
                uint8_t rd_idx = RAM8(0x200003E6UL);
                RAM8(0x200005F0UL) = RAM_PTR8(0x20000B3EUL)[rd_idx];
                rd_idx++;
                RAM8(0x200003E6UL) = (rd_idx > 15U) ? 0U : rd_idx;
                RAM8(0x200003E9UL) = (uint8_t)(io_cnt - 1U);
                FUN_08015402(RAM_PTR8(0x20000B58UL)[RAM8(0x200005F0UL)]);
                RAM8(0x200003DFUL) = 0;
            } else {
                RAM8(0x20005F2CUL) |= 0x02U; /* I/O queue overflow */
            }
        }

        /* 2. Medium priority: Communication (Freeport / UART) Interrupt Queue */
        uint8_t comm_cnt = RAM8(0x200003E8UL);
        if (comm_cnt != 0 && RAM8(0x200003DDUL) == 0 &&
            RAM8(0x200003E0UL) == 0 && RAM8(0x200003DFUL) == 0) {
            if (comm_cnt < 5U) {
                RAM8(0x200003E0UL) = 1U;
                uint8_t rd_idx = RAM8(0x200003E5UL);
                RAM8(0x200005F0UL) = RAM_PTR8(0x20000B52UL)[rd_idx];
                rd_idx++;
                RAM8(0x200003E5UL) = (rd_idx > 3U) ? 0U : rd_idx;
                RAM8(0x200003E8UL) = (uint8_t)(comm_cnt - 1U);
                FUN_08015402(RAM_PTR8(0x20000B58UL)[RAM8(0x200005F0UL)]);
                RAM8(0x200003E0UL) = 0;
            } else {
                RAM8(0x20005F2CUL) |= 0x01U; /* Comm queue overflow */
            }
        }

        /* 3. Lowest priority: Timed Interrupt Queue (SMB34/SMB35, T32/T96) */
        uint8_t tim_cnt = RAM8(0x200003EAUL);
        if (tim_cnt != 0 && RAM8(0x200003DEUL) == 0 && RAM8(0x200003E1UL) == 0 &&
            RAM8(0x200003DFUL) == 0 && RAM8(0x200003E0UL) == 0) {
            if (tim_cnt < 9U) {
                RAM8(0x200003E1UL) = 1U;
                uint8_t rd_idx = RAM8(0x200003E7UL);
                RAM8(0x200005F0UL) = RAM_PTR8(0x20000B4EUL)[rd_idx];
                rd_idx++;
                RAM8(0x200003E7UL) = (rd_idx > 3U) ? 0U : rd_idx;
                RAM8(0x200003EAUL) = (uint8_t)(tim_cnt - 1U);
                FUN_08015402(RAM_PTR8(0x20000B58UL)[RAM8(0x200005F0UL)]);
                RAM8(0x200003E1UL) = 0;
            } else {
                RAM8(0x20005F2CUL) |= 0x04U; /* Timed queue overflow */
            }
        }
    }

    FUN_080421b4();
}

/* ==========================================================================
 * Function: FUN_08037928 (PLC_ExecuteSBR_Level1) @ 0x08037928 (1546 bytes)
 * ========================================================================== */
void FUN_08037928(uint16_t sbr_id)
{
    uint8_t saved_l_mem[64];
    uint8_t saved_stack[36];
    uint16_t i;

    uint32_t param_bytes = RAM_PTR8(0x20000D1CUL)[sbr_id];
    for (i = 0; i < param_bytes; i++) {
        RAM_PTR8(0x20006110UL)[i] = RAM_PTR8(0x20006110UL)[i + 64U];
    }
    for (; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i] =
            RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U];
    }

    RAM32(0x200001DCUL) = RAM_PTR16(0x20000C1CUL)[sbr_id];

    do {
        if (RAM32(0x200001DCUL) >= RAM32(0x200001E0UL)) {
            break;
        }
        uint32_t pc_off = RAM32(0x200001DCUL) * 2U;
        for (i = 0; i < 67U && i < RAM_PTR8(0x0806D800UL)[pc_off]; i += 2U) {
            uint16_t word_idx = i >> 1;
            uint32_t byte_pos = pc_off + (uint32_t)word_idx * 2U;
            RAM_PTR8(0x20000496UL)[i]      = FLASH_PTR8(0x0806D800UL)[byte_pos];
            RAM_PTR8(0x20000496UL)[i + 1U] = FLASH_PTR8(0x0806D801UL)[byte_pos];
        }

        RAM32(0x200001DCUL) += (uint32_t)(i >> 1);
        int32_t exec_res = FUN_08021948();
        uint16_t target_id = (uint16_t)exec_res;

        if (exec_res >= 0) {
            uint32_t saved_pc = RAM32(0x200001DCUL);
            for (i = 0; i < 64U; i++) {
                saved_l_mem[i] = RAM_PTR8(0x20006110UL)[i];
            }
            for (i = 0; i < 36U; i++) {
                saved_stack[i] = RAM_PTR8(0x200004DCUL)[i];
            }
            uint8_t saved_for_idx = RAM8(0x20000500UL);
            uint8_t saved_rlo     = RAM8(0x200001D6UL);

            if (target_id < 0x00C9U) {
                FUN_08037f32(target_id);
                plc_check_and_dispatch_pending_interrupts();
            } else {
                FUN_08015402((uint16_t)(target_id - 0x00C9U));
            }

            RAM8(0x200001D6UL)  = saved_rlo;
            RAM8(0x20000500UL)  = saved_for_idx;
            for (i = 0; i < 36U; i++) {
                RAM_PTR8(0x200004DCUL)[i] = saved_stack[i];
            }
            for (i = 0; i < 64U; i++) {
                RAM_PTR8(0x20006110UL)[i] = saved_l_mem[i];
            }
            RAM32(0x200001DCUL) = saved_pc;
        } else if (exec_res == -10) {
            FUN_0801e2b4();
            break;
        } else if (exec_res == -0x16) {
            /* RET (Return from Subroutine) */
            break;
        }
    } while (true);

    for (i = 0; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i + 64U] = RAM_PTR8(0x20006110UL)[i];
    }
    for (i = (uint16_t)param_bytes; i < 64U; i++) {
        RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U] =
            RAM_PTR8(0x20006110UL)[i];
    }
}

/* ==========================================================================
 * Function: FUN_08037f32 (PLC_ExecuteSBR_Level2) @ 0x08037f32 (1408 bytes)
 * ========================================================================== */
void FUN_08037f32(uint16_t sbr_id)
{
    uint8_t saved_l_mem[64];
    uint8_t saved_stack[36];
    uint16_t i;

    uint32_t param_bytes = RAM_PTR8(0x20000D1CUL)[sbr_id];
    for (i = 0; i < param_bytes; i++) {
        RAM_PTR8(0x20006110UL)[i] = RAM_PTR8(0x20006110UL)[i + 64U];
    }
    for (; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i] =
            RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U];
    }

    RAM32(0x200001DCUL) = RAM_PTR16(0x20000C1CUL)[sbr_id];

    do {
        if (RAM32(0x200001DCUL) >= RAM32(0x200001E0UL)) {
            break;
        }
        uint32_t pc_off = RAM32(0x200001DCUL) * 2U;
        for (i = 0; i < 67U && i < RAM_PTR8(0x0806D800UL)[pc_off]; i += 2U) {
            uint16_t word_idx = i >> 1;
            uint32_t byte_pos = pc_off + (uint32_t)word_idx * 2U;
            RAM_PTR8(0x20000496UL)[i]      = FLASH_PTR8(0x0806D800UL)[byte_pos];
            RAM_PTR8(0x20000496UL)[i + 1U] = FLASH_PTR8(0x0806D801UL)[byte_pos];
        }

        RAM32(0x200001DCUL) += (uint32_t)(i >> 1);
        int32_t exec_res = FUN_08021948();
        uint16_t target_id = (uint16_t)exec_res;

        if (exec_res >= 0) {
            uint32_t saved_pc = RAM32(0x200001DCUL);
            for (i = 0; i < 64U; i++) {
                saved_l_mem[i] = RAM_PTR8(0x20006110UL)[i];
            }
            for (i = 0; i < 36U; i++) {
                saved_stack[i] = RAM_PTR8(0x200004DCUL)[i];
            }
            uint8_t saved_for_idx = RAM8(0x20000500UL);
            uint8_t saved_rlo     = RAM8(0x200001D6UL);

            if (target_id < 0x00C9U) {
                FUN_08038d08(target_id);
                plc_check_and_dispatch_pending_interrupts();
            } else {
                FUN_08015402((uint16_t)(target_id - 0x00C9U));
            }

            RAM8(0x200001D6UL)  = saved_rlo;
            RAM8(0x20000500UL)  = saved_for_idx;
            for (i = 0; i < 36U; i++) {
                RAM_PTR8(0x200004DCUL)[i] = saved_stack[i];
            }
            for (i = 0; i < 64U; i++) {
                RAM_PTR8(0x20006110UL)[i] = saved_l_mem[i];
            }
            RAM32(0x200001DCUL) = saved_pc;
        } else if (exec_res == -10) {
            FUN_0801e2b4();
            break;
        } else if (exec_res == -0x16) {
            break;
        }
    } while (true);

    for (i = 0; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i + 64U] = RAM_PTR8(0x20006110UL)[i];
    }
    for (i = (uint16_t)param_bytes; i < 64U; i++) {
        RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U] =
            RAM_PTR8(0x20006110UL)[i];
    }
}

/* ==========================================================================
 * Function: FUN_080384b2 (PLC_ExecuteSBR_FromISR_Level2) @ 0x080384b2 (740 bytes)
 * ========================================================================== */
void FUN_080384b2(uint16_t sbr_id)
{
    uint16_t i;
    uint32_t param_bytes = RAM_PTR8(0x20000D1CUL)[sbr_id];

    for (i = 0; i < param_bytes; i++) {
        RAM_PTR8(0x20006110UL)[i] = RAM_PTR8(0x20006110UL)[i + 64U];
    }
    for (; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i] =
            RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U];
    }

    RAM32(0x200001DCUL) = RAM_PTR16(0x20000C1CUL)[sbr_id];

    do {
        if (RAM32(0x200001DCUL) >= RAM32(0x200001E0UL)) {
            break;
        }
        uint32_t pc_off = RAM32(0x200001DCUL) * 2U;
        for (i = 0; i < 67U && i < RAM_PTR8(0x0806D800UL)[pc_off]; i += 2U) {
            uint16_t word_idx = i >> 1;
            uint32_t byte_pos = pc_off + (uint32_t)word_idx * 2U;
            RAM_PTR8(0x20000496UL)[i]      = FLASH_PTR8(0x0806D800UL)[byte_pos];
            RAM_PTR8(0x20000496UL)[i + 1U] = FLASH_PTR8(0x0806D801UL)[byte_pos];
        }

        RAM32(0x200001DCUL) += (uint32_t)(i >> 1);
        int32_t exec_res = FUN_08021948();
        if (exec_res == -10) {
            FUN_0801e2b4();
            break;
        }
        if (exec_res == -0x16) {
            break;
        }
    } while (true);

    for (i = 0; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i + 64U] = RAM_PTR8(0x20006110UL)[i];
    }
    for (i = (uint16_t)param_bytes; i < 64U; i++) {
        RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U] =
            RAM_PTR8(0x20006110UL)[i];
    }
}

/* ==========================================================================
 * Function: FUN_08038796 (PLC_ExecuteSBR_FromISR_Level1) @ 0x08038796 (1394 bytes)
 * ========================================================================== */
void FUN_08038796(uint16_t sbr_id)
{
    uint8_t saved_l_mem[64];
    uint8_t saved_stack[36];
    uint16_t i;

    uint32_t param_bytes = RAM_PTR8(0x20000D1CUL)[sbr_id];
    for (i = 0; i < param_bytes; i++) {
        RAM_PTR8(0x20006110UL)[i] = RAM_PTR8(0x20006110UL)[i + 64U];
    }
    for (; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i] =
            RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U];
    }

    RAM32(0x200001DCUL) = RAM_PTR16(0x20000C1CUL)[sbr_id];

    do {
        if (RAM32(0x200001DCUL) >= RAM32(0x200001E0UL)) {
            break;
        }
        uint32_t pc_off = RAM32(0x200001DCUL) * 2U;
        for (i = 0; i < 67U && i < RAM_PTR8(0x0806D800UL)[pc_off]; i += 2U) {
            uint16_t word_idx = i >> 1;
            uint32_t byte_pos = pc_off + (uint32_t)word_idx * 2U;
            RAM_PTR8(0x20000496UL)[i]      = FLASH_PTR8(0x0806D800UL)[byte_pos];
            RAM_PTR8(0x20000496UL)[i + 1U] = FLASH_PTR8(0x0806D801UL)[byte_pos];
        }

        RAM32(0x200001DCUL) += (uint32_t)(i >> 1);
        int32_t exec_res = FUN_08021948();
        uint16_t target_id = (uint16_t)exec_res;

        if (exec_res >= 0) {
            uint32_t saved_pc = RAM32(0x200001DCUL);
            for (i = 0; i < 64U; i++) {
                saved_l_mem[i] = RAM_PTR8(0x20006110UL)[i];
            }
            for (i = 0; i < 36U; i++) {
                saved_stack[i] = RAM_PTR8(0x200004DCUL)[i];
            }
            uint8_t saved_for_idx = RAM8(0x20000500UL);
            uint8_t saved_rlo     = RAM8(0x200001D6UL);

            if (target_id < 0x00C9U) {
                FUN_080384b2(target_id);
                if (RAM8(0x20000501UL) != 0 && RAM8(0x2000051BUL) == 1U) {
                    RAM16(0x2000051CUL) = 0x0100U;
                    FUN_08040ea0();
                    FUN_080421b4();
                }
            }

            RAM8(0x200001D6UL)  = saved_rlo;
            RAM8(0x20000500UL)  = saved_for_idx;
            for (i = 0; i < 36U; i++) {
                RAM_PTR8(0x200004DCUL)[i] = saved_stack[i];
            }
            for (i = 0; i < 64U; i++) {
                RAM_PTR8(0x20006110UL)[i] = saved_l_mem[i];
            }
            RAM32(0x200001DCUL) = saved_pc;
        } else if (exec_res == -10) {
            FUN_0801e2b4();
            break;
        } else if (exec_res == -0x16) {
            break;
        }
    } while (true);

    for (i = 0; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i + 64U] = RAM_PTR8(0x20006110UL)[i];
    }
    for (i = (uint16_t)param_bytes; i < 64U; i++) {
        RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U] =
            RAM_PTR8(0x20006110UL)[i];
    }
}

/* ==========================================================================
 * Function: FUN_08038d08 (PLC_ExecuteSBR_Level3) @ 0x08038d08 (1412 bytes)
 * ========================================================================== */
void FUN_08038d08(uint16_t sbr_id)
{
    uint8_t saved_l_mem[64];
    uint8_t saved_stack[36];
    uint16_t i;

    uint32_t param_bytes = RAM_PTR8(0x20000D1CUL)[sbr_id];
    for (i = 0; i < param_bytes; i++) {
        RAM_PTR8(0x20006110UL)[i] = RAM_PTR8(0x20006110UL)[i + 64U];
    }
    for (; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i] =
            RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U];
    }

    RAM32(0x200001DCUL) = RAM_PTR16(0x20000C1CUL)[sbr_id];

    do {
        if (RAM32(0x200001DCUL) >= RAM32(0x200001E0UL)) {
            break;
        }
        uint32_t pc_off = RAM32(0x200001DCUL) * 2U;
        for (i = 0; i < 67U && i < RAM_PTR8(0x0806D800UL)[pc_off]; i += 2U) {
            uint16_t word_idx = i >> 1;
            uint32_t byte_pos = pc_off + (uint32_t)word_idx * 2U;
            RAM_PTR8(0x20000496UL)[i]      = FLASH_PTR8(0x0806D800UL)[byte_pos];
            RAM_PTR8(0x20000496UL)[i + 1U] = FLASH_PTR8(0x0806D801UL)[byte_pos];
        }

        RAM32(0x200001DCUL) += (uint32_t)(i >> 1);
        int32_t exec_res = FUN_08021948();
        uint16_t target_id = (uint16_t)exec_res;

        if (exec_res >= 0) {
            uint32_t saved_pc = RAM32(0x200001DCUL);
            for (i = 0; i < 64U; i++) {
                saved_l_mem[i] = RAM_PTR8(0x20006110UL)[i];
            }
            for (i = 0; i < 36U; i++) {
                saved_stack[i] = RAM_PTR8(0x200004DCUL)[i];
            }
            uint8_t saved_for_idx = RAM8(0x20000500UL);
            uint8_t saved_rlo     = RAM8(0x200001D6UL);

            if (target_id < 0x00C9U) {
                FUN_0803928c(target_id);
                plc_check_and_dispatch_pending_interrupts();
            } else {
                FUN_08015402((uint16_t)(target_id - 0x00C9U));
            }

            RAM8(0x200001D6UL)  = saved_rlo;
            RAM8(0x20000500UL)  = saved_for_idx;
            for (i = 0; i < 36U; i++) {
                RAM_PTR8(0x200004DCUL)[i] = saved_stack[i];
            }
            for (i = 0; i < 64U; i++) {
                RAM_PTR8(0x20006110UL)[i] = saved_l_mem[i];
            }
            RAM32(0x200001DCUL) = saved_pc;
        } else if (exec_res == -10) {
            FUN_0801e2b4();
            break;
        } else if (exec_res == -0x16) {
            break;
        }
    } while (true);

    for (i = 0; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i + 64U] = RAM_PTR8(0x20006110UL)[i];
    }
    for (i = (uint16_t)param_bytes; i < 64U; i++) {
        RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U] =
            RAM_PTR8(0x20006110UL)[i];
    }
}

/* ==========================================================================
 * Function: FUN_0803928c (PLC_ExecuteSBR_Level4) @ 0x0803928c (1434 bytes)
 * ========================================================================== */
void FUN_0803928c(uint16_t sbr_id)
{
    uint8_t saved_l_mem[64];
    uint8_t saved_stack[36];
    uint16_t i;

    uint32_t param_bytes = RAM_PTR8(0x20000D1CUL)[sbr_id];
    for (i = 0; i < param_bytes; i++) {
        RAM_PTR8(0x20006110UL)[i] = RAM_PTR8(0x20006110UL)[i + 64U];
    }
    for (; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i] =
            RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U];
    }

    RAM32(0x200001DCUL) = RAM_PTR16(0x20000C1CUL)[sbr_id];

    do {
        if (RAM32(0x200001DCUL) >= RAM32(0x200001E0UL)) {
            break;
        }
        uint32_t pc_off = RAM32(0x200001DCUL) * 2U;
        for (i = 0; i < 67U && i < RAM_PTR8(0x0806D800UL)[pc_off]; i += 2U) {
            uint16_t word_idx = i >> 1;
            uint32_t byte_pos = pc_off + (uint32_t)word_idx * 2U;
            RAM_PTR8(0x20000496UL)[i]      = FLASH_PTR8(0x0806D800UL)[byte_pos];
            RAM_PTR8(0x20000496UL)[i + 1U] = FLASH_PTR8(0x0806D801UL)[byte_pos];
        }

        RAM32(0x200001DCUL) += (uint32_t)(i >> 1);
        int32_t exec_res = FUN_08021948();
        uint16_t target_id = (uint16_t)exec_res;

        if (exec_res >= 0) {
            uint32_t saved_pc = RAM32(0x200001DCUL);
            for (i = 0; i < 64U; i++) {
                saved_l_mem[i] = RAM_PTR8(0x20006110UL)[i];
            }
            for (i = 0; i < 36U; i++) {
                saved_stack[i] = RAM_PTR8(0x200004DCUL)[i];
            }
            uint8_t saved_for_idx = RAM8(0x20000500UL);
            uint8_t saved_rlo     = RAM8(0x200001D6UL);

            if (target_id < 0x00C9U) {
                FUN_08039826(target_id);
                plc_check_and_dispatch_pending_interrupts();
            } else {
                FUN_08015402((uint16_t)(target_id - 0x00C9U));
            }

            RAM8(0x200001D6UL)  = saved_rlo;
            RAM8(0x20000500UL)  = saved_for_idx;
            for (i = 0; i < 36U; i++) {
                RAM_PTR8(0x200004DCUL)[i] = saved_stack[i];
            }
            for (i = 0; i < 64U; i++) {
                RAM_PTR8(0x20006110UL)[i] = saved_l_mem[i];
            }
            RAM32(0x200001DCUL) = saved_pc;
        } else if (exec_res == -10) {
            FUN_0801e2b4();
            break;
        } else if (exec_res == -0x16) {
            break;
        }
    } while (true);

    for (i = 0; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i + 64U] = RAM_PTR8(0x20006110UL)[i];
    }
    for (i = (uint16_t)param_bytes; i < 64U; i++) {
        RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U] =
            RAM_PTR8(0x20006110UL)[i];
    }
}

/* ==========================================================================
 * Function: FUN_08039826 (PLC_ExecuteSBR_Level5) @ 0x08039826 (1448 bytes)
 * ========================================================================== */
void FUN_08039826(uint16_t sbr_id)
{
    uint8_t saved_l_mem[64];
    uint8_t saved_stack[36];
    uint16_t i;

    uint32_t param_bytes = RAM_PTR8(0x20000D1CUL)[sbr_id];
    for (i = 0; i < param_bytes; i++) {
        RAM_PTR8(0x20006110UL)[i] = RAM_PTR8(0x20006110UL)[i + 64U];
    }
    for (; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i] =
            RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U];
    }

    RAM32(0x200001DCUL) = RAM_PTR16(0x20000C1CUL)[sbr_id];

    do {
        if (RAM32(0x200001DCUL) >= RAM32(0x200001E0UL)) {
            break;
        }
        uint32_t pc_off = RAM32(0x200001DCUL) * 2U;
        for (i = 0; i < 67U && i < RAM_PTR8(0x0806D800UL)[pc_off]; i += 2U) {
            uint16_t word_idx = i >> 1;
            uint32_t byte_pos = pc_off + (uint32_t)word_idx * 2U;
            RAM_PTR8(0x20000496UL)[i]      = FLASH_PTR8(0x0806D800UL)[byte_pos];
            RAM_PTR8(0x20000496UL)[i + 1U] = FLASH_PTR8(0x0806D801UL)[byte_pos];
        }

        RAM32(0x200001DCUL) += (uint32_t)(i >> 1);
        int32_t exec_res = FUN_08021948();
        uint16_t target_id = (uint16_t)exec_res;

        if (exec_res >= 0) {
            uint32_t saved_pc = RAM32(0x200001DCUL);
            for (i = 0; i < 64U; i++) {
                saved_l_mem[i] = RAM_PTR8(0x20006110UL)[i];
            }
            for (i = 0; i < 36U; i++) {
                saved_stack[i] = RAM_PTR8(0x200004DCUL)[i];
            }
            uint8_t saved_for_idx = RAM8(0x20000500UL);
            uint8_t saved_rlo     = RAM8(0x200001D6UL);

            if (target_id < 0x00C9U) {
                FUN_08039dce(target_id);
                plc_check_and_dispatch_pending_interrupts();
            } else {
                FUN_08015402((uint16_t)(target_id - 0x00C9U));
            }

            RAM8(0x200001D6UL)  = saved_rlo;
            RAM8(0x20000500UL)  = saved_for_idx;
            for (i = 0; i < 36U; i++) {
                RAM_PTR8(0x200004DCUL)[i] = saved_stack[i];
            }
            for (i = 0; i < 64U; i++) {
                RAM_PTR8(0x20006110UL)[i] = saved_l_mem[i];
            }
            RAM32(0x200001DCUL) = saved_pc;
        } else if (exec_res == -10) {
            FUN_0801e2b4();
            break;
        } else if (exec_res == -0x16) {
            break;
        }
    } while (true);

    for (i = 0; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i + 64U] = RAM_PTR8(0x20006110UL)[i];
    }
    for (i = (uint16_t)param_bytes; i < 64U; i++) {
        RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U] =
            RAM_PTR8(0x20006110UL)[i];
    }
}

/* ==========================================================================
 * Function: FUN_08039dce (PLC_ExecuteSBR_Level6) @ 0x08039dce (1448 bytes)
 * ========================================================================== */
void FUN_08039dce(uint16_t sbr_id)
{
    uint8_t saved_l_mem[64];
    uint8_t saved_stack[36];
    uint16_t i;

    uint32_t param_bytes = RAM_PTR8(0x20000D1CUL)[sbr_id];
    for (i = 0; i < param_bytes; i++) {
        RAM_PTR8(0x20006110UL)[i] = RAM_PTR8(0x20006110UL)[i + 64U];
    }
    for (; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i] =
            RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U];
    }

    RAM32(0x200001DCUL) = RAM_PTR16(0x20000C1CUL)[sbr_id];

    do {
        if (RAM32(0x200001DCUL) >= RAM32(0x200001E0UL)) {
            break;
        }
        uint32_t pc_off = RAM32(0x200001DCUL) * 2U;
        for (i = 0; i < 67U && i < RAM_PTR8(0x0806D800UL)[pc_off]; i += 2U) {
            uint16_t word_idx = i >> 1;
            uint32_t byte_pos = pc_off + (uint32_t)word_idx * 2U;
            RAM_PTR8(0x20000496UL)[i]      = FLASH_PTR8(0x0806D800UL)[byte_pos];
            RAM_PTR8(0x20000496UL)[i + 1U] = FLASH_PTR8(0x0806D801UL)[byte_pos];
        }

        RAM32(0x200001DCUL) += (uint32_t)(i >> 1);
        int32_t exec_res = FUN_08021948();
        uint16_t target_id = (uint16_t)exec_res;

        if (exec_res >= 0) {
            uint32_t saved_pc = RAM32(0x200001DCUL);
            for (i = 0; i < 64U; i++) {
                saved_l_mem[i] = RAM_PTR8(0x20006110UL)[i];
            }
            for (i = 0; i < 36U; i++) {
                saved_stack[i] = RAM_PTR8(0x200004DCUL)[i];
            }
            uint8_t saved_for_idx = RAM8(0x20000500UL);
            uint8_t saved_rlo     = RAM8(0x200001D6UL);

            if (target_id < 0x00C9U) {
                FUN_0803a376(target_id);
                plc_check_and_dispatch_pending_interrupts();
            } else {
                FUN_08015402((uint16_t)(target_id - 0x00C9U));
            }

            RAM8(0x200001D6UL)  = saved_rlo;
            RAM8(0x20000500UL)  = saved_for_idx;
            for (i = 0; i < 36U; i++) {
                RAM_PTR8(0x200004DCUL)[i] = saved_stack[i];
            }
            for (i = 0; i < 64U; i++) {
                RAM_PTR8(0x20006110UL)[i] = saved_l_mem[i];
            }
            RAM32(0x200001DCUL) = saved_pc;
        } else if (exec_res == -10) {
            FUN_0801e2b4();
            break;
        } else if (exec_res == -0x16) {
            break;
        }
    } while (true);

    for (i = 0; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i + 64U] = RAM_PTR8(0x20006110UL)[i];
    }
    for (i = (uint16_t)param_bytes; i < 64U; i++) {
        RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U] =
            RAM_PTR8(0x20006110UL)[i];
    }
}

/* ==========================================================================
 * Function: FUN_0803a376 (PLC_ExecuteSBR_Level7) @ 0x0803a376 (1448 bytes)
 * ========================================================================== */
void FUN_0803a376(uint16_t sbr_id)
{
    uint8_t saved_l_mem[64];
    uint8_t saved_stack[36];
    uint16_t i;

    uint32_t param_bytes = RAM_PTR8(0x20000D1CUL)[sbr_id];
    for (i = 0; i < param_bytes; i++) {
        RAM_PTR8(0x20006110UL)[i] = RAM_PTR8(0x20006110UL)[i + 64U];
    }
    for (; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i] =
            RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U];
    }

    RAM32(0x200001DCUL) = RAM_PTR16(0x20000C1CUL)[sbr_id];

    do {
        if (RAM32(0x200001DCUL) >= RAM32(0x200001E0UL)) {
            break;
        }
        uint32_t pc_off = RAM32(0x200001DCUL) * 2U;
        for (i = 0; i < 67U && i < RAM_PTR8(0x0806D800UL)[pc_off]; i += 2U) {
            uint16_t word_idx = i >> 1;
            uint32_t byte_pos = pc_off + (uint32_t)word_idx * 2U;
            RAM_PTR8(0x20000496UL)[i]      = FLASH_PTR8(0x0806D800UL)[byte_pos];
            RAM_PTR8(0x20000496UL)[i + 1U] = FLASH_PTR8(0x0806D801UL)[byte_pos];
        }

        RAM32(0x200001DCUL) += (uint32_t)(i >> 1);
        int32_t exec_res = FUN_08021948();
        uint16_t target_id = (uint16_t)exec_res;

        if (exec_res >= 0) {
            uint32_t saved_pc = RAM32(0x200001DCUL);
            for (i = 0; i < 64U; i++) {
                saved_l_mem[i] = RAM_PTR8(0x20006110UL)[i];
            }
            for (i = 0; i < 36U; i++) {
                saved_stack[i] = RAM_PTR8(0x200004DCUL)[i];
            }
            uint8_t saved_for_idx = RAM8(0x20000500UL);
            uint8_t saved_rlo     = RAM8(0x200001D6UL);

            if (target_id < 0x00C9U) {
                FUN_0803a91e(target_id);
                plc_check_and_dispatch_pending_interrupts();
            } else {
                FUN_08015402((uint16_t)(target_id - 0x00C9U));
            }

            RAM8(0x200001D6UL)  = saved_rlo;
            RAM8(0x20000500UL)  = saved_for_idx;
            for (i = 0; i < 36U; i++) {
                RAM_PTR8(0x200004DCUL)[i] = saved_stack[i];
            }
            for (i = 0; i < 64U; i++) {
                RAM_PTR8(0x20006110UL)[i] = saved_l_mem[i];
            }
            RAM32(0x200001DCUL) = saved_pc;
        } else if (exec_res == -10) {
            FUN_0801e2b4();
            break;
        } else if (exec_res == -0x16) {
            break;
        }
    } while (true);

    for (i = 0; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i + 64U] = RAM_PTR8(0x20006110UL)[i];
    }
    for (i = (uint16_t)param_bytes; i < 64U; i++) {
        RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U] =
            RAM_PTR8(0x20006110UL)[i];
    }
}

/* ==========================================================================
 * Function: FUN_0803a91e (PLC_ExecuteSBR_Level8A) @ 0x0803a91e (1480 bytes)
 * ========================================================================== */
void FUN_0803a91e(uint16_t sbr_id)
{
    uint8_t saved_l_mem[64];
    uint8_t saved_stack[36];
    uint16_t i;

    uint32_t param_bytes = RAM_PTR8(0x20000D1CUL)[sbr_id];
    for (i = 0; i < param_bytes; i++) {
        RAM_PTR8(0x20006110UL)[i] = RAM_PTR8(0x20006110UL)[i + 64U];
    }
    for (; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i] =
            RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U];
    }

    RAM32(0x200001DCUL) = RAM_PTR16(0x20000C1CUL)[sbr_id];

    do {
        if (RAM32(0x200001DCUL) >= RAM32(0x200001E0UL)) {
            break;
        }
        uint32_t pc_off = RAM32(0x200001DCUL) * 2U;
        for (i = 0; i < 67U && i < RAM_PTR8(0x0806D800UL)[pc_off]; i += 2U) {
            uint16_t word_idx = i >> 1;
            uint32_t byte_pos = pc_off + (uint32_t)word_idx * 2U;
            RAM_PTR8(0x20000496UL)[i]      = FLASH_PTR8(0x0806D800UL)[byte_pos];
            RAM_PTR8(0x20000496UL)[i + 1U] = FLASH_PTR8(0x0806D801UL)[byte_pos];
        }

        RAM32(0x200001DCUL) += (uint32_t)(i >> 1);
        int32_t exec_res = FUN_08021948();
        uint16_t target_id = (uint16_t)exec_res;

        if (exec_res >= 0) {
            uint32_t saved_pc = RAM32(0x200001DCUL);
            for (i = 0; i < 64U; i++) {
                saved_l_mem[i] = RAM_PTR8(0x20006110UL)[i];
            }
            for (i = 0; i < 36U; i++) {
                saved_stack[i] = RAM_PTR8(0x200004DCUL)[i];
            }
            uint8_t saved_for_idx = RAM8(0x20000500UL);
            uint8_t saved_rlo     = RAM8(0x200001D6UL);

            if (target_id < 0x00C9U) {
                FUN_0803aee6(target_id);
                plc_check_and_dispatch_pending_interrupts();
            } else {
                FUN_08015402((uint16_t)(target_id - 0x00C9U));
            }

            RAM8(0x200001D6UL)  = saved_rlo;
            RAM8(0x20000500UL)  = saved_for_idx;
            for (i = 0; i < 36U; i++) {
                RAM_PTR8(0x200004DCUL)[i] = saved_stack[i];
            }
            for (i = 0; i < 64U; i++) {
                RAM_PTR8(0x20006110UL)[i] = saved_l_mem[i];
            }
            RAM32(0x200001DCUL) = saved_pc;
        } else if (exec_res == -10) {
            FUN_0801e2b4();
            break;
        } else if (exec_res == -0x16) {
            break;
        }
    } while (true);

    for (i = 0; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i + 64U] = RAM_PTR8(0x20006110UL)[i];
    }
    for (i = (uint16_t)param_bytes; i < 64U; i++) {
        RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U] =
            RAM_PTR8(0x20006110UL)[i];
    }
}

/* ==========================================================================
 * Function: FUN_0803aee6 (PLC_ExecuteSBR_Level8B_Leaf) @ 0x0803aee6 (1342 bytes)
 *
 * Leaf subroutine execution wrapper (0x0803aee6 .. 0x0803b423):
 *   Executes user SBR_n bytecode at the deepest nesting level and still
 *   allows asynchronous interrupt routines (INT_n, target_id >= 0xC9) to
 *   preempt between instructions.
 * ========================================================================== */
void FUN_0803aee6(uint16_t sbr_id)
{
    uint8_t saved_l_mem[64];
    uint8_t saved_stack[36];
    uint16_t i;

    uint32_t param_bytes = RAM_PTR8(0x20000D1CUL)[sbr_id];
    for (i = 0; i < param_bytes; i++) {
        RAM_PTR8(0x20006110UL)[i] = RAM_PTR8(0x20006110UL)[i + 64U];
    }
    for (; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i] =
            RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U];
    }

    RAM32(0x200001DCUL) = RAM_PTR16(0x20000C1CUL)[sbr_id];

    do {
        if (RAM32(0x200001DCUL) >= RAM32(0x200001E0UL)) {
            break;
        }
        uint32_t pc_off = RAM32(0x200001DCUL) * 2U;
        for (i = 0; i < 67U && i < RAM_PTR8(0x0806D800UL)[pc_off]; i += 2U) {
            uint16_t word_idx = i >> 1;
            uint32_t byte_pos = pc_off + (uint32_t)word_idx * 2U;
            RAM_PTR8(0x20000496UL)[i]      = FLASH_PTR8(0x0806D800UL)[byte_pos];
            RAM_PTR8(0x20000496UL)[i + 1U] = FLASH_PTR8(0x0806D801UL)[byte_pos];
        }

        RAM32(0x200001DCUL) += (uint32_t)(i >> 1);
        int32_t exec_res = FUN_08021948();
        uint16_t target_id = (uint16_t)exec_res;

        if (exec_res >= 0) {
            uint32_t saved_pc = RAM32(0x200001DCUL);
            for (i = 0; i < 64U; i++) {
                saved_l_mem[i] = RAM_PTR8(0x20006110UL)[i];
            }
            for (i = 0; i < 36U; i++) {
                saved_stack[i] = RAM_PTR8(0x200004DCUL)[i];
            }
            uint8_t saved_for_idx = RAM8(0x20000500UL);
            uint8_t saved_rlo     = RAM8(0x200001D6UL);

            if (target_id < 0x00C9U) {
                plc_check_and_dispatch_pending_interrupts();
            } else {
                FUN_08015402((uint16_t)(target_id - 0x00C9U));
            }

            RAM8(0x200001D6UL)  = saved_rlo;
            RAM8(0x20000500UL)  = saved_for_idx;
            for (i = 0; i < 36U; i++) {
                RAM_PTR8(0x200004DCUL)[i] = saved_stack[i];
            }
            for (i = 0; i < 64U; i++) {
                RAM_PTR8(0x20006110UL)[i] = saved_l_mem[i];
            }
            RAM32(0x200001DCUL) = saved_pc;
        } else if (exec_res == -10) {
            FUN_0801e2b4();
            break;
        } else if (exec_res == -0x16) {
            break;
        }
    } while (true);

    for (i = 0; i < 64U; i++) {
        RAM_PTR8(0x20006110UL)[i + 64U] = RAM_PTR8(0x20006110UL)[i];
    }
    for (i = (uint16_t)param_bytes; i < 64U; i++) {
        RAM_PTR8(0x2000BBD8UL)[i + (uint32_t)sbr_id * 64U] =
            RAM_PTR8(0x20006110UL)[i];
    }
}

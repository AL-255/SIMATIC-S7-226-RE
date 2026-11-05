/**
 * @file plc_hsc_exti.c
 * @brief High-Speed Counter (HSC0..HSC5), External Interrupts (EXTI0..EXTI15_10),
 *        PTO0/PTO1 Envelope Segment Advancement (EXTI2), and Expansion Bus
 *        Bit-Banged SPI Primitives (0x0800fb4e .. 0x08013ca7).
 *
 * Target MCU: APM32E103VET6 / STM32F103VET6 (Cortex-M3 @ 96 MHz)
 * Board:      SIMATIC S7-200 CPU 226 Clone (6ES7 216-2BD23-0XB0)
 */

#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * Hardware Register Definitions (STM32F10x / APM32E103)
 * ========================================================================== */
#define REG16(addr) (*(volatile uint16_t *)(uintptr_t)(addr))
#define REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))

/* EXTI Controller (0x40010400) */
#define EXTI_BASE           0x40010400UL
#define EXTI_IMR            REG32(0x40010400UL)
#define EXTI_EMR            REG32(0x40010404UL)
#define EXTI_RTSR           REG32(0x40010408UL)
#define EXTI_FTSR           REG32(0x4001040CUL)
#define EXTI_SWIER          REG32(0x40010410UL)
#define EXTI_PR             REG32(0x40010414UL)

/* GPIO Ports */
#define GPIOB_BASE          0x40010C00UL
#define GPIOB_IDR           REG32(0x40010C08UL)

#define GPIOC_BASE          0x40011000UL
#define GPIOC_CRH           REG32(0x40011004UL)
#define GPIOC_IDR           REG32(0x40011008UL)

#define GPIOD_BASE          0x40011400UL
#define GPIOD_IDR           REG32(0x40011408UL)

#define GPIOE_BASE          0x40011800UL
#define GPIOE_IDR           REG32(0x40011808UL)

/* Cortex-M3 Bit-Band Aliases for GPIOC_ODR (0x4001100C) pins PC8..PC11
 * Expansion Module Synchronous Serial Bus:
 *   PC8  (0x422201A0): EXP_SCK   (Clock Output)
 *   PC9  (0x422201A4): EXP_CTRL1 (Latch / Strobe 1)
 *   PC10 (0x422201A8): EXP_MOSI  (Data I/O - bidirectional on PC10)
 *   PC11 (0x422201AC): EXP_CS    (Chip Select / Frame Sync)
 */
#define BITBAND_GPIOC_ODR_PC8   REG32(0x422201A0UL)
#define BITBAND_GPIOC_ODR_PC9   REG32(0x422201A4UL)
#define BITBAND_GPIOC_ODR_PC10  REG32(0x422201A8UL)
#define BITBAND_GPIOC_ODR_PC11  REG32(0x422201ACUL)

/* Timers used by HSC1 and PTO0/PTO1 */
#define TIM2_CR1            REG16(0x40000000UL)
#define TIM2_CNT            REG16(0x40000024UL)
#define TIM3_CR1            REG16(0x40000400UL)
#define TIM3_CNT            REG16(0x40000424UL)
#define TIM4_CNT            REG16(0x40000824UL)
#define TIM5_CR1            REG16(0x40000C00UL)
#define TIM5_CNT            REG16(0x40000C24UL)

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

/* Flash Controller */
#define FLASH_SR            REG32(0x4002200CUL)

/* ==========================================================================
 * PLC Firmware RAM Globals (0x20000000 .. 0x2000BFFF)
 * ========================================================================== */
#define RAM8(addr)          (*(volatile uint8_t  *)(uintptr_t)(addr))
#define RAM16(addr)         (*(volatile uint16_t *)(uintptr_t)(addr))
#define RAM32(addr)         (*(volatile uint32_t *)(uintptr_t)(addr))
#define RAM_S32(addr)       (*(volatile int32_t  *)(uintptr_t)(addr))
#define RAM_PTR8(addr)      ((volatile uint8_t   *)(uintptr_t)(addr))

/* High-Speed Counter Current Values (HC0..HC5 in PLC memory at 0x20006150) */
#define HC0_CUR_VAL         RAM_S32(0x20006150UL) /* HC0 (I0.0 / I0.1 / I0.2) */
#define HC2_CUR_VAL         RAM_S32(0x20006154UL) /* HC2 (I0.6 / I0.7 / I1.0 / I1.1) */
#define HC4_CUR_VAL         RAM_S32(0x20006158UL) /* HC4 (I1.2 / I1.3 / I1.4 / I1.5) */
#define HC5_CUR_VAL         RAM_S32(0x2000615CUL) /* HC5 (I0.1 single-phase) */
#define HC1_CUR_VAL         RAM_S32(0x20006160UL) /* HC1 (TIM3/TIM4 hardware counter) */

/* High-Speed Counter Preset Values (SMD42, SMD52, SMD62, SMD142, SMD152) */
#define HC0_PRESET_VAL      RAM_S32(0x200061ACUL)
#define HC2_PRESET_VAL      RAM_S32(0x200061B0UL)
#define HC4_PRESET_VAL      RAM_S32(0x200061B4UL)
#define HC5_PRESET_VAL      RAM_S32(0x200061B8UL)

/* I/O & HSC Interrupt Event Circular Queue (16 entries) */
#define IO_INT_QUEUE        RAM_PTR8(0x20000B3EUL)
#define IO_INT_WRITE_IDX    RAM8(0x200003E3UL)
#define IO_INT_COUNT        RAM8(0x200003E9UL)
#define IO_INT_LOCK_FLAG    RAM8(0x200003DCUL)

/* External declarations for helper functions in other translation units */
extern uint16_t FUN_08014520(volatile void *gpio_base);
extern void     FUN_08016e8c(uint32_t pulse_cnt, uint32_t start_period, uint32_t end_period,
                             uint32_t delta_step, uint32_t step_dir);
extern void     FUN_0801704e(uint32_t pulse_cnt, uint32_t start_period, uint32_t end_period,
                             uint32_t delta_step);
extern uint64_t FUN_0800016c(uint32_t num_lo, uint32_t num_hi, uint32_t den_lo, uint32_t den_hi);

/* Forward declarations within this file */
void     FUN_080129f0(uint32_t exti_line);
uint32_t FUN_080129fc(uint32_t exti_line);

/**
 * @brief Enqueue an I/O edge or HSC/PTO interrupt event ID into the 16-entry
 *        high-priority I/O interrupt circular queue (0x20000B3E).
 */
static inline void plc_io_int_enqueue(uint8_t event_id)
{
    uint32_t idx = IO_INT_WRITE_IDX;
    IO_INT_QUEUE[idx] = event_id;
    idx = (idx + 1U) & 0xFFU;
    IO_INT_WRITE_IDX = (uint8_t)idx;
    IO_INT_COUNT = (uint8_t)(IO_INT_COUNT + 1U);
    if (idx > 0x0FU) {
        IO_INT_WRITE_IDX = 0;
    }
    IO_INT_LOCK_FLAG = 0;
}

/* ==========================================================================
 * Function: FUN_0800fb4e (EXTI0_IRQHandler) @ 0x0800fb4e (230 bytes)
 *
 * Handles EXTI Line 0 (PE0 = I0.2 / I0.3):
 *   - Mode 1/2/3: Digital input rising (event 5) / falling (event 6) edge ISR
 *   - Mode 10:    HSC0 external reset input (resets HC0 = 0, enqueues event 0x1D)
 * ========================================================================== */
void FUN_0800fb4e(void)
{
    if (FUN_080129fc(1U) == 0) {
        return;
    }
    FUN_080129f0(1U);

    uint8_t mode = RAM8(0x20000B80UL);
    if (mode != 1U && mode != 2U && mode != 3U) {
        if (mode != 10U) {
            return;
        }
        /* HSC0 External Reset enabled check (SMB37 bit 7) */
        if ((RAM8(0x20000338UL) & 0x80U) == 0) {
            return;
        }
        HC0_CUR_VAL = 0;
        if (RAM8(0x2000034AUL) != 1U) {
            HC0_CUR_VAL = 0;
            return;
        }
        plc_io_int_enqueue(0x1DU); /* Event 29: HSC0 External Reset */
        return;
    }

    uint16_t port_val = FUN_08014520((volatile void *)GPIOE_BASE);
    RAM16(0x20000050UL) = port_val & 0x0001U;
    if (RAM16(0x20000050UL) == 0) {
        if ((mode & 0x02U) != 0) {
            plc_io_int_enqueue(6U); /* Event 6: I0.3 Falling Edge */
            return;
        }
    } else if ((mode & 0x01U) != 0) {
        plc_io_int_enqueue(5U); /* Event 5: I0.3 Rising Edge */
        return;
    }

    if (IO_INT_WRITE_IDX > 0x0FU) {
        IO_INT_WRITE_IDX = 0;
    }
    IO_INT_LOCK_FLAG = 0;
}

/* ==========================================================================
 * Function: FUN_0800fc34 (EXTI15_10_IRQHandler) @ 0x0800fc34 (2536 bytes)
 *
 * Multiplexed handler for EXTI Lines 10..15:
 *   - EXTI11 (PE11, 0x0800): HSC4 Phase B / Quadrature / Up-Down counting
 *     (HC4 @ 0x20006158, PV @ 0x200061B4, events 0x11 [HC4=PV] & 0x12 [Dir])
 *   - EXTI12 (PD12, 0x1000): Debounced I0.0 edge interrupt (events 1/2) OR
 *     HSC0 Phase A clock input (HC0 @ 0x20006150, PV @ 0x200061AC,
 *     events 0x0D [HC0=PV] & 0x1C [HSC0 Dir Change])
 *   - EXTI13 (PD13, 0x2000): Debounced I0.1 edge interrupt (events 3/4),
 *     HSC0 Phase B input, OR HSC5 single-phase counter (HC5 @ 0x2000615C,
 *     event 0x21 [HC5=PV])
 *   - EXTI10 (PE10, 0x0400): HSC2 external reset (zeros HC2, event 0x10)
 * ========================================================================== */
void FUN_0800fc34(void)
{
    uint32_t gpioe_idr;
    uint32_t gpiod_idr;

    /* ----------------------------------------------------------------------
     * 1. EXTI11 (0x0800, PE11): HSC4 Phase B / Down Clock
     * ---------------------------------------------------------------------- */
    if ((EXTI_PR & 0x0800UL) != 0) {
        EXTI_PR = 0x0800UL;
        int32_t hc4_dec = HC4_CUR_VAL - 1;
        uint32_t hsc4_ctrl = RAM8(0x2000033AUL);

        switch (RAM8(0x20000B8BUL)) {
        case 0x1AU:
        hsc4_cnt_down_simple:
            if (RAM8(0x200003C6UL) == 1U) {
                RAM8(0x200003CCUL) = 1U;
            }
            RAM8(0x200003C6UL) = 2U;
            HC4_CUR_VAL = hc4_dec;
            break;

        case 0x1BU:
            if (RAM8(0x200003C6UL) == 1U) {
                RAM8(0x200003CCUL) = 1U;
            }
            RAM8(0x200003C6UL) = 2U;
            gpioe_idr = GPIOE_IDR;
            if (((gpioe_idr & 0x1000UL) == 0) || ((RAM8(0x2000033AUL) & 1U) != 0)) {
                gpioe_idr = GPIOE_IDR;
                HC4_CUR_VAL = hc4_dec;
                if (((gpioe_idr & 0x1000UL) != 0) || ((RAM8(0x2000033AUL) & 1U) == 0)) {
                    break;
                }
            }
            HC4_CUR_VAL = 0;
            break;

        case 0x1DU:
            gpioe_idr = GPIOE_IDR;
            if (((gpioe_idr & 0x0A00UL) == 0x0800UL) ||
                ((GPIOE_IDR & 0x0A00UL) == 0x0200UL)) {
                goto hsc4_cnt_down_simple;
            }
            if (((GPIOE_IDR & 0x0A00UL) == 0UL) ||
                ((~GPIOE_IDR & 0x0A00UL) == 0UL)) {
                if (RAM8(0x200003C6UL) == 2U) {
                    RAM8(0x200003CCUL) = 1U;
                }
                RAM8(0x200003C6UL) = 1U;
                HC4_CUR_VAL = HC4_CUR_VAL + 1;
            }
            break;

        case 0x1FU:
        hsc4_quad_with_reset:
            if (((GPIOE_IDR & 0x0A00UL) == 0x0800UL) ||
                ((GPIOE_IDR & 0x0A00UL) == 0x0200UL)) {
                if (RAM8(0x200003C6UL) == 1U) {
                    RAM8(0x200003CCUL) = 1U;
                }
                RAM8(0x200003C6UL) = 2U;
                HC4_CUR_VAL = hc4_dec;
            } else if (((GPIOE_IDR & 0x0A00UL) == 0UL) ||
                       ((~GPIOE_IDR & 0x0A00UL) == 0UL)) {
                if (RAM8(0x200003C6UL) == 2U) {
                    RAM8(0x200003CCUL) = 1U;
                }
                RAM8(0x200003C6UL) = 1U;
                HC4_CUR_VAL = HC4_CUR_VAL + 1;
            }
            if (((GPIOE_IDR & 0x1000UL) == 0) || ((RAM8(0x2000033AUL) & 1U) != 0)) {
                if (((GPIOE_IDR & 0x1000UL) != 0) || ((RAM8(0x2000033AUL) & 1U) == 0)) {
                    break;
                }
            }
            HC4_CUR_VAL = 0;
            break;

        case 0x23U:
            if ((((GPIOE_IDR & 0x2000UL) != 0) && ((hsc4_ctrl & 2U) == 0)) ||
                (((GPIOE_IDR & 0x2000UL) == 0) && ((hsc4_ctrl & 2U) != 0))) {
                if (RAM8(0x200003C6UL) == 1U) {
                    RAM8(0x200003CCUL) = 1U;
                }
                RAM8(0x200003C6UL) = 2U;
                if (((GPIOE_IDR & 0x1000UL) == 0) || ((RAM8(0x2000033AUL) & 1U) != 0)) {
                    gpioe_idr = GPIOE_IDR;
                    HC4_CUR_VAL = hc4_dec;
                    if (((gpioe_idr & 0x1000UL) != 0) || ((RAM8(0x2000033AUL) & 1U) == 0)) {
                        break;
                    }
                }
                HC4_CUR_VAL = 0;
            }
            break;

        case 0x25U:
            if ((((GPIOE_IDR & 0x2000UL) != 0) && ((hsc4_ctrl & 2U) == 0)) ||
                (((GPIOE_IDR & 0x2000UL) == 0) && ((hsc4_ctrl & 2U) != 0))) {
                goto hsc4_quad_with_reset;
            }
            break;

        default:
            break;
        }

        if (RAM8(0x20000346UL) == 1U && HC4_CUR_VAL == HC4_PRESET_VAL) {
            plc_io_int_enqueue(0x11U); /* Event 17: HSC4 CV == PV */
        }
        if (RAM8(0x200003D8UL) == 1U && RAM8(0x200003CCUL) == 1U) {
            RAM8(0x200003CCUL) = 0;
            plc_io_int_enqueue(0x12U); /* Event 18: HSC4 Direction Changed */
        }
    }

    /* ----------------------------------------------------------------------
     * 2. EXTI12 (0x1000, PD12 = I0.0): Debounced Input / HSC0 Phase A
     * ---------------------------------------------------------------------- */
    if ((EXTI_PR & 0x1000UL) != 0) {
        uint16_t total_samples = 0;
        uint16_t high_cnt = 0;
        uint16_t low_cnt = 0;

        while (total_samples < 16U && high_cnt < 8U && low_cnt < 8U) {
            if ((GPIOD_IDR & 0x1000UL) != 0) {
                high_cnt = 0;
                low_cnt++;
            } else {
                high_cnt++;
                low_cnt = 0;
            }
            total_samples++;
        }

        if (low_cnt == 8U) {
            int32_t hc0_inc = HC0_CUR_VAL + 1;
            int32_t hc0_dec = HC0_CUR_VAL - 1;
            int32_t hc0_next = HC0_CUR_VAL;
            uint8_t mode0 = RAM8(0x20000B7EUL);

            if (mode0 == 0x18U) {
                if ((GPIOD_IDR & 0x2000UL) == 0) {
                    goto hsc0_cnt_down;
                }
                goto hsc0_cnt_up;
            } else if (mode0 < 0x19U) {
                hc0_next = hc0_inc;
                if (mode0 != 0x14U) {
                    if (mode0 < 0x15U) {
                        if (mode0 == 1U || mode0 == 2U || mode0 == 3U) {
                            uint16_t in_val = FUN_08014520((volatile void *)GPIOD_BASE);
                            RAM16(0x20000054UL) = in_val & 0x1000U;
                            if (RAM16(0x20000054UL) == 0) {
                                if ((mode0 & 2U) != 0) {
                                    plc_io_int_enqueue(2U); /* Event 2: I0.0 Falling Edge */
                                }
                            } else if ((mode0 & 1U) != 0) {
                                plc_io_int_enqueue(1U); /* Event 1: I0.0 Rising Edge */
                            }
                            if (IO_INT_WRITE_IDX > 0x0FU) {
                                IO_INT_WRITE_IDX = 0;
                            }
                            IO_INT_LOCK_FLAG = 0;
                            hc0_next = HC0_CUR_VAL;
                        } else {
                            hc0_next = HC0_CUR_VAL;
                        }
                    } else {
                        hc0_next = hc0_dec;
                        if (mode0 != 0x15U) {
                            if (mode0 == 0x16U) {
                                goto hsc0_apply_with_reset_check;
                            } else if (mode0 == 0x17U) {
                                hc0_inc = hc0_dec;
                                goto hsc0_apply_with_reset_check;
                            } else {
                                hc0_next = HC0_CUR_VAL;
                            }
                        }
                    }
                }
            } else {
                switch (mode0) {
                case 0x19U:
                    if ((GPIOD_IDR & 0x2000UL) != 0) {
                        goto hsc0_cnt_up_with_reset;
                    }
                hsc0_cnt_down_with_reset:
                    if (RAM8(0x200003C4UL) == 1U) {
                        RAM8(0x200003CAUL) = 1U;
                    }
                    RAM8(0x200003C4UL) = 2U;
                    hc0_inc = hc0_dec;
                    goto hsc0_apply_with_reset_check;

                case 0x1AU:
                hsc0_cnt_up:
                    if (RAM8(0x200003C4UL) == 2U) {
                        RAM8(0x200003CAUL) = 1U;
                    }
                    RAM8(0x200003C4UL) = 1U;
                    hc0_next = hc0_inc;
                    break;

                case 0x1BU:
                    goto hsc0_cnt_up_with_reset;

                case 0x1CU:
                    if (((GPIOD_IDR & 0x3FFFUL) >> 12) == 1U) {
                        goto hsc0_cnt_up;
                    }
                    if ((GPIOD_IDR & 0x3000UL) == 0UL) {
                    hsc0_cnt_down:
                        if (RAM8(0x200003C4UL) == 1U) {
                            RAM8(0x200003CAUL) = 1U;
                        }
                        RAM8(0x200003C4UL) = 2U;
                        hc0_next = hc0_dec;
                    }
                    break;

                case 0x1DU:
                    gpiod_idr = (GPIOD_IDR & 0x3FFFUL) >> 12;
                    if (gpiod_idr == 1U || gpiod_idr == 2U) {
                        goto hsc0_cnt_up;
                    }
                    if ((GPIOD_IDR & 0x3000UL) == 0UL || (((GPIOD_IDR & 0x3FFFUL) >> 12) == 3U)) {
                        goto hsc0_cnt_down;
                    }
                    break;

                case 0x1EU:
                    if (((GPIOD_IDR & 0x3FFFUL) >> 12) != 1U) {
                        hc0_inc = HC0_CUR_VAL;
                        if ((GPIOD_IDR & 0x3000UL) == 0UL) {
                            goto hsc0_cnt_down_with_reset;
                        }
                        goto hsc0_apply_with_reset_check;
                    }
                    goto hsc0_cnt_up_with_reset;

                case 0x1FU:
                    gpiod_idr = (GPIOD_IDR & 0x3FFFUL) >> 12;
                    if (gpiod_idr != 1U && gpiod_idr != 2U) {
                        if ((GPIOD_IDR & 0x3000UL) != 0UL && (((GPIOD_IDR & 0x3FFFUL) >> 12) != 3U)) {
                            hc0_inc = HC0_CUR_VAL;
                            goto hsc0_apply_with_reset_check;
                        }
                        goto hsc0_cnt_down_with_reset;
                    }
                hsc0_cnt_up_with_reset:
                    if (RAM8(0x200003C4UL) == 2U) {
                        RAM8(0x200003CAUL) = 1U;
                    }
                    RAM8(0x200003C4UL) = 1U;
                hsc0_apply_with_reset_check:
                    HC0_CUR_VAL = hc0_inc;
                    hc0_next = HC0_CUR_VAL;
                    if ((((GPIOE_IDR & 1UL) != 0) && ((RAM8(0x20000338UL) & 1U) == 0)) ||
                        (((GPIOE_IDR & 1UL) == 0) && ((RAM8(0x20000338UL) & 1U) != 0))) {
                        HC0_CUR_VAL = 0;
                        hc0_next = 0;
                    }
                    break;

                default:
                    break;
                }
            }

            HC0_CUR_VAL = hc0_next;
            if (RAM8(0x20000344UL) == 1U && (uint8_t)(mode0 - 0x14U) < 0x0CU &&
                HC0_CUR_VAL == HC0_PRESET_VAL) {
                plc_io_int_enqueue(0x0DU); /* Event 13: HSC0 CV == PV */
            }
            if (RAM8(0x200003D6UL) == 1U && (uint8_t)(mode0 - 0x14U) < 0x0CU &&
                RAM8(0x200003CAUL) == 1U) {
                RAM8(0x200003CAUL) = 0;
                plc_io_int_enqueue(0x1CU); /* Event 28: HSC0 Direction Changed */
            }
        }
        EXTI_PR = 0x1000UL;
    }

    /* ----------------------------------------------------------------------
     * 3. EXTI13 (0x2000, PD13 = I0.1): Debounced Input / HSC0 Phase B / HSC5
     * ---------------------------------------------------------------------- */
    if ((EXTI_PR & 0x2000UL) != 0) {
        uint16_t total_samples = 0;
        uint16_t high_cnt = 0;
        uint16_t low_cnt = 0;

        while (total_samples < 16U && high_cnt < 8U && low_cnt < 8U) {
            if ((GPIOD_IDR & 0x2000UL) != 0) {
                high_cnt = 0;
                low_cnt++;
            } else {
                high_cnt++;
                low_cnt = 0;
            }
            total_samples++;
        }

        if (low_cnt == 8U) {
            uint8_t mode1 = RAM8(0x20000B7FUL);
            if (mode1 == 0x1BU) {
                HC0_CUR_VAL = HC0_CUR_VAL - 1;
                if (RAM8(0x200003C4UL) == 1U) {
                    RAM8(0x200003CAUL) = 1U;
                }
                RAM8(0x200003C4UL) = 2U;
                if ((((GPIOE_IDR & 1UL) != 0) && ((RAM8(0x20000338UL) & 1U) == 0)) ||
                    (((GPIOE_IDR & 1UL) == 0) && ((RAM8(0x20000338UL) & 1U) != 0))) {
                    HC0_CUR_VAL = 0;
                }
            } else if (mode1 < 0x1CU) {
                if (mode1 == 1U || mode1 == 2U || mode1 == 3U) {
                    uint16_t in_val = FUN_08014520((volatile void *)GPIOD_BASE);
                    RAM16(0x20000054UL) = in_val & 0x2000U;
                    if (RAM16(0x20000054UL) == 0) {
                        if ((mode1 & 2U) != 0) {
                            plc_io_int_enqueue(4U); /* Event 4: I0.1 Falling Edge */
                        }
                    } else if ((mode1 & 1U) != 0) {
                        plc_io_int_enqueue(3U); /* Event 3: I0.1 Rising Edge */
                    }
                    if (IO_INT_WRITE_IDX > 0x0FU) {
                        IO_INT_WRITE_IDX = 0;
                    }
                    IO_INT_LOCK_FLAG = 0;
                } else if (mode1 == 0x1AU) {
                hsc0_phb_down:
                    HC0_CUR_VAL = HC0_CUR_VAL - 1;
                    if (RAM8(0x200003C4UL) == 1U) {
                        RAM8(0x200003CAUL) = 1U;
                    }
                    RAM8(0x200003C4UL) = 2U;
                }
            } else if (mode1 == 0x1DU) {
                gpiod_idr = (GPIOD_IDR & 0x3FFFUL) >> 12;
                if (gpiod_idr == 1U || gpiod_idr == 2U) {
                    goto hsc0_phb_down;
                }
                if ((GPIOD_IDR & 0x3000UL) == 0UL || (((GPIOD_IDR & 0x3FFFUL) >> 12) == 3U)) {
                    HC0_CUR_VAL = HC0_CUR_VAL + 1;
                    if (RAM8(0x200003C4UL) == 2U) {
                        RAM8(0x200003CAUL) = 1U;
                    }
                    RAM8(0x200003C4UL) = 1U;
                }
            } else if (mode1 == 0x1FU) {
                gpiod_idr = (GPIOD_IDR & 0x3FFFUL) >> 12;
                if (gpiod_idr == 1U || gpiod_idr == 2U) {
                    HC0_CUR_VAL = HC0_CUR_VAL - 1;
                    if (RAM8(0x200003C4UL) == 1U) {
                        RAM8(0x200003CAUL) = 1U;
                    }
                    RAM8(0x200003C4UL) = 2U;
                } else if ((GPIOD_IDR & 0x3000UL) == 0UL || (((GPIOD_IDR & 0x3FFFUL) >> 12) == 3U)) {
                    HC0_CUR_VAL = HC0_CUR_VAL + 1;
                    if (RAM8(0x200003C4UL) == 2U) {
                        RAM8(0x200003CAUL) = 1U;
                    }
                    RAM8(0x200003C4UL) = 1U;
                }
                if ((((GPIOE_IDR & 1UL) != 0) && ((RAM8(0x20000338UL) & 1U) == 0)) ||
                    (((GPIOE_IDR & 1UL) == 0) && ((RAM8(0x20000338UL) & 1U) != 0))) {
                    HC0_CUR_VAL = 0;
                }
            } else if (mode1 == 0x3CU) {
                HC5_CUR_VAL = HC5_CUR_VAL + 1;
            } else if (mode1 == 0x3DU) {
                HC5_CUR_VAL = HC5_CUR_VAL - 1;
            }

            if (RAM8(0x20000344UL) == 1U && (uint8_t)(mode1 - 0x14U) < 0x0CU &&
                HC0_CUR_VAL == HC0_PRESET_VAL) {
                plc_io_int_enqueue(0x0DU); /* Event 13: HSC0 CV == PV */
            }
            if (RAM8(0x200003D6UL) == 1U && (uint8_t)(mode1 - 0x14U) < 0x0CU &&
                RAM8(0x200003CAUL) == 1U) {
                RAM8(0x200003CAUL) = 0;
                plc_io_int_enqueue(0x1CU); /* Event 28: HSC0 Direction Changed */
            }
            if (RAM8(0x20000347UL) == 1U && (uint8_t)(mode1 - 0x3CU) < 2U &&
                HC5_CUR_VAL == HC5_PRESET_VAL) {
                plc_io_int_enqueue(0x21U); /* Event 33: HSC5 CV == PV */
            }
        }
        EXTI_PR = 0x2000UL;
    }

    /* ----------------------------------------------------------------------
     * 4. EXTI10 (0x0400, PE10): HSC2 External Reset Input
     * ---------------------------------------------------------------------- */
    if ((EXTI_PR & 0x0400UL) != 0) {
        EXTI_PR = 0x0400UL;
        if (RAM8(0x20000B89UL) == 10U) {
            if ((((GPIOE_IDR & 0x0100UL) != 0) && ((RAM8(0x20000339UL) & 1U) == 0)) ||
                (((GPIOE_IDR & 0x0100UL) == 0) && ((RAM8(0x20000339UL) & 1U) != 0))) {
                HC2_CUR_VAL = 0;
                if (RAM8(0x2000034BUL) == 1U) {
                    plc_io_int_enqueue(0x10U); /* Event 16: HSC2 External Reset */
                }
            }
        }
    }
}

/* ==========================================================================
 * Function: FUN_08010620 (EXTI1_IRQHandler) @ 0x08010620 (688 bytes)
 *
 * Handles EXTI Line 1 (PE1 hardware HSC1 external reset AND software-triggered
 * deferred HSC/PTO interrupt multiplexer via EXTI_SWIER |= 2):
 *   - HSC1 External Reset -> Event 0x20 (32)
 *   - HSC1 CV == PV       -> Event 0x1E (30)
 *   - HSC1 Dir Changed    -> Event 0x1F (31)
 *   - HSC3 CV == PV       -> Event 0x22 (34)
 *   - HSC4 CV == PV (SW)  -> Event 0x13 (19)
 *   - PTO0 Done           -> Event 0x14 (20)
 *   - PTO1 Done           -> Event 0x15 (21)
 *   - HSC0 CV == PV (SW)  -> Event 0x0D (13)
 *   - HSC5 CV == PV (SW)  -> Event 0x21 (33)
 * ========================================================================== */
void FUN_08010620(void)
{
    if (FUN_080129fc(2U) == 0) {
        return;
    }

    if ((RAM8(0x2000033CUL) & 0x80U) != 0) {
        uint8_t hsc1_mode = RAM8(0x20000342UL);
        if ((hsc1_mode == 2U || hsc1_mode == 5U || hsc1_mode == 8U || hsc1_mode == 11U) &&
            ((((GPIOE_IDR & 0x0002UL) != 0) && ((RAM8(0x2000033CUL) & 1U) == 0)) ||
             (((GPIOE_IDR & 0x0002UL) == 0) && ((RAM8(0x2000033CUL) & 1U) != 0)))) {
            HC1_CUR_VAL = 0;
            TIM3_CNT = 0;
            RAM32(0x20000B2CUL) = 0;
            if (hsc1_mode == 8U) {
                TIM4_CNT = 0;
                RAM32(0x20000B30UL) = 0;
            }
            if (RAM8(0x2000034EUL) == 1U) {
                plc_io_int_enqueue(0x20U); /* Event 32: HSC1 External Reset */
            }
            RAM32(0x20000068UL) = 0;
            RAM32(0x20000074UL) = 0;
            RAM32(0x20000060UL) = 0;
            RAM32(0x2000007CUL) = 0;
            RAM8(0x2000002AUL)  = 0;
            RAM8(0x200003C8UL)  = 0;
            RAM8(0x200003CEUL)  = 0;
        } else {
            if (RAM8(0x2000002AUL) == 1U) {
                RAM8(0x2000002AUL) = 0;
                plc_io_int_enqueue(0x1EU); /* Event 30: HSC1 CV == PV */
            }
            if (RAM8(0x200003CEUL) == 1U) {
                RAM8(0x200003CEUL) = 0;
                plc_io_int_enqueue(0x1FU); /* Event 31: HSC1 Direction Changed */
            }
        }
        RAM8(0x2000002EUL) = 0;
    }

    if (RAM8(0x2000002BUL) == 1U) {
        RAM8(0x2000002BUL) = 0;
        plc_io_int_enqueue(0x22U); /* Event 34: HSC3 CV == PV */
    }
    if (RAM8(0x20000352UL) == 1U) {
        RAM8(0x20000352UL) = 0;
        plc_io_int_enqueue(0x13U); /* Event 19: HSC4 CV == PV */
    }
    if (RAM8(0x20000031UL) == 1U) {
        RAM8(0x20000031UL) = 0;
        plc_io_int_enqueue(0x14U); /* Event 20: PTO0 Complete */
    }
    if (RAM8(0x20000032UL) == 1U) {
        RAM8(0x20000032UL) = 0;
        plc_io_int_enqueue(0x15U); /* Event 21: PTO1 Complete */
    }
    if (RAM8(0x2000018BUL) == 1U) {
        RAM8(0x2000018BUL) = 0;
        plc_io_int_enqueue(0x0DU); /* Event 13: HSC0 CV == PV */
    }
    if (RAM8(0x2000018CUL) == 1U) {
        RAM8(0x2000018CUL) = 0;
        plc_io_int_enqueue(0x21U); /* Event 33: HSC5 CV == PV */
    }

    FUN_080129f0(2U);
}

/* ==========================================================================
 * Function: FUN_080108d0 (EXTI2_IRQHandler) @ 0x080108d0 (5650 bytes)
 *
 * Software-triggered interrupt handler (EXTI_SWIER |= 4) responsible for
 * advancing PTO0 (Q0.0, TIM1 + TIM2) and PTO1 (Q0.1, TIM8 + TIM5) motion
 * profile / envelope segments:
 *   - Supports both EM253 / Position Control Wizard 10-byte segment descriptors
 *     (when 0x2000018F / 0x20000196 == 1) and standard S7-200 PLS envelope
 *     8-byte segment descriptors in V-memory (0x2000108C + offset).
 *   - Computes initial/final cycle periods and 64-bit frequency ramp step
 *     increments using FUN_0800016c (__aeabi_uldivmod), then programs
 *     TIM1/TIM2 via FUN_08016e8c (PTO0) or TIM8/TIM5 via FUN_0801704e (PTO1).
 * ========================================================================== */
uint64_t FUN_080108d0(uint32_t param_1, uint32_t param_2, uint32_t param_3, uint32_t param_4)
{
    if (FUN_080129fc(4U) == 0) {
        return ((uint64_t)param_2 << 32) | param_1;
    }
    FUN_080129f0(4U);

    uint8_t smb66  = RAM8(0x20005F6AUL);
    uint8_t sm_d2  = RAM8(0x20005FD2UL);
    uint8_t sm_d3  = RAM8(0x20005FD3UL);

    /* ----------------------------------------------------------------------
     * PTO0 (Q0.0) Next Envelope Segment Advance (triggered by 0x20000404 == 1)
     * ---------------------------------------------------------------------- */
    if (RAM8(0x20000404UL) == 1U) {
        RAM8(0x20000404UL) = 0;
        RAM8(0x20000029UL) = RAM8(0x20005F6BUL) & 0x20U;

        if (RAM8(0x2000018FUL) == 1U) {
            /* Extended 10-byte Position Control profile segment mode */
            uint8_t next_seg = (uint8_t)(RAM8(0x20005FCEUL) + 1U);
            param_1 = next_seg;
            uint32_t total_segs = RAM16(0x20000420UL);
            if (param_1 < total_segs) {
                uint32_t seg_off = (uint32_t)RAM16(0x20000424UL) + param_1 * 10U;
                RAM16(0x20000422UL) = RAM_PTR8(0x2000108EUL)[seg_off];
                uint8_t seg_cmd = RAM_PTR8(0x2000108FUL)[seg_off];
                RAM8(0x200003ECUL) = seg_cmd;

                if (seg_cmd != 0x10U || (param_1 + 1U) < total_segs) {
                    RAM8(0x2000018EUL) = 10U;
                    if (seg_cmd == 0x04U || seg_cmd == 0x18U || seg_cmd == 0x10U) {
                        uint32_t raw_freq =
                            ((uint32_t)RAM_PTR8(0x20001090UL)[seg_off] << 24) |
                            ((uint32_t)RAM_PTR8(0x20001091UL)[seg_off] << 16) |
                            ((uint32_t)RAM_PTR8(0x20001092UL)[seg_off] << 8)  |
                            ((uint32_t)RAM_PTR8(0x20001093UL)[seg_off]);
                        uint32_t period = ((int32_t)raw_freq >> 11) * 100U;
                        if (period < RAM32(0x2000041CUL)) {
                            period = RAM32(0x2000041CUL);
                        }
                        if (period > RAM32(0x20000418UL)) {
                            period = RAM32(0x20000400UL);
                        }
                        RAM32(0x2000040CUL) = period;
                        RAM16(0x20000044UL) = 0x0BU;
                        RAM32(0x200003F0UL) = period;
                        RAM32(0x200003F4UL) = period;
                    } else if (seg_cmd == 0x08U || seg_cmd == 0x00U) {
                        uint32_t raw_freq =
                            ((uint32_t)RAM_PTR8(0x2000109AUL)[seg_off] << 24) |
                            ((uint32_t)RAM_PTR8(0x2000109BUL)[seg_off] << 16) |
                            ((uint32_t)RAM_PTR8(0x2000109CUL)[seg_off] << 8)  |
                            ((uint32_t)RAM_PTR8(0x2000109DUL)[seg_off]);
                        uint32_t period = ((int32_t)raw_freq >> 11) * 100U;
                        if (period < RAM32(0x2000041CUL)) {
                            period = RAM32(0x2000041CUL);
                        }
                        if (period > RAM32(0x20000418UL)) {
                            period = RAM32(0x20000400UL);
                        }
                        RAM32(0x2000040CUL) = period;
                        RAM16(0x20000044UL) = (seg_cmd == 0x08U) ? 0x0DU : 0x0FU;
                        RAM32(0x200003F0UL) = RAM32(0x20000400UL);
                        RAM32(0x200003F4UL) = period;
                    }

                    uint32_t pulses =
                        ((uint32_t)RAM_PTR8(0x20001094UL)[seg_off] << 24) |
                        ((uint32_t)RAM_PTR8(0x20001095UL)[seg_off] << 16) |
                        ((uint32_t)RAM_PTR8(0x20001096UL)[seg_off] << 8)  |
                        ((uint32_t)RAM_PTR8(0x20001097UL)[seg_off]);
                    RAM32(0x20000410UL) = pulses;

                    if (seg_cmd == 0x18U) {
                        if (pulses != 0) {
                            RAM8(0x20005FD2UL) &= 0xFEU;
                            RAM8(0x20005FD3UL) |= 0x80U;
                            RAM8(0x20005F6AUL) |= 0x80U;
                            goto pto0_abort_seg;
                        }
                        RAM32(0x20000410UL) = 180000UL;
                        RAM8(0x20005FD2UL) |= 0x04U;
                        RAM16(0x20000190UL) = 0;
                    }

                    if (RAM16(0x20000044UL) == 0x0FU && (param_1 + 1U) < total_segs) {
                        RAM8(0x20000028UL) = RAM_PTR8(0x20001099UL)[seg_off];
                        if (RAM8(0x20000028UL) == 0x10U) {
                            uint32_t extra_pulses =
                                ((uint32_t)RAM_PTR8(0x2000109EUL)[seg_off] << 24) |
                                ((uint32_t)RAM_PTR8(0x2000109FUL)[seg_off] << 16) |
                                ((uint32_t)RAM_PTR8(0x200010A0UL)[seg_off] << 8)  |
                                ((uint32_t)RAM_PTR8(0x200010A1UL)[seg_off]);
                            RAM32(0x20000410UL) += extra_pulses;
                        }
                    }

                    RAM8(0x20005FCEUL) = next_seg;
                    param_1 = RAM32(0x200003FCUL);
                    FUN_08016e8c(RAM32(0x20000410UL), RAM32(0x200003F0UL),
                                 RAM32(0x200003F4UL), RAM32(0x200003F8UL),
                                 RAM32(0x200003FCUL));
                    TIM2_CNT = 0;
                    TIM1_CNT = 0;
                    RAM8(0x200003EDUL) = (RAM32(0x20000410UL) == 1UL) ? 1U : 0U;
                    RAM8(0x20005F6AUL) &= 0x5FU;
                    if (RAM8(0x200003EDUL) != 0) {
                        TIM1_SR = 0xFFFEU;
                        TIM1_DIER |= 1U;
                    }
                    TIM2_CR1 |= 1U;
                    TIM1_CR1 |= 1U;
                    TIM1_BDTR |= 0x8000U;
                    RAM8(0x2000018EUL) = (uint8_t)RAM16(0x20000044UL);
                    goto pto0_reset_accumulators;
                }
            }
        pto0_profile_done:
            RAM8(0x20005FCEUL) = next_seg;
            RAM8(0x2000018EUL) = 0x10U;
            smb66 = RAM8(0x20005F6AUL) | 0x80U;
            sm_d2 = RAM8(0x20005FD2UL) & 0xFEU;
            sm_d3 = RAM8(0x20005FD3UL) | 0x80U;
            if (RAM8(0x2000002FUL) == 1U) {
                RAM8(0x20000031UL) = 1U;
                EXTI_SWIER |= 2UL;
            }
        } else {
            /* Standard S7-200 8-byte PLS Envelope Table mode */
            uint8_t next_seg = RAM8(0x20005FCEUL);
            if ((RAM8(0x20005F6BUL) & 0x20U) == 0) {
                goto pto0_profile_done;
            }
            next_seg = (uint8_t)(next_seg + 1U);
            param_1 = RAM16(0x20000414UL);
            if (RAM_PTR8(0x2000108CUL)[param_1] <= next_seg) {
                goto pto0_profile_done;
            }
            RAM8(0x2000018EUL) = 10U;
            RAM32(0x20000098UL) += RAM32(0x20000380UL);
            uint32_t seg_off = param_1 + (uint32_t)next_seg * 8U;
            uint32_t pulses =
                ((uint32_t)RAM_PTR8(0x20001091UL)[seg_off] << 24) |
                ((uint32_t)RAM_PTR8(0x20001092UL)[seg_off] << 16) |
                ((uint32_t)RAM_PTR8(0x20001093UL)[seg_off] << 8)  |
                ((uint32_t)RAM_PTR8(0x20001094UL)[seg_off]);
            RAM32(0x20000410UL) = pulses;
            param_4 = pulses;

            if (pulses != 0) {
                uint16_t init_cyc = ((uint16_t)RAM_PTR8(0x2000108DUL)[seg_off] << 8) |
                                    RAM_PTR8(0x2000108EUL)[seg_off];
                RAM16(0x20000044UL) = init_cyc;
                if (init_cyc > 1U) {
                    uint32_t timebase_num = (RAM8(0x200003EBUL) == 0) ? 100000UL : 100000000UL;
                    RAM32(0x200003F0UL) = timebase_num / init_cyc;
                    int16_t delta_cyc = (int16_t)(((uint16_t)RAM_PTR8(0x2000108FUL)[seg_off] << 8) |
                                                  RAM_PTR8(0x20001090UL)[seg_off]);
                    RAM16(0x20000046UL) = (uint16_t)delta_cyc;
                    RAM16(0x20000406UL) = init_cyc;

                    uint32_t end_cyc = init_cyc;
                    if (delta_cyc != 0) {
                        uint32_t total_delta = pulses * (uint32_t)(int32_t)delta_cyc;
                        RAM32(0x200000D8UL) = total_delta;
                        RAM32(0x200000DCUL) = 0;
                        RAM16(0x20000416UL) = (uint16_t)delta_cyc;
                        end_cyc = (uint32_t)init_cyc + total_delta;
                        if (end_cyc >= 0x10000UL) {
                            RAM8(0x20005F6AUL) |= 0x10U;
                            goto pto0_seg_err;
                        }
                        RAM16(0x20000044UL) = (uint16_t)end_cyc;
                    } else {
                        RAM16(0x20000416UL) = 0;
                        RAM32(0x200000D8UL) = 0;
                        RAM32(0x200000DCUL) = 0;
                    }

                    RAM32(0x200003F4UL) = timebase_num / (end_cyc & 0xFFFFU);
                    if (RAM32(0x200003F0UL) < 100UL) {
                        RAM32(0x200003F0UL) = 100UL;
                    } else if (RAM32(0x200003F0UL) > 20000000UL) {
                        RAM32(0x200003F0UL) = 20000000UL;
                    }
                    if (RAM32(0x200003F4UL) < 100UL) {
                        RAM32(0x200003F4UL) = 100UL;
                    } else if (RAM32(0x200003F4UL) > 20000000UL) {
                        RAM32(0x200003F4UL) = 20000000UL;
                    }

                    uint64_t sum_prod = (uint64_t)(RAM32(0x200003F0UL) + RAM32(0x200003F4UL)) *
                                        (uint64_t)pulses * 100000ULL;
                    RAM8(0x20005FCEUL) = next_seg;
                    param_2 = RAM32(0x200003F4UL);
                    param_3 = RAM32(0x200003F0UL);
                    uint64_t div1 = FUN_0800016c((uint32_t)sum_prod, (uint32_t)(sum_prod >> 32),
                                                 param_2, 0);
                    uint64_t div2 = FUN_0800016c((uint32_t)div1, (uint32_t)(div1 >> 32),
                                                 param_3, 0);
                    RAM32(0x200000DCUL) = (uint32_t)(div2 >> 33);
                    RAM32(0x200000D8UL) = (uint32_t)(div2 >> 1);
                    RAM32(0x200003FCUL) = 1UL;
                    param_1 = 1UL;
                    RAM32(0x200003F8UL) = RAM32(0x200000D8UL);

                    FUN_08016e8c(param_4, param_3, param_2, RAM32(0x200003F8UL), 1UL);
                    RAM8(0x2000018EUL) = (RAM16(0x20000046UL) != 0) ? 0x0EU : 0x0BU;

                pto0_reset_accumulators:
                    RAM32(0x20000400UL) = RAM32(0x200003F0UL);
                    RAM32(0x200000E0UL) = 0;
                    RAM32(0x200000E4UL) = 0;
                    RAM32(0x200000E8UL) = 0;
                    RAM32(0x200000ECUL) = 0;
                    RAM32(0x20000380UL) = 0;
                    RAM32(0x20000384UL) = 0;
                    RAM32(0x20000084UL) = 0;
                    RAM32(0x20000094UL) = RAM32(0x20000410UL);
                    TIM2_CNT = 0;
                    TIM1_CNT = 0;
                    RAM8(0x20000192UL) = 0;
                    RAM8(0x200003EDUL) = (RAM32(0x20000410UL) == 1UL) ? 1U : 0U;
                    RAM8(0x20005F6AUL) &= 0x5FU;
                    if (RAM8(0x200003EDUL) != 0) {
                        TIM1_SR = 0xFFFEU;
                        TIM1_DIER |= 1U;
                    }
                    TIM2_CR1 |= 1U;
                    TIM1_CR1 |= 1U;
                    TIM1_BDTR |= 0x8000U;
                    smb66 = RAM8(0x20005F6AUL);
                    sm_d2 = RAM8(0x20005FD2UL);
                    sm_d3 = RAM8(0x20005FD3UL);
                    goto pto0_update_status;
                }
            pto0_seg_err:
                RAM8(0x20005FD3UL) |= 0x80U;
                RAM8(0x20005FD2UL) &= 0xFEU;
                RAM8(0x20005F6AUL) |= 0x80U;
            pto0_abort_seg:
                RAM8(0x20005FCEUL) += 1U;
                RAM8(0x2000018EUL) = 0x10U;
                smb66 = RAM8(0x20005F6AUL);
                sm_d2 = RAM8(0x20005FD2UL);
                sm_d3 = RAM8(0x20005FD3UL);
                goto pto0_update_status;
            }
            RAM8(0x20000404UL) = 1U;
            RAM8(0x20005FCEUL) = next_seg;
            RAM8(0x20005FD3UL) |= 0x80U;
            RAM8(0x20005FD2UL) &= 0xFEU;
            RAM8(0x20005F6AUL) |= 0x80U;
            RAM8(0x2000018EUL) = 0x10U;
            EXTI_SWIER |= 4UL;
            smb66 = RAM8(0x20005F6AUL);
            sm_d2 = RAM8(0x20005FD2UL);
            sm_d3 = RAM8(0x20005FD3UL);
        }
    }

pto0_update_status:
    RAM8(0x20005FD3UL) = sm_d3;
    RAM8(0x20005FD2UL) = sm_d2;
    RAM8(0x20005F6AUL) = smb66;

    /* PTO0 Direct Reload (0x20000405 == 1) */
    if (RAM8(0x20000405UL) == 1U) {
        RAM8(0x20000405UL) = 0;
        uint32_t seg_off = (uint32_t)RAM16(0x20000424UL) + (uint32_t)RAM8(0x20005FCEUL) * 10U;
        if (RAM8(0x200003ECUL) == 0x04U) {
            uint32_t raw_freq =
                ((uint32_t)RAM_PTR8(0x20001090UL)[seg_off] << 24) |
                ((uint32_t)RAM_PTR8(0x20001091UL)[seg_off] << 16) |
                ((uint32_t)RAM_PTR8(0x20001092UL)[seg_off] << 8)  |
                ((uint32_t)RAM_PTR8(0x20001093UL)[seg_off]);
            uint32_t period = ((int32_t)raw_freq >> 11) * 100U;
            if (period < RAM32(0x2000041CUL)) period = RAM32(0x2000041CUL);
            if (period > RAM32(0x20000418UL)) period = RAM32(0x20000400UL);
            RAM32(0x2000040CUL) = period;
            RAM16(0x20000044UL) = 0x0BU;
            RAM32(0x200003F0UL) = period;
            RAM32(0x200003F4UL) = period;
        } else if (RAM8(0x200003ECUL) == 0x00U) {
            uint32_t raw_freq =
                ((uint32_t)RAM_PTR8(0x2000109AUL)[seg_off] << 24) |
                ((uint32_t)RAM_PTR8(0x2000109BUL)[seg_off] << 16) |
                ((uint32_t)RAM_PTR8(0x2000109CUL)[seg_off] << 8)  |
                ((uint32_t)RAM_PTR8(0x2000109DUL)[seg_off]);
            uint32_t period = ((int32_t)raw_freq >> 11) * 100U;
            if (period < RAM32(0x2000041CUL)) period = RAM32(0x2000041CUL);
            if (period > RAM32(0x20000418UL)) period = RAM32(0x20000400UL);
            RAM32(0x2000040CUL) = period;
            RAM32(0x200003F0UL) = RAM32(0x20000400UL);
            RAM32(0x200003F4UL) = period;
            RAM16(0x20000044UL) = 0x0FU;
        }
        uint32_t pulses =
            ((uint32_t)RAM_PTR8(0x20001094UL)[seg_off] << 24) |
            ((uint32_t)RAM_PTR8(0x20001095UL)[seg_off] << 16) |
            ((uint32_t)RAM_PTR8(0x20001096UL)[seg_off] << 8)  |
            ((uint32_t)RAM_PTR8(0x20001097UL)[seg_off]);
        RAM32(0x20000410UL) = pulses;
        if (RAM16(0x20000044UL) == 0x0FU && (uint32_t)(RAM8(0x20005FCEUL) + 1U) < RAM16(0x20000420UL)) {
            RAM8(0x20000028UL) = RAM_PTR8(0x20001099UL)[seg_off];
            if (RAM8(0x20000028UL) == 0x10U) {
                RAM32(0x20000410UL) +=
                    ((uint32_t)RAM_PTR8(0x2000109EUL)[seg_off] << 24) |
                    ((uint32_t)RAM_PTR8(0x2000109FUL)[seg_off] << 16) |
                    ((uint32_t)RAM_PTR8(0x200010A0UL)[seg_off] << 8)  |
                    ((uint32_t)RAM_PTR8(0x200010A1UL)[seg_off]);
            }
        }
        param_1 = RAM32(0x200003FCUL);
        FUN_08016e8c(RAM32(0x20000410UL), RAM32(0x200003F0UL),
                     RAM32(0x200003F4UL), RAM32(0x200003F8UL),
                     RAM32(0x200003FCUL));
        RAM32(0x20000400UL) = RAM32(0x200003F0UL);
        RAM32(0x200000E0UL) = 0;
        RAM32(0x200000E4UL) = 0;
        RAM32(0x200000E8UL) = 0;
        RAM32(0x200000ECUL) = 0;
        RAM32(0x20000380UL) = 0;
        RAM32(0x20000384UL) = 0;
        RAM32(0x20000084UL) = 0;
        RAM32(0x20000094UL) = RAM32(0x20000410UL);
        TIM2_CNT = 0;
        TIM1_CNT = 0;
        RAM8(0x20000192UL) = 0;
        RAM8(0x200003EDUL) = (RAM32(0x20000410UL) == 1UL) ? 1U : 0U;
        RAM8(0x20005F6AUL) &= 0x5FU;
        if (RAM8(0x200003EDUL) != 0) {
            TIM1_SR = 0xFFFEU;
            TIM1_DIER |= 1U;
        }
        TIM2_CR1 |= 1U;
        TIM1_CR1 |= 1U;
        TIM1_BDTR |= 0x8000U;
        RAM8(0x2000018EUL) = (uint8_t)RAM16(0x20000044UL);
        RAM16(0x20000190UL) = 0;
    }

    /* ----------------------------------------------------------------------
     * PTO1 (Q0.1) Next Envelope Segment Advance (triggered by 0x20000440 == 1)
     * ---------------------------------------------------------------------- */
    uint8_t smb76  = RAM8(0x20005F74UL);
    uint8_t sm_dc  = RAM8(0x20005FDCUL);
    uint8_t sm_dd  = RAM8(0x20005FDDUL);

    if (RAM8(0x20000440UL) == 1U) {
        RAM8(0x20000440UL) = 0;
        RAM8(0x20000029UL) = RAM8(0x20005F75UL) & 0x20U;

        if (RAM8(0x20000196UL) == 1U) {
            uint8_t next_seg = (uint8_t)(RAM8(0x20005FD8UL) + 1U);
            uint32_t total_segs = RAM16(0x2000045CUL);
            param_1 = total_segs;
            if (next_seg < total_segs) {
                uint32_t seg_off = (uint32_t)RAM16(0x20000460UL) + (uint32_t)next_seg * 10U;
                RAM16(0x2000045EUL) = RAM_PTR8(0x2000108EUL)[seg_off];
                uint8_t seg_cmd = RAM_PTR8(0x2000108FUL)[seg_off];
                RAM8(0x20000427UL) = seg_cmd;

                if (seg_cmd != 0x10U || ((uint32_t)next_seg + 1U) < total_segs) {
                    RAM8(0x20000195UL) = 10U;
                    if (seg_cmd == 0x04U || seg_cmd == 0x18U || seg_cmd == 0x10U) {
                        uint32_t raw_freq =
                            ((uint32_t)RAM_PTR8(0x20001090UL)[seg_off] << 24) |
                            ((uint32_t)RAM_PTR8(0x20001091UL)[seg_off] << 16) |
                            ((uint32_t)RAM_PTR8(0x20001092UL)[seg_off] << 8)  |
                            ((uint32_t)RAM_PTR8(0x20001093UL)[seg_off]);
                        uint32_t period = ((int32_t)raw_freq >> 11) * 100U;
                        if (period < RAM32(0x20000458UL)) period = RAM32(0x20000458UL);
                        if (period > RAM32(0x20000454UL)) period = RAM32(0x2000043CUL);
                        RAM32(0x20000448UL) = period;
                        RAM16(0x20000044UL) = 0x0BU;
                        RAM32(0x2000042CUL) = period;
                        RAM32(0x20000430UL) = period;
                    } else if (seg_cmd == 0x08U || seg_cmd == 0x00U) {
                        uint32_t raw_freq =
                            ((uint32_t)RAM_PTR8(0x2000109AUL)[seg_off] << 24) |
                            ((uint32_t)RAM_PTR8(0x2000109BUL)[seg_off] << 16) |
                            ((uint32_t)RAM_PTR8(0x2000109CUL)[seg_off] << 8)  |
                            ((uint32_t)RAM_PTR8(0x2000109DUL)[seg_off]);
                        uint32_t period = ((int32_t)raw_freq >> 11) * 100U;
                        if (period < RAM32(0x20000458UL)) period = RAM32(0x20000458UL);
                        if (period > RAM32(0x20000454UL)) period = RAM32(0x2000043CUL);
                        RAM32(0x20000448UL) = period;
                        RAM16(0x20000044UL) = (seg_cmd == 0x08U) ? 0x0DU : 0x0FU;
                        RAM32(0x2000042CUL) = RAM32(0x2000043CUL);
                        RAM32(0x20000430UL) = period;
                    }

                    uint32_t pulses =
                        ((uint32_t)RAM_PTR8(0x20001094UL)[seg_off] << 24) |
                        ((uint32_t)RAM_PTR8(0x20001095UL)[seg_off] << 16) |
                        ((uint32_t)RAM_PTR8(0x20001096UL)[seg_off] << 8)  |
                        ((uint32_t)RAM_PTR8(0x20001097UL)[seg_off]);
                    RAM32(0x2000044CUL) = pulses;

                    if (seg_cmd == 0x18U) {
                        if (pulses != 0) {
                            goto pto1_seg_err;
                        }
                        RAM32(0x2000044CUL) = 180000UL;
                        RAM8(0x20005FDCUL) |= 0x04U;
                        RAM16(0x20000197UL) = 0;
                    }

                    if (RAM16(0x20000044UL) == 0x0FU && ((uint32_t)next_seg + 1U) < total_segs) {
                        RAM8(0x20000028UL) = RAM_PTR8(0x20001099UL)[seg_off];
                        if (RAM8(0x20000028UL) == 0x10U) {
                            RAM32(0x2000044CUL) +=
                                ((uint32_t)RAM_PTR8(0x2000109EUL)[seg_off] << 24) |
                                ((uint32_t)RAM_PTR8(0x2000109FUL)[seg_off] << 16) |
                                ((uint32_t)RAM_PTR8(0x200010A0UL)[seg_off] << 8)  |
                                ((uint32_t)RAM_PTR8(0x200010A1UL)[seg_off]);
                        }
                    }

                    RAM8(0x20005FD8UL) = next_seg;
                    param_1 = RAM32(0x20000438UL);
                    FUN_0801704e(RAM32(0x2000044CUL), RAM32(0x2000042CUL),
                                 RAM32(0x20000430UL), RAM32(0x20000434UL));
                    TIM5_CNT = 0;
                    TIM8_CNT = 0;
                    RAM8(0x20000428UL) = (RAM32(0x2000044CUL) == 1UL) ? 1U : 0U;
                    RAM8(0x20005F74UL) &= 0x5FU;
                    if (RAM8(0x20000428UL) != 0) {
                        TIM8_SR = 0xFFFEU;
                        TIM8_DIER |= 1U;
                    }
                    TIM5_CR1 |= 1U;
                    TIM8_CR1 |= 1U;
                    TIM8_BDTR |= 0x8000U;
                    RAM8(0x20000195UL) = (uint8_t)RAM16(0x20000044UL);
                    goto pto1_reset_accumulators;
                }
            }
        pto1_profile_done:
            RAM8(0x20005FD8UL) = next_seg;
            RAM8(0x20000195UL) = 0x10U;
            smb76 = RAM8(0x20005F74UL) | 0x80U;
            sm_dc = RAM8(0x20005FDCUL) & 0xFEU;
            sm_dd = RAM8(0x20005FDDUL) | 0x80U;
            if (RAM8(0x20000030UL) == 1U) {
                RAM8(0x20000032UL) = 1U;
                EXTI_SWIER |= 2UL;
            }
        } else {
            uint8_t next_seg = RAM8(0x20005FD8UL);
            if ((RAM8(0x20005F75UL) & 0x20U) == 0) {
                goto pto1_profile_done;
            }
            next_seg = (uint8_t)(next_seg + 1U);
            param_1 = RAM16(0x20000450UL);
            if (RAM_PTR8(0x2000108CUL)[param_1] <= next_seg) {
                goto pto1_profile_done;
            }
            RAM8(0x20000195UL) = 10U;
            RAM32(0x200000B0UL) += RAM32(0x20000388UL);
            uint32_t seg_off = param_1 + (uint32_t)next_seg * 8U;
            uint32_t pulses =
                ((uint32_t)RAM_PTR8(0x20001091UL)[seg_off] << 24) |
                ((uint32_t)RAM_PTR8(0x20001092UL)[seg_off] << 16) |
                ((uint32_t)RAM_PTR8(0x20001093UL)[seg_off] << 8)  |
                ((uint32_t)RAM_PTR8(0x20001094UL)[seg_off]);
            RAM32(0x2000044CUL) = pulses;

            if (pulses != 0) {
                uint16_t init_cyc = ((uint16_t)RAM_PTR8(0x2000108DUL)[seg_off] << 8) |
                                    RAM_PTR8(0x2000108EUL)[seg_off];
                RAM16(0x20000044UL) = init_cyc;
                if (init_cyc < 2U) {
                pto1_seg_err:
                    RAM8(0x20005F74UL) |= 0x80U;
                pto1_seg_err_common:
                    RAM8(0x20005FDDUL) |= 0x80U;
                    RAM8(0x20005FDCUL) &= 0xFEU;
                    RAM8(0x20005FD8UL) += 1U;
                    RAM8(0x20000195UL) = 0x10U;
                    smb76 = RAM8(0x20005F74UL);
                    sm_dc = RAM8(0x20005FDCUL);
                    sm_dd = RAM8(0x20005FDDUL);
                    goto pto1_update_status;
                }

                uint32_t timebase_num = (RAM8(0x20000426UL) == 0) ? 100000UL : 100000000UL;
                RAM32(0x2000042CUL) = timebase_num / init_cyc;
                int16_t delta_cyc = (int16_t)(((uint16_t)RAM_PTR8(0x2000108FUL)[seg_off] << 8) |
                                              RAM_PTR8(0x20001090UL)[seg_off]);
                RAM16(0x20000046UL) = (uint16_t)delta_cyc;
                RAM16(0x20000442UL) = init_cyc;

                uint32_t end_cyc = init_cyc;
                if (delta_cyc == 0) {
                    RAM16(0x20000452UL) = 0;
                    RAM32(0x200000D8UL) = 0;
                    RAM32(0x200000DCUL) = 0;
                } else {
                    uint32_t total_delta = pulses * (uint32_t)(int32_t)delta_cyc;
                    RAM32(0x200000D8UL) = total_delta;
                    RAM32(0x200000DCUL) = 0;
                    RAM16(0x20000452UL) = (uint16_t)delta_cyc;
                    end_cyc = (uint32_t)init_cyc + total_delta;
                    if (end_cyc >= 0x10000UL) {
                        RAM8(0x20005F74UL) |= 0x90U;
                        goto pto1_seg_err_common;
                    }
                }

                RAM16(0x20000044UL) = (uint16_t)end_cyc;
                RAM32(0x20000430UL) = timebase_num / (end_cyc & 0xFFFFU);
                if (RAM32(0x2000042CUL) < 100UL) {
                    RAM32(0x2000042CUL) = 100UL;
                } else if (RAM32(0x2000042CUL) > 20000000UL) {
                    RAM32(0x2000042CUL) = 20000000UL;
                }
                if (RAM32(0x20000430UL) < 100UL) {
                    RAM32(0x20000430UL) = 100UL;
                } else if (RAM32(0x20000430UL) > 20000000UL) {
                    RAM32(0x20000430UL) = 20000000UL;
                }

                uint64_t sum_prod = (uint64_t)(RAM32(0x20000430UL) + RAM32(0x2000042CUL)) *
                                    (uint64_t)pulses * 100000ULL;
                RAM8(0x20005FD8UL) = next_seg;
                param_2 = RAM32(0x20000430UL);
                uint32_t start_p = RAM32(0x2000042CUL);
                uint64_t div1 = FUN_0800016c((uint32_t)sum_prod, (uint32_t)(sum_prod >> 32),
                                             param_2, 0);
                uint64_t div2 = FUN_0800016c((uint32_t)div1, (uint32_t)(div1 >> 32),
                                             start_p, 0);
                RAM32(0x200000DCUL) = (uint32_t)(div2 >> 33);
                RAM32(0x200000D8UL) = (uint32_t)(div2 >> 1);
                RAM32(0x20000438UL) = 1UL;
                param_1 = 1UL;
                RAM32(0x20000434UL) = RAM32(0x200000D8UL);

                FUN_0801704e(pulses, start_p, param_2, RAM32(0x20000434UL));
                RAM8(0x20000195UL) = (RAM16(0x20000046UL) == 0) ? 0x0BU : 0x0EU;

            pto1_reset_accumulators:
                RAM32(0x2000038CUL) = 0;
                RAM32(0x20000388UL) = 0;
                RAM8(0x20000199UL)  = 0;
                RAM32(0x200000FCUL) = 0;
                RAM32(0x200000F8UL) = 0;
                RAM32(0x200000F4UL) = 0;
                RAM32(0x200000F0UL) = 0;
                RAM32(0x2000009CUL) = 0;
                RAM32(0x200000ACUL) = RAM32(0x2000044CUL);
                RAM32(0x2000043CUL) = RAM32(0x2000042CUL);
                TIM5_CNT = 0;
                TIM8_CNT = 0;
                RAM8(0x20000428UL) = (RAM32(0x2000044CUL) == 1UL) ? 1U : 0U;
                RAM8(0x20005F74UL) &= 0x5FU;
                if (RAM8(0x20000428UL) != 0) {
                    TIM8_SR = 0xFFFEU;
                    TIM8_DIER |= 1U;
                }
                TIM5_CR1 |= 1U;
                TIM8_CR1 |= 1U;
                TIM8_BDTR |= 0x8000U;
                smb76 = RAM8(0x20005F74UL);
                sm_dc = RAM8(0x20005FDCUL);
                sm_dd = RAM8(0x20005FDDUL);
                goto pto1_update_status;
            }
            RAM8(0x20000440UL) = 1U;
            RAM8(0x20005FD8UL) = next_seg;
            RAM8(0x20005FDDUL) |= 0x80U;
            RAM8(0x20005FDCUL) &= 0xFEU;
            RAM8(0x20005F74UL) |= 0x80U;
            RAM8(0x20000195UL) = 0x10U;
            EXTI_SWIER |= 4UL;
            smb76 = RAM8(0x20005F74UL);
            sm_dc = RAM8(0x20005FDCUL);
            sm_dd = RAM8(0x20005FDDUL);
        }
    }

pto1_update_status:
    RAM8(0x20005FDDUL) = sm_dd;
    RAM8(0x20005FDCUL) = sm_dc;
    RAM8(0x20005F74UL) = smb76;

    /* PTO1 Direct Reload (0x20000441 == 1) */
    if (RAM8(0x20000441UL) == 1U) {
        RAM8(0x20000441UL) = 0;
        uint32_t seg_off = (uint32_t)RAM16(0x20000460UL) + (uint32_t)RAM8(0x20005FD8UL) * 10U;
        if (RAM8(0x20000427UL) == 0x04U) {
            uint32_t raw_freq =
                ((uint32_t)RAM_PTR8(0x20001090UL)[seg_off] << 24) |
                ((uint32_t)RAM_PTR8(0x20001091UL)[seg_off] << 16) |
                ((uint32_t)RAM_PTR8(0x20001092UL)[seg_off] << 8)  |
                ((uint32_t)RAM_PTR8(0x20001093UL)[seg_off]);
            uint32_t period = ((int32_t)raw_freq >> 11) * 100U;
            if (period < RAM32(0x20000458UL)) period = RAM32(0x20000458UL);
            if (period > RAM32(0x20000454UL)) period = RAM32(0x2000043CUL);
            RAM32(0x20000448UL) = period;
            RAM16(0x20000044UL) = 0x0BU;
            RAM32(0x2000042CUL) = period;
            RAM32(0x20000430UL) = period;
        } else if (RAM8(0x20000427UL) == 0x00U) {
            uint32_t raw_freq =
                ((uint32_t)RAM_PTR8(0x2000109AUL)[seg_off] << 24) |
                ((uint32_t)RAM_PTR8(0x2000109BUL)[seg_off] << 16) |
                ((uint32_t)RAM_PTR8(0x2000109CUL)[seg_off] << 8)  |
                ((uint32_t)RAM_PTR8(0x2000109DUL)[seg_off]);
            uint32_t period = ((int32_t)raw_freq >> 11) * 100U;
            if (period < RAM32(0x20000458UL)) period = RAM32(0x20000458UL);
            if (period > RAM32(0x20000454UL)) period = RAM32(0x2000043CUL);
            RAM32(0x20000448UL) = period;
            RAM32(0x2000042CUL) = RAM32(0x2000043CUL);
            RAM32(0x20000430UL) = period;
            RAM16(0x20000044UL) = 0x0FU;
        }
        uint32_t pulses =
            ((uint32_t)RAM_PTR8(0x20001094UL)[seg_off] << 24) |
            ((uint32_t)RAM_PTR8(0x20001095UL)[seg_off] << 16) |
            ((uint32_t)RAM_PTR8(0x20001096UL)[seg_off] << 8)  |
            ((uint32_t)RAM_PTR8(0x20001097UL)[seg_off]);
        RAM32(0x2000044CUL) = pulses;
        if (RAM16(0x20000044UL) == 0x0FU && (uint32_t)(RAM8(0x20005FD8UL) + 1U) < RAM16(0x2000045CUL)) {
            RAM8(0x20000028UL) = RAM_PTR8(0x20001099UL)[seg_off];
            if (RAM8(0x20000028UL) == 0x10U) {
                RAM32(0x2000044CUL) +=
                    ((uint32_t)RAM_PTR8(0x2000109EUL)[seg_off] << 24) |
                    ((uint32_t)RAM_PTR8(0x2000109FUL)[seg_off] << 16) |
                    ((uint32_t)RAM_PTR8(0x200010A0UL)[seg_off] << 8)  |
                    ((uint32_t)RAM_PTR8(0x200010A1UL)[seg_off]);
            }
        }
        param_1 = RAM32(0x20000438UL);
        FUN_0801704e(RAM32(0x2000044CUL), RAM32(0x2000042CUL),
                     RAM32(0x20000430UL), RAM32(0x20000434UL));
        RAM32(0x2000043CUL) = RAM32(0x2000042CUL);
        RAM32(0x200000F0UL) = 0;
        RAM32(0x200000F4UL) = 0;
        RAM32(0x200000F8UL) = 0;
        RAM32(0x200000FCUL) = 0;
        RAM32(0x20000388UL) = 0;
        RAM32(0x2000038CUL) = 0;
        RAM32(0x2000009CUL) = 0;
        RAM32(0x200000ACUL) = RAM32(0x2000044CUL);
        TIM5_CNT = 0;
        TIM8_CNT = 0;
        RAM8(0x20000199UL) = 0;
        RAM8(0x20000428UL) = (RAM32(0x2000044CUL) == 1UL) ? 1U : 0U;
        RAM8(0x20005F74UL) &= 0x5FU;
        if (RAM8(0x20000428UL) != 0) {
            TIM8_SR = 0xFFFEU;
            TIM8_DIER |= 1U;
        }
        TIM5_CR1 |= 1U;
        TIM8_CR1 |= 1U;
        TIM8_BDTR |= 0x8000U;
        RAM8(0x20000195UL) = (uint8_t)RAM16(0x20000044UL);
        RAM16(0x20000197UL) = 0;
    }

    return ((uint64_t)param_2 << 32) | param_1;
}

/* ==========================================================================
 * Function: FUN_08011ee2 (EXTI4_IRQHandler) @ 0x08011ee2 (858 bytes)
 *
 * Handles EXTI Line 4 (PB4 = I0.6): HSC2 Phase A Clock Input
 *   - Supports modes 0..11 (encoded as 0x14..0x25 in 0x20000B84)
 *   - Updates HC2 (0x20006154), checks preset HC2 == PV (0x200061B0 -> event 0x0E)
 *     and direction change (0x200003CB -> event 0x0F).
 * ========================================================================== */
void FUN_08011ee2(void)
{
    if ((EXTI_PR & 0x0010UL) == 0) {
        return;
    }
    EXTI_PR = 0x0010UL;

    uint32_t hsc2_ctrl = RAM8(0x20000339UL);
    int32_t  hc2_inc   = HC2_CUR_VAL + 1;
    int32_t  hc2_dec   = HC2_CUR_VAL - 1;
    uint32_t gpiob_ab;

    switch (RAM8(0x20000B84UL)) {
    case 0x14U:
        HC2_CUR_VAL = hc2_inc;
        goto hsc2_check_events;

    case 0x15U:
        HC2_CUR_VAL = hc2_dec;
        goto hsc2_check_events;

    case 0x16U:
        break;

    case 0x17U:
        hc2_inc = hc2_dec;
        break;

    case 0x18U:
        if ((GPIOB_IDR & 0x0020UL) == 0) {
        hsc2_cnt_down:
            if (RAM8(0x200003C5UL) == 1U) {
                RAM8(0x200003CBUL) = 1U;
            }
            RAM8(0x200003C5UL) = 2U;
            HC2_CUR_VAL = hc2_dec;
            goto hsc2_check_events;
        }
        goto hsc2_cnt_up;

    case 0x19U:
        if ((GPIOB_IDR & 0x0020UL) != 0) {
            goto hsc2_cnt_up_with_reset;
        }
        goto hsc2_cnt_down_with_reset;

    case 0x1AU:
        goto hsc2_cnt_up;

    case 0x1BU:
        goto hsc2_cnt_up_with_reset;

    case 0x1CU:
        if (((GPIOB_IDR & 0x003FUL) >> 4) != 1U) {
            if ((GPIOB_IDR & 0x0030UL) == 0) {
                goto hsc2_cnt_down;
            }
            goto hsc2_check_events;
        }
        goto hsc2_cnt_up;

    case 0x1DU:
        gpiob_ab = (GPIOB_IDR & 0x003FUL) >> 4;
        if (gpiob_ab != 2U && gpiob_ab != 1U) {
            if ((GPIOB_IDR & 0x0030UL) != 0 && (((GPIOB_IDR & 0x003FUL) >> 4) != 3U)) {
                goto hsc2_check_events;
            }
            goto hsc2_cnt_down;
        }
    hsc2_cnt_up:
        if (RAM8(0x200003C5UL) == 2U) {
            RAM8(0x200003CBUL) = 1U;
        }
        RAM8(0x200003C5UL) = 1U;
        HC2_CUR_VAL = hc2_inc;
        goto hsc2_check_events;

    case 0x1EU:
        goto hsc2_quad_1x_reset;

    case 0x1FU:
        goto hsc2_quad_4x_reset;

    case 0x20U:
        if (((GPIOE_IDR & 0x0400UL) == 0) || ((hsc2_ctrl & 2U) != 0)) {
            if (((GPIOE_IDR & 0x0400UL) != 0) || ((hsc2_ctrl & 2U) == 0)) {
                goto hsc2_check_events;
            }
        }
        break;

    case 0x21U:
        if (((GPIOE_IDR & 0x0400UL) == 0) || ((hsc2_ctrl & 2U) != 0)) {
            hc2_inc = hc2_dec;
            if (((GPIOE_IDR & 0x0400UL) != 0) || ((hsc2_ctrl & 2U) == 0)) {
                goto hsc2_check_events;
            }
        } else {
            hc2_inc = hc2_dec;
        }
        break;

    case 0x22U:
        if ((((GPIOE_IDR & 0x0400UL) == 0) || ((hsc2_ctrl & 2U) != 0)) &&
            (((GPIOE_IDR & 0x0400UL) != 0) || ((hsc2_ctrl & 2U) == 0))) {
            goto hsc2_check_events;
        }
        if ((GPIOB_IDR & 0x0020UL) != 0) {
            if (RAM8(0x200003C5UL) == 2U) {
                RAM8(0x200003CBUL) = 1U;
            }
            RAM8(0x200003C5UL) = 1U;
            HC2_CUR_VAL = hc2_inc;
        } else {
            if (RAM8(0x200003C5UL) == 1U) {
                RAM8(0x200003CBUL) = 1U;
            }
            RAM8(0x200003C5UL) = 2U;
            HC2_CUR_VAL = hc2_dec;
        }
        goto hsc2_check_ext_reset;

    case 0x23U:
        if ((((GPIOE_IDR & 0x0400UL) == 0) || ((hsc2_ctrl & 2U) != 0)) &&
            (((GPIOE_IDR & 0x0400UL) != 0) || ((hsc2_ctrl & 2U) == 0))) {
            goto hsc2_check_events;
        }
        if (RAM8(0x200003C5UL) == 2U) {
            RAM8(0x200003CBUL) = 1U;
        }
        RAM8(0x200003C5UL) = 1U;
        break;

    case 0x24U:
        if ((((GPIOE_IDR & 0x0400UL) == 0) || ((hsc2_ctrl & 2U) != 0)) &&
            (((GPIOE_IDR & 0x0400UL) != 0) || ((hsc2_ctrl & 2U) == 0))) {
            goto hsc2_check_events;
        }
    hsc2_quad_1x_reset:
        if (((GPIOB_IDR & 0x003FUL) >> 4) == 1U) {
        hsc2_cnt_up_with_reset:
            if (RAM8(0x200003C5UL) == 2U) {
                RAM8(0x200003CBUL) = 1U;
            }
            RAM8(0x200003C5UL) = 1U;
        } else {
            hc2_inc = HC2_CUR_VAL;
            if ((GPIOB_IDR & 0x0030UL) == 0) {
            hsc2_cnt_down_with_reset:
                if (RAM8(0x200003C5UL) == 1U) {
                    RAM8(0x200003CBUL) = 1U;
                }
                RAM8(0x200003C5UL) = 2U;
                hc2_inc = hc2_dec;
            }
        }
        break;

    case 0x25U:
        if ((((GPIOE_IDR & 0x0400UL) == 0) || ((hsc2_ctrl & 2U) != 0)) &&
            (((GPIOE_IDR & 0x0400UL) != 0) || ((hsc2_ctrl & 2U) == 0))) {
            goto hsc2_check_events;
        }
    hsc2_quad_4x_reset:
        gpiob_ab = (GPIOB_IDR & 0x003FUL) >> 4;
        if (gpiob_ab == 2U || gpiob_ab == 1U) {
            goto hsc2_cnt_up_with_reset;
        }
        if ((GPIOB_IDR & 0x0030UL) != 0 && (((GPIOB_IDR & 0x003FUL) >> 4) != 3U)) {
            hc2_inc = HC2_CUR_VAL;
            break;
        }
        goto hsc2_cnt_down_with_reset;

    default:
        goto hsc2_check_events;
    }

    HC2_CUR_VAL = hc2_inc;
hsc2_check_ext_reset:
    if ((((GPIOE_IDR & 0x0100UL) != 0) && ((RAM8(0x20000339UL) & 1U) == 0)) ||
        (((GPIOE_IDR & 0x0100UL) == 0) && ((RAM8(0x20000339UL) & 1U) != 0))) {
        HC2_CUR_VAL = 0;
    }

hsc2_check_events:
    if (RAM8(0x20000345UL) == 1U && HC2_CUR_VAL == HC2_PRESET_VAL) {
        plc_io_int_enqueue(0x0EU); /* Event 14: HSC2 CV == PV */
    }
    if (RAM8(0x200003D7UL) == 1U && RAM8(0x200003CBUL) == 1U) {
        RAM8(0x200003CBUL) = 0;
        plc_io_int_enqueue(0x0FU); /* Event 15: HSC2 Direction Changed */
    }
}

/* ==========================================================================
 * Function: FUN_0801223c (EXTI9_5_IRQHandler) @ 0x0801223c (1972 bytes)
 *
 * Multiplexed handler for EXTI Lines 5..9:
 *   - EXTI9 (0x0200, PE9): HSC4 Phase A Clock Input (HC4 @ 0x20006158,
 *     events 0x11 [HC4=PV] & 0x12 [HSC4 Dir Change])
 *   - EXTI5 (0x0020, PB5): HSC2 Phase B Clock Input (HC2 @ 0x20006154,
 *     events 0x0E [HC2=PV] & 0x0F [HSC2 Dir Change])
 *   - EXTI6 (0x0040, PC6): Digital Input I0.2 Rising (7) / Falling (8) Edge
 *   - EXTI8 (0x0100, PE8): HSC2 External Reset Input (event 0x10)
 *   - EXTI7 (0x0080, PC7): HSC1 Hardware Direction Input (updates TIM3_CR1.DIR)
 * ========================================================================== */
void FUN_0801223c(void)
{
    uint32_t gpiob_ab;

    /* ----------------------------------------------------------------------
     * 1. EXTI9 (0x0200, PE9): HSC4 Phase A Clock Input
     * ---------------------------------------------------------------------- */
    if ((EXTI_PR & 0x0200UL) != 0) {
        EXTI_PR = 0x0200UL;
        int32_t  hc4_inc   = HC4_CUR_VAL + 1;
        int32_t  hc4_dec   = HC4_CUR_VAL - 1;
        uint32_t hsc4_ctrl = RAM8(0x2000033AUL);

        switch (RAM8(0x20000B8AUL)) {
        case 0x14U:
            HC4_CUR_VAL = hc4_inc;
            break;

        case 0x15U:
            HC4_CUR_VAL = hc4_dec;
            break;

        case 0x16U:
            hc4_dec = hc4_inc;
            goto hsc4_pha_apply_reset_check;

        case 0x17U:
            goto hsc4_pha_apply_reset_check;

        case 0x18U:
            if ((GPIOE_IDR & 0x0800UL) != 0) {
                goto hsc4_pha_cnt_up;
            }
        hsc4_pha_cnt_down:
            if (RAM8(0x200003C6UL) == 1U) {
                RAM8(0x200003CCUL) = 1U;
            }
            RAM8(0x200003C6UL) = 2U;
            HC4_CUR_VAL = hc4_dec;
            break;

        case 0x19U:
        hsc4_pha_dir_with_reset:
            if ((GPIOE_IDR & 0x0800UL) == 0) {
                goto hsc4_pha_cnt_down_with_reset;
            }
        hsc4_pha_cnt_up_with_reset:
            if (RAM8(0x200003C6UL) == 2U) {
                RAM8(0x200003CCUL) = 1U;
            }
            RAM8(0x200003C6UL) = 1U;
            HC4_CUR_VAL = hc4_inc;
            goto hsc4_pha_ext_reset_only;

        case 0x1AU:
        hsc4_pha_cnt_up:
            if (RAM8(0x200003C6UL) == 2U) {
                RAM8(0x200003CCUL) = 1U;
            }
            RAM8(0x200003C6UL) = 1U;
            HC4_CUR_VAL = hc4_inc;
            break;

        case 0x1BU:
            if (RAM8(0x200003C6UL) == 2U) {
                RAM8(0x200003CCUL) = 1U;
            }
            RAM8(0x200003C6UL) = 1U;
            hc4_dec = hc4_inc;
            goto hsc4_pha_apply_reset_check;

        case 0x1CU:
            if ((GPIOE_IDR & 0x0A00UL) == 0x0200UL) {
                goto hsc4_pha_cnt_up;
            }
            if ((GPIOE_IDR & 0x0A00UL) == 0UL) {
                goto hsc4_pha_cnt_down;
            }
            break;

        case 0x1DU:
            if (((GPIOE_IDR & 0x0A00UL) == 0x0800UL) ||
                ((GPIOE_IDR & 0x0A00UL) == 0x0200UL)) {
                goto hsc4_pha_cnt_up;
            }
            if (((GPIOE_IDR & 0x0A00UL) == 0UL) ||
                ((~GPIOE_IDR & 0x0A00UL) == 0UL)) {
                goto hsc4_pha_cnt_down;
            }
            break;

        case 0x1EU:
        hsc4_pha_quad1x_reset:
            if ((GPIOE_IDR & 0x0A00UL) == 0x0200UL) {
                goto hsc4_pha_cnt_up_with_reset;
            }
            if ((GPIOE_IDR & 0x0A00UL) == 0UL) {
                goto hsc4_pha_cnt_down_with_reset;
            }
            goto hsc4_pha_ext_reset_only;

        case 0x1FU:
        hsc4_pha_quad4x_reset:
            if (((GPIOE_IDR & 0x0A00UL) == 0x0800UL) ||
                ((GPIOE_IDR & 0x0A00UL) == 0x0200UL)) {
                goto hsc4_pha_cnt_up_with_reset;
            }
            if (((GPIOE_IDR & 0x0A00UL) != 0UL) &&
                ((~GPIOE_IDR & 0x0A00UL) != 0UL)) {
                goto hsc4_pha_ext_reset_only;
            }
        hsc4_pha_cnt_down_with_reset:
            if (RAM8(0x200003C6UL) == 1U) {
                RAM8(0x200003CCUL) = 1U;
            }
            RAM8(0x200003C6UL) = 2U;
            HC4_CUR_VAL = hc4_dec;
        hsc4_pha_ext_reset_only:
            if ((((GPIOE_IDR & 0x1000UL) != 0) && ((RAM8(0x2000033AUL) & 1U) == 0)) ||
                (((GPIOE_IDR & 0x1000UL) == 0) && ((RAM8(0x2000033AUL) & 1U) != 0))) {
                HC4_CUR_VAL = 0;
            }
            break;

        case 0x20U:
            if ((((GPIOE_IDR & 0x2000UL) != 0) && ((hsc4_ctrl & 2U) == 0)) ||
                (((GPIOE_IDR & 0x2000UL) == 0) && ((hsc4_ctrl & 2U) != 0))) {
                hc4_dec = hc4_inc;
                goto hsc4_pha_apply_reset_check;
            }
            break;

        case 0x21U:
            if ((((GPIOE_IDR & 0x2000UL) != 0) && ((hsc4_ctrl & 2U) == 0)) ||
                (((GPIOE_IDR & 0x2000UL) == 0) && ((hsc4_ctrl & 2U) != 0))) {
            hsc4_pha_apply_reset_check:
                HC4_CUR_VAL = hc4_dec;
                goto hsc4_pha_ext_reset_only;
            }
            break;

        case 0x22U:
            if ((((GPIOE_IDR & 0x2000UL) != 0) && ((hsc4_ctrl & 2U) == 0)) ||
                (((GPIOE_IDR & 0x2000UL) == 0) && ((hsc4_ctrl & 2U) != 0))) {
                goto hsc4_pha_dir_with_reset;
            }
            break;

        case 0x23U:
            if ((((GPIOE_IDR & 0x2000UL) != 0) && ((hsc4_ctrl & 2U) == 0)) ||
                (((GPIOE_IDR & 0x2000UL) == 0) && ((hsc4_ctrl & 2U) != 0))) {
                goto hsc4_pha_cnt_up_with_reset;
            }
            break;

        case 0x24U:
            if ((((GPIOE_IDR & 0x2000UL) != 0) && ((hsc4_ctrl & 2U) == 0)) ||
                (((GPIOE_IDR & 0x2000UL) == 0) && ((hsc4_ctrl & 2U) != 0))) {
                goto hsc4_pha_quad1x_reset;
            }
            break;

        case 0x25U:
            if ((((GPIOE_IDR & 0x2000UL) != 0) && ((hsc4_ctrl & 2U) == 0)) ||
                (((GPIOE_IDR & 0x2000UL) == 0) && ((hsc4_ctrl & 2U) != 0))) {
                goto hsc4_pha_quad4x_reset;
            }
            break;

        default:
            break;
        }

        if (RAM8(0x20000346UL) == 1U && HC4_CUR_VAL == HC4_PRESET_VAL) {
            plc_io_int_enqueue(0x11U); /* Event 17: HSC4 CV == PV */
        }
        if (RAM8(0x200003D8UL) == 1U && RAM8(0x200003CCUL) == 1U) {
            RAM8(0x200003CCUL) = 0;
            plc_io_int_enqueue(0x12U); /* Event 18: HSC4 Direction Changed */
        }
    }

    /* ----------------------------------------------------------------------
     * 2. EXTI5 (0x0020, PB5): HSC2 Phase B Clock Input
     * ---------------------------------------------------------------------- */
    if ((EXTI_PR & 0x0020UL) != 0) {
        EXTI_PR = 0x0020UL;
        int32_t  hc2_dec   = HC2_CUR_VAL - 1;
        int32_t  hc2_inc   = HC2_CUR_VAL + 1;
        uint32_t hsc2_ctrl = RAM8(0x20000339UL);

        switch (RAM8(0x20000B85UL)) {
        case 0x1AU:
        hsc2_phb_cnt_down:
            if (RAM8(0x200003C5UL) == 1U) {
                RAM8(0x200003CBUL) = 1U;
            }
            RAM8(0x200003C5UL) = 2U;
            HC2_CUR_VAL = hc2_dec;
            break;

        case 0x1BU:
        hsc2_phb_cnt_down_with_reset:
            if (RAM8(0x200003C5UL) == 1U) {
                RAM8(0x200003CBUL) = 1U;
            }
            RAM8(0x200003C5UL) = 2U;
            HC2_CUR_VAL = hc2_dec;
            goto hsc2_phb_ext_reset_check;

        case 0x1DU:
            gpiob_ab = (GPIOB_IDR & 0x003FUL) >> 4;
            if (gpiob_ab == 2U || gpiob_ab == 1U) {
                goto hsc2_phb_cnt_down;
            }
            if ((GPIOB_IDR & 0x0030UL) == 0 || (((GPIOB_IDR & 0x003FUL) >> 4) == 3U)) {
                if (RAM8(0x200003C5UL) == 2U) {
                    RAM8(0x200003CBUL) = 1U;
                }
                RAM8(0x200003C5UL) = 1U;
                HC2_CUR_VAL = hc2_inc;
            }
            break;

        case 0x1FU:
            gpiob_ab = (GPIOB_IDR & 0x003FUL) >> 4;
            if (gpiob_ab == 2U || gpiob_ab == 1U) {
                goto hsc2_phb_cnt_down_with_reset;
            }
            if ((GPIOB_IDR & 0x0030UL) == 0 || (((GPIOB_IDR & 0x003FUL) >> 4) == 3U)) {
                if (RAM8(0x200003C5UL) == 2U) {
                    RAM8(0x200003CBUL) = 1U;
                }
                RAM8(0x200003C5UL) = 1U;
                HC2_CUR_VAL = hc2_inc;
            }
        hsc2_phb_ext_reset_check:
            if ((((GPIOE_IDR & 0x0100UL) != 0) && ((RAM8(0x20000339UL) & 1U) == 0)) ||
                (((GPIOE_IDR & 0x0100UL) == 0) && ((RAM8(0x20000339UL) & 1U) != 0))) {
                HC2_CUR_VAL = 0;
            }
            break;

        case 0x23U:
            if ((((GPIOE_IDR & 0x0400UL) == 0) || ((hsc2_ctrl & 2U) != 0)) &&
                (((GPIOE_IDR & 0x0400UL) != 0) || ((hsc2_ctrl & 2U) == 0))) {
                break;
            }
            if (RAM8(0x200003C5UL) == 1U) {
                RAM8(0x200003CBUL) = 1U;
            }
            RAM8(0x200003C5UL) = 2U;
            HC2_CUR_VAL = hc2_dec;
            goto hsc2_phb_ext_reset_check;

        case 0x25U:
            if ((((GPIOE_IDR & 0x0400UL) == 0) || ((hsc2_ctrl & 2U) != 0)) &&
                (((GPIOE_IDR & 0x0400UL) != 0) || ((hsc2_ctrl & 2U) == 0))) {
                break;
            }
            gpiob_ab = (GPIOB_IDR & 0x003FUL) >> 4;
            if (gpiob_ab == 2U || gpiob_ab == 1U) {
                if (RAM8(0x200003C5UL) == 1U) {
                    RAM8(0x200003CBUL) = 1U;
                }
                RAM8(0x200003C5UL) = 2U;
                HC2_CUR_VAL = hc2_dec;
            } else if ((GPIOB_IDR & 0x0030UL) == 0 || (((GPIOB_IDR & 0x003FUL) >> 4) == 3U)) {
                if (RAM8(0x200003C5UL) == 2U) {
                    RAM8(0x200003CBUL) = 1U;
                }
                RAM8(0x200003C5UL) = 1U;
                HC2_CUR_VAL = hc2_inc;
            }
            goto hsc2_phb_ext_reset_check;

        default:
            break;
        }

        if (RAM8(0x20000345UL) == 1U && HC2_CUR_VAL == HC2_PRESET_VAL) {
            plc_io_int_enqueue(0x0EU); /* Event 14: HSC2 CV == PV */
        }
        if (RAM8(0x200003D7UL) == 1U && RAM8(0x200003CBUL) == 1U) {
            RAM8(0x200003CBUL) = 0;
            plc_io_int_enqueue(0x0FU); /* Event 15: HSC2 Direction Changed */
        }
    }

    /* ----------------------------------------------------------------------
     * 3. EXTI6 (0x0040, PC6): Digital Input I0.2 Rising / Falling Edge ISR
     * ---------------------------------------------------------------------- */
    if ((EXTI_PR & 0x0040UL) != 0) {
        EXTI_PR = 0x0040UL;
        IO_INT_LOCK_FLAG = 1U;
        uint16_t port_c = FUN_08014520((volatile void *)GPIOC_BASE);
        RAM16(0x20000052UL) = port_c & 0x0040U;
        uint8_t mode_i02 = RAM8(0x20000B81UL);
        if (RAM16(0x20000052UL) == 0) {
            if ((mode_i02 & 2U) != 0) {
                plc_io_int_enqueue(8U); /* Event 8: I0.2 Falling Edge */
            }
        } else if ((mode_i02 & 1U) != 0) {
            plc_io_int_enqueue(7U); /* Event 7: I0.2 Rising Edge */
        }
        if (IO_INT_WRITE_IDX > 0x0FU) {
            IO_INT_WRITE_IDX = 0;
        }
        IO_INT_LOCK_FLAG = 0;
    }

    /* ----------------------------------------------------------------------
     * 4. EXTI8 (0x0100, PE8): HSC2 External Reset Input
     * ---------------------------------------------------------------------- */
    if ((EXTI_PR & 0x0100UL) != 0) {
        EXTI_PR = 0x0100UL;
        uint8_t rst_mode = RAM8(0x20000B88UL);
        uint32_t hsc2_ctrl = RAM8(0x20000339UL);
        if (rst_mode == 10U) {
            if ((hsc2_ctrl & 0x80U) != 0) {
                HC2_CUR_VAL = 0;
                if (RAM8(0x2000034BUL) == 1U) {
                    plc_io_int_enqueue(0x10U); /* Event 16: HSC2 External Reset */
                }
            }
        } else if (rst_mode == 11U && (hsc2_ctrl & 0x80U) != 0) {
            if (((GPIOE_IDR & 0x0400UL) == 0) || ((hsc2_ctrl & 2U) != 0)) {
                if (((GPIOE_IDR & 0x0400UL) == 0) && ((hsc2_ctrl & 2U) != 0)) {
                    HC2_CUR_VAL = 0;
                    if (RAM8(0x2000034BUL) == 1U) {
                        plc_io_int_enqueue(0x10U); /* Event 16: HSC2 External Reset */
                    }
                }
            } else {
                HC2_CUR_VAL = 0;
                if (RAM8(0x2000034BUL) == 1U) {
                    plc_io_int_enqueue(0x10U); /* Event 16: HSC2 External Reset */
                }
            }
        }
    }

    /* ----------------------------------------------------------------------
     * 5. EXTI7 (0x0080, PC7): HSC1 Hardware Direction Input (TIM3_CR1.DIR)
     * ---------------------------------------------------------------------- */
    if ((EXTI_PR & 0x0080UL) != 0) {
        EXTI_PR = 0x0080UL;
        if (RAM8(0x20000B82UL) == 0x3CU) {
            uint16_t cr1 = TIM3_CR1;
            if ((GPIOC_IDR & 0x0080UL) != 0) {
                cr1 |= 0x0010U; /* DIR = 1 (downcounting) */
            } else {
                cr1 &= (uint16_t)~0x0010U; /* DIR = 0 (upcounting) */
            }
            TIM3_CR1 = cr1;
        }
    }
}

/* ==========================================================================
 * Function: FUN_080129f0 (EXTI_ClearITPendingBit) @ 0x080129f0 (12 bytes)
 * ========================================================================== */
void FUN_080129f0(uint32_t exti_line)
{
    EXTI_PR = exti_line;
}

/* ==========================================================================
 * Function: FUN_080129fc (EXTI_GetITStatus) @ 0x080129fc (38 bytes)
 * ========================================================================== */
uint32_t FUN_080129fc(uint32_t exti_line)
{
    uint32_t imr = EXTI_IMR;
    uint32_t pr  = EXTI_PR;
    if (((pr & exti_line) != 0) && ((imr & exti_line) != 0)) {
        return 1U;
    }
    return 0U;
}

/* ==========================================================================
 * Function: FUN_08012a22 (EXTI_Init) @ 0x08012a22 (138 bytes)
 *
 * Standard STM32F10x / APM32E103 EXTI_Init(&EXTI_InitStruct):
 *   param_1[0] (offset 0, uint32_t): EXTI_Line bitmask
 *   offset 4   (uint8_t):            EXTI_Mode (0x00 = Interrupt, 0x04 = Event)
 *   offset 5   (uint8_t):            EXTI_Trigger (0x08 = Rising, 0x0C = Falling,
 *                                                  0x10 = Rising_Falling)
 *   offset 6   (uint8_t):            EXTI_LineCmd (0 = DISABLE, non-zero = ENABLE)
 * ========================================================================== */
void FUN_08012a22(uint32_t *param_1)
{
    uint32_t line_mask = param_1[0];
    uint8_t  mode_off  = *(uint8_t *)((uintptr_t)param_1 + 4U);
    uint8_t  trig_off  = *(uint8_t *)((uintptr_t)param_1 + 5U);
    uint8_t  line_cmd  = *(uint8_t *)((uintptr_t)param_1 + 6U);

    if (line_cmd == 0) {
        volatile uint32_t *reg = (volatile uint32_t *)(EXTI_BASE + mode_off);
        *reg &= ~line_mask;
    } else {
        EXTI_IMR &= ~line_mask;
        EXTI_EMR &= ~line_mask;
        volatile uint32_t *mode_reg = (volatile uint32_t *)(EXTI_BASE + mode_off);
        *mode_reg |= line_mask;

        EXTI_RTSR &= ~line_mask;
        EXTI_FTSR &= ~line_mask;
        if (trig_off != 0x10U) {
            volatile uint32_t *trig_reg = (volatile uint32_t *)(EXTI_BASE + trig_off);
            *trig_reg |= line_mask;
        } else {
            EXTI_RTSR |= line_mask;
            EXTI_FTSR |= line_mask;
        }
    }
}

/* ==========================================================================
 * Function: FUN_08012aac (ExpBus_Transfer16BitCmd_Read3BitAck) @ 0x08012aac (2202 bytes)
 *
 * Bit-banged synchronous serial transaction on the S7-200 Expansion Module Bus
 * (GPIOC pins PC8=SCK, PC9=CTRL1, PC10=DATA I/O, PC11=CS):
 *   1. Configures PC10 as 50MHz Push-Pull Output (GPIOC_CRH bits 11:8 = 0x3)
 *   2. Shifts out 16-bit command `cmd_word` MSB-first on PC10 (clocked by PC8),
 *      inserting `delay_loops` software delay ticks (`DAT_20000139++`) per phase
 *   3. Switches PC10 to Input with Pull-Up/Down (GPIOC_CRH bits 11:8 = 0x8)
 *   4. Clocks in a 3-bit acknowledge/status code (bits 2..0) from PC10
 *   5. Restores PC10 to Push-Pull Output and releases bus lines
 * ========================================================================== */
uint32_t FUN_08012aac(uint16_t cmd_word, uint8_t delay_loops)
{
    uint32_t rx_ack = 0;

    /* Configure PC10 as 50MHz Push-Pull Output */
    GPIOC_CRH = (GPIOC_CRH & 0xFFFFF0FFUL) | 0x00000300UL;
    BITBAND_GPIOC_ODR_PC9  = 1UL;
    BITBAND_GPIOC_ODR_PC11 = 1UL;
    for (uint8_t d = 0; d < delay_loops; d++) {
        RAM8(0x20000139UL) += 1U;
    }

    /* Shift out 16 bits MSB-first on PC10 with clock on PC8 */
    for (int bit = 15; bit >= 0; bit--) {
        BITBAND_GPIOC_ODR_PC8 = 1UL;
        for (uint8_t d = 0; d < delay_loops; d++) {
            RAM8(0x20000139UL) += 1U;
        }
        BITBAND_GPIOC_ODR_PC10 = ((cmd_word >> bit) & 1U) ? 1UL : 0UL;
        for (uint8_t d = 0; d < delay_loops; d++) {
            RAM8(0x20000139UL) += 1U;
        }
        BITBAND_GPIOC_ODR_PC8 = 0UL;
        for (uint8_t d = 0; d < delay_loops; d++) {
            RAM8(0x20000139UL) += 1U;
        }
    }

    /* Switch PC10 to Input with Pull-Up/Pull-Down (0x8) to read 3-bit response */
    GPIOC_CRH = (GPIOC_CRH & 0xFFFFF0FFUL) | 0x00000800UL;

    for (uint32_t mask = 4U; mask != 0; mask >>= 1) {
        BITBAND_GPIOC_ODR_PC8 = 1UL;
        for (uint8_t d = 0; d < delay_loops; d++) {
            RAM8(0x20000139UL) += 1U;
        }
        BITBAND_GPIOC_ODR_PC8 = 0UL;
        for (uint8_t d = 0; d < delay_loops; d++) {
            RAM8(0x20000139UL) += 1U;
        }
        if ((GPIOC_IDR & 0x0400UL) != 0) {
            rx_ack |= mask;
        }
    }

    BITBAND_GPIOC_ODR_PC11 = 0UL;
    for (uint8_t d = 0; d < 19U; d++) {
        RAM8(0x20000139UL) += 1U;
    }

    /* Restore PC10 to Push-Pull Output and drive bus low */
    GPIOC_CRH = (GPIOC_CRH & 0xFFFFF0FFUL) | 0x00000300UL;
    BITBAND_GPIOC_ODR_PC10 = 0UL;
    BITBAND_GPIOC_ODR_PC8  = 0UL;
    return rx_ack;
}

/* ==========================================================================
 * Function: FUN_08013346 (ExpBus_Transfer8BitCmd_Read12BitData) @ 0x08013346 (2390 bytes)
 *
 * Bit-banged synchronous serial read transaction on the Expansion Module Bus:
 *   1. Configures PC10 as 50MHz Push-Pull Output
 *   2. Shifts out 8-bit command `cmd_byte` MSB-first on PC10 (clocked by PC8)
 *   3. Switches PC10 to Input with Pull-Up/Down
 *   4. Clocks in a 12-bit response word (bits 11..0, 0x800 down to 0x001)
 *      containing expansion module ID / status / digital or analog I/O data
 *   5. Restores PC10 to Push-Pull Output and releases bus lines
 * ========================================================================== */
uint32_t FUN_08013346(uint8_t cmd_byte, uint8_t delay_loops)
{
    uint32_t rx_word = 0;

    /* Configure PC10 as 50MHz Push-Pull Output */
    GPIOC_CRH = (GPIOC_CRH & 0xFFFFF0FFUL) | 0x00000300UL;
    BITBAND_GPIOC_ODR_PC9  = 1UL;
    BITBAND_GPIOC_ODR_PC11 = 1UL;
    for (uint8_t d = 0; d < delay_loops; d++) {
        RAM8(0x20000139UL) += 1U;
    }

    /* Shift out 8-bit command MSB-first on PC10 */
    for (int bit = 7; bit >= 0; bit--) {
        BITBAND_GPIOC_ODR_PC8 = 1UL;
        for (uint8_t d = 0; d < delay_loops; d++) {
            RAM8(0x20000139UL) += 1U;
        }
        BITBAND_GPIOC_ODR_PC10 = ((cmd_byte >> bit) & 1U) ? 1UL : 0UL;
        for (uint8_t d = 0; d < delay_loops; d++) {
            RAM8(0x20000139UL) += 1U;
        }
        BITBAND_GPIOC_ODR_PC8 = 0UL;
        for (uint8_t d = 0; d < delay_loops; d++) {
            RAM8(0x20000139UL) += 1U;
        }
    }

    /* Switch PC10 to Input with Pull-Up/Pull-Down (0x8) to read 12-bit response */
    GPIOC_CRH = (GPIOC_CRH & 0xFFFFF0FFUL) | 0x00000800UL;

    for (uint32_t mask = 0x0800UL; mask != 0; mask >>= 1) {
        BITBAND_GPIOC_ODR_PC8 = 1UL;
        for (uint8_t d = 0; d < delay_loops; d++) {
            RAM8(0x20000139UL) += 1U;
        }
        BITBAND_GPIOC_ODR_PC8 = 0UL;
        for (uint8_t d = 0; d < delay_loops; d++) {
            RAM8(0x20000139UL) += 1U;
        }
        if ((GPIOC_IDR & 0x0400UL) != 0) {
            rx_word |= mask;
        }
    }

    for (uint8_t d = 0; d < 56U; d++) {
        RAM8(0x20000139UL) += 1U;
    }
    BITBAND_GPIOC_ODR_PC11 = 0UL;

    /* Restore PC10 to Push-Pull Output and drive bus low */
    GPIOC_CRH = (GPIOC_CRH & 0xFFFFF0FFUL) | 0x00000300UL;
    BITBAND_GPIOC_ODR_PC10 = 0UL;
    BITBAND_GPIOC_ODR_PC8  = 0UL;
    return rx_word;
}

/* ==========================================================================
 * Function: FUN_08013c9c (FLASH_ClearFlag) @ 0x08013c9c (12 bytes)
 * ========================================================================== */
void FUN_08013c9c(uint32_t flash_flag)
{
    FLASH_SR = flash_flag;
}
